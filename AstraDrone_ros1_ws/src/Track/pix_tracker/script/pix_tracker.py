#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import PositionTarget, State
from cv_bridge import CvBridge
import threading

hight = 2.0 

class PIDController:
    """简单的PID控制器"""
    def __init__(self, kp=0.5, ki=0.1, kd=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.Time.now()
        
    def update(self, error):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            dt = 0.01  # 防止除零
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        print("P = ", self.kp * error, "i = ", self.ki * self.integral, "D = ", self.kd * derivative)
        self.prev_error = error
        self.last_time = current_time
        return output
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.Time.now()

class DroneTracker:
    def __init__(self):
        rospy.init_node('drone_tracker', anonymous=True)

        self.bridge = CvBridge()
        self.current_image = None
        self.target_pixel = None
        self.image_center = None
        self.is_offboard = False
        self.current_pose = None

        self.image_width = 640
        self.image_height = 480

        self.max_velocity = rospy.get_param('~control/max_velocity', 1.0)
        global hight
        hight = rospy.get_param('~control/target_height', 2.0)
        self.deadzone_pixels = rospy.get_param('~control/deadzone_pixels', 20)

        image_topic = rospy.get_param('~image_topic', '/iris_mid360/camera/rgb/image_raw')
        target_topic = rospy.get_param('~target_topic', '/target_pixel')
        cmd_topic = rospy.get_param('~cmd_topic', '/mavros/setpoint_raw/local')
        state_topic = rospy.get_param('~state_topic', '/mavros/state')

        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.target_sub = rospy.Subscriber(target_topic, Point, self.target_callback)
        self.state_sub = rospy.Subscriber(state_topic, State, self.state_callback)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        self.cmd_pub = rospy.Publisher(cmd_topic, PositionTarget, queue_size=10)
        self.position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        kp = rospy.get_param('~pid/kp', 0.001)
        ki = rospy.get_param('~pid/ki', 0.00001)
        kd = rospy.get_param('~pid/kd', 0.0005)

        self.pid_x = PIDController(kp=kp, ki=ki, kd=kd)
        self.pid_y = PIDController(kp=kp, ki=ki, kd=kd)

        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)  # 20Hz
        self.lock = threading.Lock()
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.current_image = cv_image
                self.image_height, self.image_width = cv_image.shape[:2]
                self.image_center = (self.image_width // 2, self.image_height // 2)
        except Exception as e:
            rospy.logerr(f"图像处理错误: {e}")
    
    def target_callback(self, msg):
        with self.lock:
            self.target_pixel = (int(msg.x), int(msg.y))
    
    def state_callback(self, msg):
        self.is_offboard = (msg.mode == "OFFBOARD")
    
    def pose_callback(self, msg):
        with self.lock:
            self.current_pose = msg
    
    def calculate_control_velocities(self):
        if self.target_pixel is None or self.image_center is None:
            return 0.0, 0.0
        error_u = self.target_pixel[0] - self.image_center[0]
        error_v = self.target_pixel[1] - self.image_center[1]
        if abs(error_u) < self.deadzone_pixels and abs(error_v) < self.deadzone_pixels:
            return 0.0, 0.0
        print("vel_x pid:")
        vel_x = self.pid_x.update(-error_v)
        print("vel_y pid:")
        vel_y = self.pid_y.update(-error_u)
        vel_x = max(-self.max_velocity, min(self.max_velocity, vel_x))
        vel_y = max(-self.max_velocity, min(self.max_velocity, vel_y))
        return vel_x, vel_y
    
    def create_position_target_msg(self, vel_x=0.0, vel_y=0.0, vel_z=0.0, z_pos=hight):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (PositionTarget.IGNORE_PX | 
                         PositionTarget.IGNORE_PY |
                         PositionTarget.IGNORE_AFX |
                         PositionTarget.IGNORE_AFY |
                         PositionTarget.IGNORE_AFZ |
                         PositionTarget.IGNORE_YAW |
                         PositionTarget.IGNORE_YAW_RATE)
        msg.position.z = z_pos
        msg.velocity.x = vel_x
        msg.velocity.y = vel_y
        msg.velocity.z = vel_z
        return msg

    def control_loop(self, event):
        try:
            if self.is_offboard:
                vel_x, vel_y = self.calculate_control_velocities()
                cmd_msg = self.create_position_target_msg(
                    vel_x=vel_x, 
                    vel_y=vel_y, 
                    vel_z=0.0, 
                    z_pos=hight
                )
                self.cmd_pub.publish(cmd_msg)
            else:
                with self.lock:
                    if self.current_pose is not None:
                        hold_pose = PoseStamped()
                        hold_pose.header.stamp = rospy.Time.now()
                        hold_pose.header.frame_id = "map"
                        hold_pose.pose = self.current_pose.pose
                        self.position_pub.publish(hold_pose)
                self.pid_x.reset()
                self.pid_y.reset()
        except Exception as e:
            rospy.logerr(f"控制循环错误: {e}")
    
    def run(self):
        rospy.loginfo("无人机追踪控制器运行中...")
        rospy.loginfo("发布目标像素坐标到话题: /target_pixel")
        rospy.loginfo("消息格式: geometry_msgs/Point (x: 像素x坐标, y: 像素y坐标, z: 未使用)")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("无人机追踪控制器停止")

def main():
    try:
        tracker = DroneTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"程序错误: {e}")

if __name__ == '__main__':
    main()
