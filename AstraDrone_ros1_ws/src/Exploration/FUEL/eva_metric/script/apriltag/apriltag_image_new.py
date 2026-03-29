#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import tf
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import apriltag

# Global variables
bridge = CvBridge()
camera_matrix = None
dist_coeffs = None
odom_pose = None  # UAV 位置和姿态
tag_size_ = 0.6  # AprilTag 码的真实尺寸（单位：米）

def camera_info_callback(msg):
    """ 订阅相机参数信息 """
    global camera_matrix, dist_coeffs
    camera_matrix = np.array(msg.K).reshape(3, 3)
    dist_coeffs = np.array(msg.D)

def odom_callback(msg):
    """ 订阅无人机位姿（世界坐标系下） """
    global odom_pose
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    odom_pose = (position, orientation)

def rgb_image_callback(msg):
    """ 处理相机图像，检测 AprilTag 并计算世界坐标 """
    global camera_matrix, dist_coeffs, odom_pose
    
    if camera_matrix is None or dist_coeffs is None or odom_pose is None:
        return  # 等待所有数据就绪
    
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11', quad_decimate=1.0, nthreads=4))
    detections = detector.detect(gray)
    
    if detections:
        for detection in detections:
            tag_corners = np.array(detection.corners, dtype=np.float32)

            # 计算 AprilTag 角点的 3D 位置
            half_size = tag_size_ / 2
            object_points = np.array([
                [-half_size, -half_size, 0],
                [ half_size, -half_size, 0],
                [ half_size,  half_size, 0],
                [-half_size,  half_size, 0]
            ], dtype=np.float32)
            
            ret, rvec, tvec = cv2.solvePnP(object_points, tag_corners, camera_matrix, dist_coeffs)
            if ret:
                tag_position_camera = tvec.flatten()  # 相机坐标系下的位置
                # 相机坐标系修正为FCU坐标系 （xyz定义替换）
                # 相机坐标系 → FCU坐标系
                tag_position_fcu = np.array([
                    tag_position_camera[2],   # x_fcu = z_cam
                    -tag_position_camera[0],  # y_fcu = -x_cam
                    -tag_position_camera[1]   # z_fcu = -y_cam
                ])

                tag_position_world = transform_to_world(tag_position_fcu)
                publish_tag_pose(tag_position_world)

def transform_to_world(tag_position_camera):
    """ 将相机坐标系下的 AprilTag 位置转换到世界坐标系 """
    global odom_pose

    # print(f"tag_camera: {tag_position_camera}")

    # 1. 相机坐标系 -> 飞控中心 (FCU)
    leftcam2fcu_xyz = np.array([0.12, 0.00, 0.0])  # 左相机在 FCU 坐标系下的偏移
    # for cam 4 test  
    # for front  0.12, 0.00, 0.2  ok
    # leftcam2fcu_xyz = np.array([0.12, 0.00, 0.2])  # 左相机在 FCU 坐标系下的偏移
    # for back  wrong！！！ 没有考虑旋转
    # leftcam2fcu_xyz = np.array([-0.12, 0.00, 0.2])  # 左相机在 FCU 坐标系下的偏移

    tag_position_fcu = tag_position_camera + leftcam2fcu_xyz  # 平移到 FCU

    # print(f"tag_fcu: {tag_position_fcu}")

    # 2. FCU 坐标系 -> 世界坐标系
    position, orientation = odom_pose
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    rotation_matrix = tf.transformations.quaternion_matrix(quaternion)[:3, :3]

    # 先旋转后平移
    tag_position_world = np.dot(rotation_matrix, tag_position_fcu) + np.array([position.x, position.y, position.z])

    return tag_position_world

def publish_tag_pose(tag_position_world):
    """ 发布 AprilTag 在世界坐标系中的位置 """
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world'
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = tag_position_world
    apritag_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('apriltag_detection_node', anonymous=True)
    
    rospy.Subscriber('/camera/left/image_raw', Image, rgb_image_callback)
    rospy.Subscriber('/camera/left/camera_info', CameraInfo, camera_info_callback)
    rospy.Subscriber('/ground_truth/odom', Odometry, odom_callback)
    
    apritag_pub = rospy.Publisher("/apritag_pose", PoseStamped, queue_size=10)
    
    rospy.spin()
