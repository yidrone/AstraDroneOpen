#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

from siyiA8mini import siyisdk

# 初始化SDK，连接相机
siyi = siyisdk.SIYISDK("192.168.1.25", 37260, 1024)

# 云台角度限制（按你给的文档）
MIN_YAW, MAX_YAW = -135.0, 135.0     # yaw: 左右
MIN_PITCH, MAX_PITCH = -90.0, 25.0   # pitch: 上下

# ====== 超时回中相关全局变量 ======
last_cmd_time = None        # 最近一次收到指令的时间（rospy.get_time()）
has_auto_centered = False   # 是否已经因为超时自动回中过一次
cmd_timeout = 15.0           # 超时时间（秒），会在 main() 里用参数覆盖


def clamp(value, min_val, max_val):
    """简单限幅函数"""
    return max(min_val, min(max_val, value))


def gimbal_cmd_callback(msg):
    """
    订阅 /gimbal/cmd_angle 的回调
    msg.x = pitch
    msg.y = yaw
    """
    global last_cmd_time, has_auto_centered

    # 记录收到指令的时间，并清除"已自动回中"标记
    last_cmd_time = rospy.get_time()
    has_auto_centered = False

    # 取出指令
    pitch_cmd = msg.x
    yaw_cmd   = msg.y

    # 做限幅，避免超出云台机械范围
    pitch_cmd = clamp(pitch_cmd, MIN_PITCH, MAX_PITCH)
    yaw_cmd   = clamp(yaw_cmd, MIN_YAW, MAX_YAW)

    rospy.loginfo("Recv gimbal cmd: pitch=%.2f, yaw=%.2f (deg)", pitch_cmd, yaw_cmd)

    # 调用你已有的控制函数：注意参数顺序 turn_to(yaw, pitch)
    try:
        siyi.turn_to(yaw_cmd, pitch_cmd)
    except Exception as e:
        rospy.logerr("call turn_to() failed: %s", str(e))


def publish_attitude_timer_cb(event):
    """
    定时器回调：以1Hz频率发布云台当前姿态
    发布消息类型: Float32MultiArray
    数据格式: [yaw, pitch]
    """
    try:
        # 获取姿态数据
        attitude = siyi.get_attitude()
        
        if attitude is not None:
            yaw, pitch = attitude
            
            # 确保角度在合理范围内
            if -135.0 <= yaw <= 135.0 and -90.0 <= pitch <= 25.0:
                # 量化到一位小数：先乘以10，取整，再除以10
                yaw_quantized = round(yaw * 10) / 10.0
                pitch_quantized = round(pitch * 10) / 10.0
                
                # 创建并发布消息
                attitude_msg = Float32MultiArray()
                attitude_msg.data = [yaw_quantized, pitch_quantized]
                
                # 发布到 /gimbal/attitude 话题
                attitude_pub.publish(attitude_msg)

            else:
                rospy.logwarn_throttle(5, "Invalid attitude values: yaw=%.1f°, pitch=%.1f°", yaw, pitch)
                
        else:
            rospy.logwarn_throttle(10, "Failed to get attitude from gimbal")
            
    except Exception as e:
        rospy.logerr_throttle(10, "Error in publish_attitude_timer_cb: %s", str(e))


def watchdog_timer_cb(event):
    """
    定时器回调：
    如果超过一定时间没有收到新的指令，则自动回中（调用 one_click_back）
    """
    global last_cmd_time, has_auto_centered, cmd_timeout

    # 如果 SDK 这边出问题，也没必要继续
    if siyi is None:
        return

    now = rospy.get_time()

    # 从未收到过指令的情况：可以选择忽略，或者也自动回中一次
    # 这里设计为：如果 last_cmd_time 是 None，则简单跳过，
    # 因为 main() 启动时已经做过一次回中。
    if last_cmd_time is None:
        return

    # 超时未收到新指令，且尚未因超时回中过
    if (now - last_cmd_time) > cmd_timeout and not has_auto_centered:
        rospy.logwarn("No gimbal cmd for %.1f s, auto centering...", cmd_timeout)
        try:
            siyi.one_click_back()
            rospy.loginfo("Gimbal auto centered due to timeout")
            # 标记为已经自动回中，避免定时器高频重复下发回中命令
            has_auto_centered = True
        except Exception as e:
            rospy.logerr("auto center (one_click_back) failed: %s", str(e))


def on_shutdown():
    """ROS 退出时调用云台的 close() 函数"""
    rospy.loginfo("Shutting down gimbal controller...")
    try:
        siyi.close()
    except Exception as e:
        rospy.logwarn("close() failed: %s", str(e))


def main():
    global cmd_timeout, attitude_pub

    rospy.init_node("gimbal_cmd_angle_bridge")

    # 从参数服务器读取超时时间（秒），默认 15.0
    cmd_timeout = rospy.get_param("~cmd_timeout", 15.0)

    # 创建姿态发布器
    attitude_pub = rospy.Publisher("/gimbal/attitude", Float32MultiArray, queue_size=10)

    # 启动即回中
    try:
        siyi.one_click_back()
        rospy.loginfo("Gimbal centered at startup")
    except Exception as e:
        rospy.logerr("one_click_back() failed: %s", str(e))

    # 订阅自定义云台控制话题
    sub = rospy.Subscriber("/gimbal/cmd_angle",
                           Point,
                           gimbal_cmd_callback,
                           queue_size=1)

    # 创建定时器，周期性检查是否超时未收到指令
    # 这里每 0.5 秒检查一次
    rospy.Timer(rospy.Duration(0.5), watchdog_timer_cb)
    
    # 创建定时器，以1Hz频率发布云台姿态
    rospy.Timer(rospy.Duration(1.0), publish_attitude_timer_cb)

    rospy.loginfo("Gimbal angle bridge started, listening to /gimbal/cmd_angle ...")
    rospy.loginfo("Auto-center timeout set to %.1f s", cmd_timeout)
    # rospy.loginfo("Watchdog timer DISABLED (No auto-center)") # 提示一下功能已关闭
    rospy.loginfo("Attitude publishing at 1Hz to /gimbal/attitude")

    # 注册关闭回调
    rospy.on_shutdown(on_shutdown)

    rospy.spin()


if __name__ == "__main__":
    main()
