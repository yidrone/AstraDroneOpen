#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
import time

def main():
    rospy.init_node("gimbal_test_publisher")

    pub = rospy.Publisher("/gimbal/cmd_angle", Point, queue_size=1)

    rospy.loginfo("等待订阅者连接...")
    time.sleep(1)

    rate = rospy.Rate(0.2)  # 0.5 Hz = 每 5 秒发一次

    while not rospy.is_shutdown():
        # ====== 测试 1：向右转 ======
        msg = Point()
        msg.x = 0.0     # pitch
        msg.y = 30.0    # yaw
        msg.z = 0.0
        pub.publish(msg)
        rospy.loginfo("发布：pitch=0, yaw=30")
        rate.sleep()

        # ====== 测试 2：向左转 ======
        msg.y = -30.0
        pub.publish(msg)
        rospy.loginfo("发布：pitch=0, yaw=-30")
        rate.sleep()

        # ====== 测试 3：向上仰 ======
        msg.x = 20.0
        msg.y = 0.0
        pub.publish(msg)
        rospy.loginfo("发布：pitch=20, yaw=0")
        rate.sleep()

        # ====== 测试 4：向下俯 ======
        msg.x = -20.0
        msg.y = 0.0
        pub.publish(msg)
        rospy.loginfo("发布：pitch=-20, yaw=0")
        rate.sleep()

        # ====== 回中 ======
        msg.x = 0.0
        msg.y = 0.0
        pub.publish(msg)
        rospy.loginfo("发布：pitch=0, yaw=0")
        rate.sleep()


if __name__ == "__main__":
    main()

