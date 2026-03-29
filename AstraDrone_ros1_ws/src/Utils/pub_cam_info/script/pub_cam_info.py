#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CameraInfo

def publish_camera_info():
    rospy.init_node('camera_info_publisher', anonymous=True)
    pub = rospy.Publisher('/head_camera/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    cam_info = CameraInfo()
    cam_info.width = 640
    cam_info.height = 480
    cam_info.header.frame_id = "head_camera"

    cam_info.distortion_model = "plumb_bob"
    cam_info.D = [-0.361976, 0.110510, 0.001014, 0.000505, 0.000000]
    cam_info.K = [
        438.783367, 0.0,       305.593336,
        0.0,       437.302876, 243.738352,
        0.0,       0.0,        1.0
    ]
    cam_info.R = [
        0.999978, 0.002789, -0.006046,
       -0.002816, 0.999986, -0.004401,
        0.006034, 0.004417, 0.999972
    ]
    cam_info.P = [
        393.653800, 0.0,        322.797939, 0.0,
        0.0,        393.653800, 241.090902, 0.0,
        0.0,        0.0,        1.0,        0.0
    ]

    while not rospy.is_shutdown():
        cam_info.header.stamp = rospy.Time.now()
        pub.publish(cam_info)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass
