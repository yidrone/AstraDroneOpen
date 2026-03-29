#!/usr/bin/env python3

import cv2
from siyi_sdk import SIYISDK
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def rtsp_video():

    print("video starting")
    rtsp_url="rtsp://192.168.144.25:8554/main.264"
    print("video started")

    cap     = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print("Error: Could not open video.")
        exit(1)

    cam     = SIYISDK(server_ip="192.168.144.25", port=37260)

    if not cam.connect():
        exit(1)

    cam.requestHardwareID()
    cam.setGimbalRotation(0,-90.0)

    # ROS publisher
    pub = rospy.Publisher('/a8_mini_image', Image, queue_size=10)
    bridge = CvBridge()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow("a8_mini stream", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
               break

        rpy = cam.getAttitude()  

        print("gimbal angles (yaw,pitch,roll) deg: ", rpy)
        print("Camera hardware ID: ", cam.getHardwareID())
        

        # Convert the image to ROS image message and publish
        img = cv2.resize(frame, (320, 240))
        ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(ros_image)
        
    cam.disconnect()


if __name__ == "__main__":
    rospy.init_node('a8_mini_img_publisher', anonymous=True)
    rtsp_video()