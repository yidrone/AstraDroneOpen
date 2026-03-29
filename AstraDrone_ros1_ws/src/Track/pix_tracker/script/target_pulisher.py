#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class TargetVisualizer:
    """图像中显示 target_pixel 的可视化工具（读取参数/订阅话题）"""

    def __init__(self):
        rospy.init_node('target_visualizer', anonymous=True)

        self.bridge = CvBridge()
        self.current_image = None
        self.target_point = Point()

        # 获取初始目标像素点参数（默认图像中心）
        self.target_point.x = rospy.get_param("~target_pixel/x", 320.0)
        self.target_point.y = rospy.get_param("~target_pixel/y", 240.0)
        self.target_point.z = 0.0

        # 订阅图像
        self.image_sub = rospy.Subscriber(
            '/iris_mid360/camera/rgb/image_raw', Image, self.image_callback)

        # 订阅 target_pixel 坐标
        self.target_sub = rospy.Subscriber(
            '/target_pixel', Point, self.target_callback)

        rospy.loginfo("图像目标可视化工具已启动")
        rospy.loginfo(
            f"初始目标像素点: ({self.target_point.x}, {self.target_point.y})")

    def target_callback(self, msg):
        """接收目标像素点话题"""
        self.target_point = msg

    def image_callback(self, msg):
        """图像回调：处理并显示图像"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image.copy()

            self.draw_markers()

            cv2.imshow('Target Visualizer', self.current_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"图像处理错误: {e}")

    def draw_markers(self):
        """在图像上绘制中心点和目标点标记"""
        if self.current_image is None:
            return

        height, width = self.current_image.shape[:2]
        center_x, center_y = width // 2, height // 2

        # 中心点（绿色十字）
        cv2.drawMarker(self.current_image, (center_x, center_y),
                       (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(self.current_image, 'CENTER', (center_x - 30, center_y - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 目标点（红圈）
        if self.target_point.x > 1 and self.target_point.y > 1:
            target_x = int(self.target_point.x)
            target_y = int(self.target_point.y)

            cv2.circle(self.current_image, (target_x, target_y),
                       10, (0, 0, 255), 2)
            cv2.putText(self.current_image, 'TARGET', (target_x - 25, target_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.line(self.current_image, (center_x, center_y),
                     (target_x, target_y), (255, 0, 0), 1)

        # 说明文字
        cv2.putText(self.current_image, 'Subscribed /target_pixel',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    def run(self):
        rospy.loginfo("图像目标可视化运行中...")
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        visualizer = TargetVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"程序错误: {e}")
