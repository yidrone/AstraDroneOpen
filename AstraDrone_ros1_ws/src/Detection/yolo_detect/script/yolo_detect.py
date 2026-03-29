#!/usr/bin/env python3
import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detection_node', anonymous=True)
        self.model = YOLO("/home/uav/AstraDrone/AstraDrone_ros1_ws/src/Detection/yolo_detect/pt/yolov8n.pt")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/csi_camera/image_raw", Image, self.image_callback)
        # 添加图像发布者
        self.detection_pub = rospy.Publisher("/yolo/detect_image", Image, queue_size=1)
        
        self.target_size = 640
        self.frame_skip = 1
        self.frame_counter = 0
        self.processed_frames = 0
        self.start_time = time.time()
        self.last_time = time.time()
        rospy.loginfo("YOLO检测器已启动,等待图像数据...")

    def resize_image(self, image):
        h, w = image.shape[:2]
        scale = self.target_size / max(h, w)
        if scale < 1.0:
            return cv2.resize(image, (int(w * scale), int(h * scale))), scale
        return image, 1.0
    
    @staticmethod
    def ROSImage2CVImage(ros_img_msg):
        img = np.frombuffer(ros_img_msg.data, dtype = np.uint8).reshape(ros_img_msg.height, ros_img_msg.width, -1)
        img = np.ascontiguousarray(img)
        if ros_img_msg.encoding in ["rgb8", "rgb16"]:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    
    def image_callback(self, msg):
        self.frame_counter += 1
        if self.frame_counter <= self.frame_skip:
            return
        self.frame_counter = 0
        
        start_time = time.time()
        cv_image = self.ROSImage2CVImage(msg)
        resized_img, scale = self.resize_image(cv_image)
        results = self.model(resized_img, verbose=True)
        process_time = time.time() - start_time
        self.processed_frames += 1
        
        # 创建带标注的图像
        annotated_frame = results[0].plot()
        
        # 添加性能信息到图像
        fps = 1.0 / process_time 
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(annotated_frame, f"Proc: {process_time*1000:.1f}ms", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(annotated_frame, f"Size: {resized_img.shape[1]}x{resized_img.shape[0]}", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        try:
            # 将标注后的图像转换为ROS消息并发布
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.detection_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = YOLODetector()
    try:
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点已关闭")