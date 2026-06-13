#!/usr/bin/env python3
import os
import rospy
import cv2
import time
import numpy as np
import math
import threading
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import tf2_ros

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detection_node', anonymous=True)
        self.model = YOLO(os.path.join(_SCRIPT_DIR, "..", "pt", "yolov8n.pt"))
        self.bridge = CvBridge()

        self.camera_info_processed = False

        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.depth_sub = rospy.Subscriber("/fused/depth_image", Image, self.depth_callback, queue_size=1, buff_size=2**24)
        self.camera_info_sub = rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback)

        self.camera_matrix = None
        self.image_width = 0
        self.image_height = 0

        self.depth_scale = 0.001
        self.search_radius = 10

        self.detection_pub = rospy.Publisher("/yolo/detect_image", Image, queue_size=1)
        self.pose_pub = rospy.Publisher("/detect/local_position/pose_flu", PoseStamped, queue_size=10)

        self.camera_info_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.latest_depth_map = None
        self.latest_depth_encoding = None
        self.latest_depth_scale = 1.0

        self.target_size = 640
        self.frame_skip = 1
        self.frame_counter = 0
        self.processed_frames = 0
        self.start_time = time.time()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("YOLO detector started. Waiting for image data...")
        rospy.loginfo("Waiting for camera intrinsics...")

    def camera_info_callback(self, msg):
        if self.camera_info_processed:
            return

        with self.camera_info_lock:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            self.image_width = msg.width
            self.image_height = msg.height

            self.camera_info_processed = True

            rospy.loginfo("Received camera intrinsics:\n%s", self.camera_matrix)
            rospy.loginfo("Distortion coefficients: %s", self.dist_coeffs)
            rospy.loginfo("Image size: %dx%d", self.image_width, self.image_height)

            self.camera_info_sub.unregister()
            rospy.loginfo("Unsubscribed from camera info")

    def resize_image(self, image):
        h, w = image.shape[:2]
        scale = self.target_size / max(h, w)
        if scale < 1.0:
            return cv2.resize(image, (int(w * scale), int(h * scale))), scale
        return image, 1.0

    def get_depth_value(self, depth_image, u, v, depth_scale):
        if depth_image is None:
            return None, "No depth data"

        depth_height, depth_width = depth_image.shape[:2]
        if self.image_width <= 0 or self.image_height <= 0:
            return None, "Invalid image dimensions"

        scaled_u = int(u * depth_width / self.image_width + 0.5)
        scaled_v = int(v * depth_height / self.image_height + 0.5)

        if scaled_u < 0 or scaled_u >= depth_width or scaled_v < 0 or scaled_v >= depth_height:
            return None, "Coordinates out of bounds"

        center_depth = depth_image[scaled_v, scaled_u]
        if center_depth > 0:
            return center_depth * depth_scale, "Center depth"

        distances_and_depths = []
        for dv in range(-self.search_radius, self.search_radius + 1):
            for du in range(-self.search_radius, self.search_radius + 1):
                if du == 0 and dv == 0:
                    continue
                u_neighbor = scaled_u + du
                v_neighbor = scaled_v + dv
                if 0 <= u_neighbor < depth_width and 0 <= v_neighbor < depth_height:
                    neighbor_depth = depth_image[v_neighbor, u_neighbor]
                    if neighbor_depth > 0:
                        distance = math.sqrt(du * du + dv * dv)
                        distances_and_depths.append((distance, neighbor_depth))

        if not distances_and_depths:
            return None, "No valid depth in neighborhood"

        distances_and_depths.sort(key=lambda x: x[0])
        count = min(4, len(distances_and_depths))
        depth_sum = sum(d[1] for d in distances_and_depths[:count])

        return (depth_sum / count) * depth_scale, f"Average of {count} neighbors"

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        return np.array([
            [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
            [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])

    def project_pixel_to_3d(self, u, v, depth):
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z

    def depth_callback(self, msg):
        try:
            with self.depth_lock:
                if msg.encoding == '16UC1':
                    self.latest_depth_map = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                    self.latest_depth_encoding = msg.encoding
                    self.latest_depth_scale = 0.001
                elif msg.encoding == '32FC1':
                    self.latest_depth_map = self.bridge.imgmsg_to_cv2(msg, "32FC1")
                    self.latest_depth_encoding = msg.encoding
                    self.latest_depth_scale = 1.0
                else:
                    rospy.logwarn("Unsupported depth image encoding: %s", msg.encoding)
                    self.latest_depth_map = None
        except CvBridgeError as e:
            rospy.logerr("Depth image conversion failed: %s", e)

    def image_callback(self, color_msg):
        # 记录回调开始时间
        start_time = time.time()
        
        self.frame_counter += 1
        if self.frame_counter <= self.frame_skip:
            return
        self.frame_counter = 0

        try:
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            if self.image_width == 0:
                self.image_width = cv_color.shape[1]
                self.image_height = cv_color.shape[0]
        except CvBridgeError as e:
            rospy.logerr("Color image conversion failed: %s", e)
            return

        with self.depth_lock:
            if self.latest_depth_map is None:
                rospy.logwarn("No depth image available. Skipping frame.")
                return
            depth_image = self.latest_depth_map.copy()
            depth_scale = self.latest_depth_scale

        # 记录处理开始时间
        process_start = time.time()
        
        resized_img, scale = self.resize_image(cv_color)
        results = self.model(resized_img, verbose=False)
        annotated_frame = results[0].plot()
        boxes = results[0].boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx_r = int((x1 + x2) / 2)
            cy_r = int((y1 + y2) / 2)
            cx_o = int(cx_r / scale)
            cy_o = int(cy_r / scale)
            depth_value, depth_source = self.get_depth_value(depth_image, cx_o, cy_o, depth_scale)
            if depth_value is None:
                continue
            x_cam, y_cam, z_cam = self.project_pixel_to_3d(cx_o, cy_o, depth_value)
            try:
                tf_cam2fmu = self.tf_buffer.lookup_transform("fmu", "camera", color_msg.header.stamp, rospy.Duration(0.1))
                R = self.quaternion_to_rotation_matrix(
                    tf_cam2fmu.transform.rotation.x,
                    tf_cam2fmu.transform.rotation.y,
                    tf_cam2fmu.transform.rotation.z,
                    tf_cam2fmu.transform.rotation.w
                )
                t = np.array([
                    tf_cam2fmu.transform.translation.x,
                    tf_cam2fmu.transform.translation.y,
                    tf_cam2fmu.transform.translation.z
                ])
                pt_fmu = R @ np.array([x_cam, y_cam, z_cam]) + t
                pose = PoseStamped()
                pose.header.stamp = color_msg.header.stamp
                pose.header.frame_id = "fmu"
                pose.pose.position.x = pt_fmu[0]
                pose.pose.position.y = pt_fmu[1]
                pose.pose.position.z = pt_fmu[2]
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)
                cv2.circle(annotated_frame, (cx_r, cy_r), 4, (255, 0, 255), -1)
                cv2.putText(annotated_frame, f"{depth_value:.2f}m", (cx_r + 5, cy_r - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            except Exception as e:
                rospy.logwarn("Transform failed: %s", str(e))
                
        # 计算处理时间
        process_time = time.time() - process_start
        # 计算实时帧率 (FPS)
        instant_fps = 1.0 / process_time if process_time > 0 else 0
        
        # 在图像上标注处理时间和帧率
        text_processing = f"Proc: {process_time*1000:.1f}ms"
        text_fps = f"FPS: {instant_fps:.1f}"
        cv2.putText(annotated_frame, text_processing, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated_frame, text_fps, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 在终端打印
        rospy.loginfo(f"Frame processed: {text_processing}, {text_fps}")
        
        # 计算并打印总耗时（从回调开始到结束）
        total_time = time.time() - start_time
        rospy.loginfo(f"Total callback time: {total_time*1000:.1f}ms")

        try:
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.detection_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"Image conversion failed: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = YOLODetector()
    try:
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLO detector node has been shut down.")