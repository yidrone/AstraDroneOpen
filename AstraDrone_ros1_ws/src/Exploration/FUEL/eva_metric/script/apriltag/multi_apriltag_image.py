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

# Global
bridge = CvBridge()
odom_pose = None  # UAV 当前位姿
tag_size_ = 0.6   # AprilTag 的真实边长 (m)

# Camera info & publishers for each camera
# camera 2 fcu xyz   camera_pose in fcu_frame 
# camera_params = {
#     'front': {'K': None, 'D': None, 'offset': np.array([0.12,  0.00, 0.20])},
#     'left':  {'K': None, 'D': None, 'offset': np.array([0.00,  0.12, 0.20])},
#     'right': {'K': None, 'D': None, 'offset': np.array([0.00, -0.12, 0.20])},
#     'back':  {'K': None, 'D': None, 'offset': np.array([-0.12, 0.00, 0.20])}
# }
# 还需要考量旋转... 

# 相机内参配置，包括 K, D, 相对FCU偏移和姿态四元数（绕Z轴旋转）
from tf.transformations import quaternion_from_euler

camera_params = {
    'front': {
        'K': None,
        'D': None,
        'offset': np.array([0.12, 0.00, 0.20]),
        'rotation': quaternion_from_euler(-np.pi/2, 0, -np.pi/2)
    },
    'left': {
        'K': None,
        'D': None,
        'offset': np.array([0.00, 0.12, 0.20]),
        'rotation': quaternion_from_euler(-np.pi/2, 0, 0)
    },
    'right': {
        'K': None,
        'D': None,
        'offset': np.array([0.00, -0.12, 0.20]),
        'rotation': quaternion_from_euler(-np.pi/2, 0, np.pi)
    },
    'back': {
        'K': None,
        'D': None,
        'offset': np.array([-0.12, 0.00, 0.20]),
        'rotation': quaternion_from_euler(-np.pi/2, 0, np.pi/2)
    }
}


publishers = {}

def odom_callback(msg):
    global odom_pose
    odom_pose = (msg.pose.pose.position, msg.pose.pose.orientation)

def camera_info_callback(msg, cam_name):
    camera_params[cam_name]['K'] = np.array(msg.K).reshape(3, 3)
    camera_params[cam_name]['D'] = np.array(msg.D)

def rgb_image_callback(msg, cam_name):
    global odom_pose

    cam = camera_params[cam_name]
    if cam['K'] is None or cam['D'] is None or odom_pose is None:
        return

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11', quad_decimate=1.0, nthreads=4))
    detections = detector.detect(gray)

    if detections:
        for detection in detections:
            tag_corners = np.array(detection.corners, dtype=np.float32)
            half_size = tag_size_ / 2
            object_points = np.array([
                [-half_size, -half_size, 0],
                [ half_size, -half_size, 0],
                [ half_size,  half_size, 0],
                [-half_size,  half_size, 0]
            ], dtype=np.float32)

            ret, rvec, tvec = cv2.solvePnP(object_points, tag_corners, cam['K'], cam['D'])
            if ret:
                # 相机 → FCU 旋转
                r_cam_to_fcu = tf.transformations.quaternion_matrix(cam['rotation'])[:3, :3]
                tag_cam = tvec.flatten()

                # tag_fcu = np.array([tag_cam[2], -tag_cam[0], -tag_cam[1]])

                tag_fcu = r_cam_to_fcu @ tag_cam  # 转换到 FCU 坐标系
                tag_world = transform_to_world(tag_fcu, cam['offset'])
                publish_tag_pose(tag_world, cam_name)

def transform_to_world(tag_fcu_pos, cam_offset_fcu):
    global odom_pose
    tag_pos_fcu = tag_fcu_pos + cam_offset_fcu
    position, orientation = odom_pose
    quat = (orientation.x, orientation.y, orientation.z, orientation.w)
    rot_mat = tf.transformations.quaternion_matrix(quat)[:3, :3]
    tag_world = np.dot(rot_mat, tag_pos_fcu) + np.array([position.x, position.y, position.z])
    return tag_world

def publish_tag_pose(tag_world, cam_name):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world'
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = tag_world
    publishers[cam_name].publish(msg)

if __name__ == "__main__":
    rospy.init_node('multi_camera_apriltag_node')

    cam_list = ['front', 'left', 'right', 'back']
    # cam_list = ['right']
    for cam in cam_list:
        image_topic = f"/{cam}_camera/image_raw"
        info_topic = f"/{cam}_camera/camera_info"
        pub_topic = f"/{cam}_camera/apritag_pose"

        # Camera info and image subscribers
        rospy.Subscriber(info_topic, CameraInfo, camera_info_callback, cam)
        rospy.Subscriber(image_topic, Image, rgb_image_callback, cam)

        # Pose publisher
        publishers[cam] = rospy.Publisher(pub_topic, PoseStamped, queue_size=10)

    # Odom subscriber (shared)
    rospy.Subscriber('/ground_truth/odom', Odometry, odom_callback)

    rospy.spin()
