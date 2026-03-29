#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
import numpy as np
import threading

# 相机名	朝向 (面对方向)	绕 Z 轴旋转角度	四元数 (Euler)
# front	正前	-π/2	(-π/2, 0, -π/2)
# left	左侧	0	(-π/2, 0, 0)
# right	右侧	π	(-π/2, 0, π)
# back	后方	π/2	(-π/2, 0, π/2)

class ApriltagProcessor:
    def __init__(self, process_interval=0.5, position_tolerance=2.5, existing_target_tolerance=2.0):
        self.process_interval = process_interval
        self.position_tolerance = position_tolerance
        self.existing_target_tolerance = existing_target_tolerance

        self.drones = ['front', 'left', 'right', 'back']
        self.all_positions = []  # 所有无人机检测到的点

        self.detected_targets = []
        self.lock = threading.Lock()

        for drone in self.drones:
            rospy.Subscriber(f"/{drone}_camera/apritag_pose", PoseStamped, self.callback)

        self.pointcloud_publisher = rospy.Publisher('/detected_targets_pointcloud', PointCloud2, queue_size=10)

        self.custom_timer = threading.Thread(target=self.run_timer)
        self.custom_timer.daemon = True
        self.custom_timer.start()

    def callback(self, msg):
        position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        with self.lock:
            self.all_positions.append(position)
        
    def run_timer(self):
        while not rospy.is_shutdown():
            self.process_all_data()

            # 可视化
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "world"
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.FLOAT32, 1)
            ]
            points = []
            with self.lock:
                for target in self.detected_targets:
                    x, y, z = target
                    r, g, b, a = 255, 0, 0, 255
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    points.append([x, y, z, rgb])
            pc2_msg = pc2.create_cloud(header, fields, points)
            self.pointcloud_publisher.publish(pc2_msg)

            rospy.sleep(self.process_interval)

    def process_all_data(self):
        with self.lock:
            if not self.all_positions:
                return

            positions = np.array(self.all_positions)
            self.all_positions.clear()

            # 聚类逻辑（基于距离阈值的简单聚类）
            clusters = []
            for point in positions:
                added = False
                for cluster in clusters:
                    if np.linalg.norm(point - cluster['center']) < self.position_tolerance:
                        cluster['points'].append(point)
                        cluster['center'] = np.mean(cluster['points'], axis=0)
                        added = True
                        break
                if not added:
                    clusters.append({'points': [point], 'center': point})

            for cluster in clusters:
                center = cluster['center']
                if self.is_new_target(center):
                    self.detected_targets.append(center)
                    rospy.loginfo(f"New target added: {center}")

    def is_new_target(self, new_target):
        for existing in self.detected_targets:
            if np.linalg.norm(new_target - existing) < self.existing_target_tolerance:
                return False
        return True

if __name__ == "__main__":
    rospy.init_node('apriltag_processor')

    process_interval = rospy.get_param('process_interval', 0.5)
    position_tolerance = rospy.get_param('position_tolerance', 2.5)
    existing_target_tolerance = rospy.get_param('existing_target_tolerance', 2.0)

    rospy.loginfo(f"Params:\n  process_interval={process_interval}\n  position_tolerance={position_tolerance}\n  existing_target_tolerance={existing_target_tolerance}")

    processor = ApriltagProcessor(process_interval, position_tolerance, existing_target_tolerance)
    rospy.spin()
