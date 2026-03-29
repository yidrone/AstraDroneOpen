#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from collections import deque
import numpy as np
from sklearn.cluster import KMeans
import threading
import time
from std_msgs.msg import Header

from sensor_msgs.msg import PointCloud2, PointField
import struct
import sensor_msgs.point_cloud2 as pc2

class ApriltagProcessor:
    # 连续数据传入的滑动时间窗口 time_window
    # 定时器周期 process_interval
    # 目标数据偏移量分类阈值  position_tolerance  （一个聚类 or 两个聚类）
    # 获取新的数据回调时清空之前的数据 new_sequence_threshold （暂时不用调节）
    # 新目标与已存在目标的gap existing_target_tolerance 
    def __init__(self, time_window=2.0, process_interval=0.5, position_tolerance=2.50, new_sequence_threshold=2.0, existing_target_tolerance=2.0):
        self.time_window = time_window
        self.process_interval = process_interval
        self.position_tolerance = position_tolerance
        self.new_sequence_threshold = new_sequence_threshold

        # 可能需要根据定位精度调整
        self.existing_target_tolerance = existing_target_tolerance  # Tolerance to determine if a new target overlaps with existing targets
        
        self.drones = ['drone1', 'drone2', 'drone3', 'drone4']
        self.data_queues = {drone: deque() for drone in self.drones}
        self.last_timestamps = {drone: None for drone in self.drones}
        self.targets = {drone: [] for drone in self.drones}  # To store the identified targets

        self.detected_targets = []  # To store the all targets
        
        # Mutex for thread-safe access to shared data
        self.lock = threading.Lock()
        
        for drone in self.drones:
            rospy.Subscriber(f"/{drone}/apritag_pose", PoseStamped, self.callback, callback_args=drone)
        
        self.pointcloud_publisher = rospy.Publisher('/detected_targets_pointcloud', PointCloud2, queue_size=10)
        
        # Replace rospy.Timer with our custom timer
        self.custom_timer = threading.Thread(target=self.run_timer)
        self.custom_timer.daemon = True  # Ensures the thread will exit when the main program exits
        self.custom_timer.start()

    def callback(self, msg, drone):
        try:
            current_time = rospy.Time.now().to_sec()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn("ROS time moved backwards, skipping this message.")
            return

        position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        
        # Acquire the lock before accessing shared data
        with self.lock:
            # Check the time interval since the last data point
            if self.last_timestamps[drone] is not None:
                time_interval = current_time - self.last_timestamps[drone]
                if time_interval > self.new_sequence_threshold:
                    # Reset the queue for a new sequence
                    self.data_queues[drone].clear()
            
            # Update the last timestamp
            self.last_timestamps[drone] = current_time
            
            # Add the current data to the queue
            self.data_queues[drone].append((current_time, position))
            
            # Remove old data points outside the time window
            while self.data_queues[drone] and (current_time - self.data_queues[drone][0][0]) > self.time_window:
                self.data_queues[drone].popleft()
    
    def run_timer(self):
        """Custom timer to replace rospy.Timer."""
        last_time = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():
            try:
                current_time = rospy.Time.now().to_sec()
                # If time moved backwards, wait and continue
                if current_time < last_time:
                    rospy.logwarn("ROS time moved backwards, waiting before next process.")
                    time.sleep(self.process_interval)
                    continue

                process_needed = False
                with self.lock:
                    for drone in self.drones:
                        if self.last_timestamps[drone] is not None:
                            time_since_last_data = current_time - self.last_timestamps[drone]
                            if time_since_last_data > 0.5:
                                process_needed = True
                                break  # We only need one drone to have stale data to process

                if process_needed:
                    self.process_all_data()

                last_time = current_time
                time.sleep(self.process_interval)

                # Create PointCloud2 message
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
                for target in self.detected_targets:
                    x, y, z = target
                    r, g, b = 255, 0, 0  # Red color
                    a = 255
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    points.append([x, y, z, rgb])
                
                pc2_msg = pc2.create_cloud(header, fields, points)
                
                # Publish the pointcloud
                self.pointcloud_publisher.publish(pc2_msg)

            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("ROS time moved backwards during custom timer, skipping this cycle.")
                time.sleep(self.process_interval)  # Wait before the next cycle
    
    def process_all_data(self):
        # Acquire the lock before accessing shared data
        with self.lock:
            # Process data for each drone
            for drone in self.drones:
                self.process_data(drone)
    
    def process_data(self, drone):
        positions = np.array([pos for _, pos in self.data_queues[drone]])
        if len(positions) < 2: # Ensure at least 2 samples for clustering
            # rospy.loginfo(f"Drone: {drone}, Not enough samples ({len(positions)}) for clustering.")
            return
        
        # Determine the number of clusters (targets)
        # kmeans = KMeans(n_clusters=2, random_state=0).fit(positions)
        kmeans = KMeans(n_clusters=2, random_state=0, n_init=10).fit(positions)
        centers = kmeans.cluster_centers_
        labels = kmeans.labels_
        
        # Calculate the distance between the cluster centers
        center_distance = np.linalg.norm(centers[0] - centers[1])
        
        if center_distance < self.position_tolerance:
            # Treat as a single target, filter and average
            mean_position = np.mean(positions, axis=0)
            filtered_positions = positions[np.linalg.norm(positions - mean_position, axis=1) < self.position_tolerance]
            if len(filtered_positions) > 0:
                final_position = np.mean(filtered_positions, axis=0)
                self.add_target(drone, final_position)
            else:
                rospy.loginfo(f"Drone: {drone}, No position within tolerance.")
        else:
            # Treat as two separate targets
            target1_positions = positions[labels == 0]
            target2_positions = positions[labels == 1]
            
            if len(target1_positions) > 0:
                target1_mean_position = np.mean(target1_positions, axis=0)
                self.add_target(drone, target1_mean_position)
            
            if len(target2_positions) > 0:
                target2_mean_position = np.mean(target2_positions, axis=0)
                self.add_target(drone, target2_mean_position)

        # Clear the queue after processing
        self.data_queues[drone].clear()
    
    def add_target(self, drone, new_target):
        """
        Add a new target position to the list of targets for the specified drone.
        If the new target is too close to an existing target, it is considered overlapping and not added.
        """
        # Check if the new target overlaps with existing targets
        # for existing_target in self.targets[drone]:
        #     if np.linalg.norm(np.array(new_target) - np.array(existing_target)) < self.existing_target_tolerance:
        #         rospy.loginfo(f"Drone: {drone}, New target overlaps with existing target. Ignoring: {new_target}")
        #         return
        # 不区分target是哪个无人机检测到
        for existing_target in self.detected_targets:
            if np.linalg.norm(np.array(new_target) - np.array(existing_target)) < self.existing_target_tolerance:
                rospy.loginfo(f"Drone: {drone}, New target overlaps with existing target. Ignoring: {new_target}")
                return
        
        # If no overlap, add the new target
        self.targets[drone].append(new_target)

        self.detected_targets.append(new_target)
        rospy.loginfo(f"Drone: {drone}, New target added: {new_target}")

if __name__ == "__main__":
    rospy.init_node('apriltag_processor')
    # tuning params 
    time_window = rospy.get_param('time_window', 2.0)
    process_interval = rospy.get_param('process_interval', 0.5)
    position_tolerance = rospy.get_param('position_tolerance', 2.5)
    new_sequence_threshold = rospy.get_param('new_sequence_threshold', 2.0)
    existing_target_tolerance = rospy.get_param('existing_target_tolerance', 2.0)

    print(f'Params Settings as below: ')
    print(f'time_window: {time_window}')
    print(f'process_interval: {process_interval}')
    print(f'position_tolerance: {position_tolerance}')
    print(f'new_sequence_threshold: {new_sequence_threshold}')
    print(f'existing_target_tolerance: {existing_target_tolerance}')

    # processor = ApriltagProcessor()
    processor = ApriltagProcessor(time_window, process_interval, position_tolerance, new_sequence_threshold, existing_target_tolerance)
    rospy.spin()
