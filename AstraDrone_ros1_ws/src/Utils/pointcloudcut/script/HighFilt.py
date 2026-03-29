#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import struct

class PointCloudHeightFilter:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('point_cloud_height_filter', anonymous=True)
        
        # 设置高度过滤范围（单位：米）
        self.min_height = rospy.get_param('~min_height', 0.1)  # 默认最小高度为-0.5米
        self.max_height = rospy.get_param('~max_height', 2.0)   # 默认最大高度为2.0米
        
        # 创建发布者，发布过滤后的点云数据
        self.filtered_pub = rospy.Publisher('/filtered_cloud', PointCloud2, queue_size=1)
        
        # 创建订阅者，订阅原始点云数据
        self.cloud_sub = rospy.Subscriber('/laser_cloud_map', PointCloud2, self.cloud_callback)
        
        rospy.loginfo("Point cloud height filter initialized")
        rospy.loginfo(f"Height range: [{self.min_height}, {self.max_height}] meters")

    def cloud_callback(self, cloud_msg):
        # 转换点云数据为numpy数组
        cloud_points = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            cloud_points.append(point)
        
        if not cloud_points:
            rospy.logwarn("Received empty point cloud")
            return
            
        points_array = np.array(cloud_points)
        
        # 按照高度（z轴）进行过滤
        height_mask = (points_array[:, 2] >= self.min_height) & (points_array[:, 2] <= self.max_height)
        filtered_points = points_array[height_mask]
        
        if filtered_points.size == 0:
            rospy.logwarn("No points left after height filtering")
            return
            
        # 创建一个新的点云消息
        filtered_cloud = self.create_cloud_msg(filtered_points, cloud_msg.header)
        
        # 发布过滤后的点云
        self.filtered_pub.publish(filtered_cloud)
        
        # 打印信息
        filter_ratio = 100 * (1 - len(filtered_points) / len(points_array))
        rospy.loginfo(f"Filtered {filter_ratio:.2f}% of points. Published {len(filtered_points)} points.")

    def create_cloud_msg(self, points_array, header):
        """创建一个新的PointCloud2消息"""
        # 复制原始消息的头部信息
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        
        # 设置点云的基本属性
        cloud_msg.height = 1
        cloud_msg.width = len(points_array)
        
        # 定义点云的字段（x, y, z）
        cloud_msg.fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        ]
        
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 * 4 bytes (float32)
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        
        # 将点云数据打包到消息中
        cloud_msg.is_dense = True
        cloud_msg.data = np.asarray(points_array, np.float32).tobytes()
        
        return cloud_msg

def main():
    try:
        filter_node = PointCloudHeightFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
