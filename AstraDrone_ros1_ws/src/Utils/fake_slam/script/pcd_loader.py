#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import open3d as o3d
import numpy as np  # 导入numpy

def load_pcd_and_publish(pcd_file, voxel_size=0.1):  # voxel_size 控制降采样的精度
    # 初始化ROS节点
    rospy.init_node('pcd_publisher', anonymous=True)
    pub = rospy.Publisher('/laser_cloud_map', PointCloud2, queue_size=10)

    # 加载PCD文件
    cloud = o3d.io.read_point_cloud(pcd_file)
    
    # 降采样点云
    cloud = cloud.voxel_down_sample(voxel_size)

    # 获取降采样后的点云数据
    points = np.asarray(cloud.points)

    # 构建PointCloud2消息
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "camera_init"
    point_cloud_msg = point_cloud2.create_cloud_xyz32(header, points)

    rate = rospy.Rate(0.4)  # 频率0.4Hz（根据需求可调整）
    while not rospy.is_shutdown():
        point_cloud_msg.header.stamp = rospy.Time.now()  # 更新时间戳
        pub.publish(point_cloud_msg)
        rate.sleep()

if __name__ == "__main__":
    import sys
    try:
        if len(sys.argv) < 2:
            print("用法: python3 pcd_loader.py <pcd文件路径> [体素大小]")
            print("示例: python3 pcd_loader.py /path/to/map.pcd 0.08")
            sys.exit(1)
        pcd_file = sys.argv[1]
        voxel_size = float(sys.argv[2]) if len(sys.argv) > 2 else 0.08  # 设置体素大小（越小降采样越精细，越大降采样越粗）
        load_pcd_and_publish(pcd_file, voxel_size)
    except rospy.ROSInterruptException:
        pass
