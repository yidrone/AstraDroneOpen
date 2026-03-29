#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_srvs.srv import Trigger, TriggerResponse
import os

def make_cloud(points_xyz, frame_id="world"):
    fields = [
        PointField('x', 0,  PointField.FLOAT32, 1),
        PointField('y', 4,  PointField.FLOAT32, 1),
        PointField('z', 8,  PointField.FLOAT32, 1),
    ]
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return pc2.create_cloud(header, fields, points_xyz)

class PCDPublisher:
    def __init__(self):
        self.pub = rospy.Publisher("/map_cloud", PointCloud2, queue_size=1, latch=True)
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.voxel = float(rospy.get_param("~voxel", 0.15))
        self.pcd_path = rospy.get_param("~pcd_path", "")
        self.load_srv = rospy.Service("~load", Trigger, self.load_cb)
        rospy.loginfo("pcd_publisher ready: service /pcd_publisher/load")

    def load_cb(self, req):
        self.pcd_path = rospy.get_param("~pcd_path", self.pcd_path)
        self.voxel = float(rospy.get_param("~voxel", self.voxel))

        if not self.pcd_path or not os.path.exists(self.pcd_path):
            return TriggerResponse(False, f"pcd not found: {self.pcd_path}")

        pcd = o3d.io.read_point_cloud(self.pcd_path)
        if self.voxel and self.voxel > 0:
            pcd = pcd.voxel_down_sample(self.voxel)

        pts = np.asarray(pcd.points, dtype=np.float32)
        msg = make_cloud(pts, frame_id=self.frame_id)
        self.pub.publish(msg)
        return TriggerResponse(True, f"loaded {os.path.basename(self.pcd_path)} points={len(pts)} voxel={self.voxel}")

if __name__ == "__main__":
    rospy.init_node("pcd_publisher")
    PCDPublisher()
    rospy.spin()
