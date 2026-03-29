#ifndef _VISUALIZATION_H
#define _VISUALIZATION_H

#include <Eigen/Eigen>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h> // 必须引入：用于点云变换
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
// 增加：TF2相关库用于坐标变换
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "freedom/freedom.h"

namespace freedom{
class Visualizer{
public:
    struct Config
    {
        std::string map_tf_frame;
        double sub_voxel_size;
        int voxel_depth;
        int block_depth;
        bool enable_raycast_enhancement;
        // 【新增】：用于接收传感器范围参数
        double real_min_range;
        double real_max_range;
    
        // --- 雷达安装平移（m） ---
        double lidar_x = 0.0;
        double lidar_y = 0.0;
        double lidar_z = 0.0;

        // --- 雷达安装姿态（deg） ---
        double lidar_roll_deg  = 0.0;
        double lidar_pitch_deg = 0.0;
        double lidar_yaw_deg   = 0.0;

        bool flag_dynamic = false;
    };

    Visualizer(){}

    void set_params(const Config& config,ros::NodeHandle& nh);

    void visualize_scan_removal_result(const ScanMap& scan);
    void visualize_raycast_enhancement_result(const DepthImage& image);
    // 修改：增加 current_scan 和 tf_buffer 参数，默认为 nullptr 用于兼容旧调用
    void visualize_map_removal_result(const MRMap& map, 
                                      const sensor_msgs::PointCloud2::ConstPtr& current_scan = nullptr,
                                      tf2_ros::Buffer* tf_buffer = nullptr); // 修改这里：改为指针
private:
    std::string map_tf_frame;
    // 【新增】：类成员变量，用于在 visualize_static_pointcloud 中使用
    double real_min_range_;
    double real_max_range_;

    double sub_voxel_size;
    double voxel_size;
    double block_size;
    double half_sub_voxel_size;
    double half_voxel_size;
    double half_block_size;
    Point half_sub_voxel_bias;
    Point half_voxel_bias;
    Point half_block_bias;

    // --- 雷达安装平移（m） ---
    double lidar_x_ = 0.0;
    double lidar_y_ = 0.0;
    double lidar_z_ = 0.0;

    // --- 雷达安装姿态（deg） ---
    double lidar_roll_deg_  = 0.0;
    double lidar_pitch_deg_ = 0.0;
    double lidar_yaw_deg_   = 0.0;

    bool flag_dynamic_;

    ros::Publisher scan_blocks_pub;
    ros::Publisher scan_voxels_pub;
    ros::Publisher clusters_pub;
    
    ros::Publisher depth_image_pub;
    ros::Publisher enhanced_depth_image_pub;
    ros::Publisher enhanced_pointcloud_pub;

    ros::Publisher raycasted_blocks_pub;
    ros::Publisher raycasted_voxels_pub;
    ros::Publisher free_blocks_pub;
    ros::Publisher free_voxels_pub;
    ros::Publisher static_blocks_pub;
    ros::Publisher static_voxels_pub;
    ros::Publisher static_subvoxels_pub;
    ros::Publisher static_pointcloud_pub;

    ros::Publisher scan_map_range_pub;
    ros::Publisher local_map_range_pub;
    ros::Publisher raycast_map_range_pub;

    bool enable_raycast_enhancement;

    // scan removal results
    void visualize_scan_blocks(const ScanMap& scan);
    void visualize_scan_voxels(const ScanMap& scan);
    void visualize_clusters(const ScanMap& scan);

    // raycast enhancement results
    void visualize_depth_image(const DepthImage& image);
    void visualize_enhanced_pointcloud(const DepthImage& image);

    // map removal results
    void visualize_raycasted_blocks(const MRMap& map);
    void visualize_raycasted_voxels(const MRMap& map);
    void visualize_free_blocks(const MRMap& map);
    void visualize_free_voxels(const MRMap& map);

    void visualize_static_blocks(const MRMap& map);
    void visualize_static_voxels(const MRMap& map);
    void visualize_static_subvoxels(const MRMap& map);
    // 修改：核心处理函数，增加参数
    void visualize_static_pointcloud(const MRMap& map,
                                     const sensor_msgs::PointCloud2::ConstPtr& current_scan,
                                     tf2_ros::Buffer* tf_buffer); // 修改这里：改为指针
    void visualize_scan_map_range(const ScanMap& scan);
    void visualize_local_map_range(const MRMap& map);
    void visualize_raycast_map_range(const MRMap& map);
};
}
#endif