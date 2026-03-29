#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utils/ikd_Tree.h>
#include <utils/yaml_loader.hpp>
#include <nav_msgs/Odometry.h>
#include "nav_msgs/Path.h"

#include <signal.h>
#include <pcl/io/pcd_io.h>

#include <std_msgs/Header.h>

using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;
using namespace std;

// 全局变量
pcl::PointCloud<pcl::PointXYZ> lidar_cloud_;
bool first_map_flag_ = true;
KD_TREE<PointType> ikd_Tree_map;

// 
std::string pointcloud_topic_, pub_map_topic_, odom_topic_, pub_path_topic_;
float ikdtree_res_;
bool save_pcd = false;
nav_msgs::Path path;

std::string filename, store_dir, base_filename;
string explore_finish_trigger;


// 全局地图边界
Eigen::Vector3d map_min_ = Eigen::Vector3d::Constant(1e9);
Eigen::Vector3d map_max_ = Eigen::Vector3d::Constant(-1e9);

// 发布器
ros::Publisher pub_global_map, pub_path;
ros::Subscriber cloud_sub, odom_sub, trigger_sub;

// 半边界框
Eigen::Vector3d rog_box_half_(0.5, 0.5, 0.5); // 可调参数

void saveCloudMap() {
    string filename_new;
    // 获取系统时间，存储对应的文件
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    // 格式化时间为字符串
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M-%S");

    filename_new = store_dir + "/" + ss.str() + "_" + base_filename;;  // 使用 .str() 方法转换

    if(first_map_flag_){
        ROS_ERROR("Empty ikd_Tree_map! Break saveCloudMap process! %s", filename_new.c_str());
        return;
    } 

    // 获取所有点
    PointVector all_points;

    BoxPointType map_box;
    for (int i = 0; i < 3; ++i) {
        map_box.vertex_min[i] = map_min_[i] - rog_box_half_[i] ;
        map_box.vertex_max[i] = map_max_[i] + rog_box_half_[i] ;
    }
    ikd_Tree_map.Box_Search(map_box, all_points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points = all_points;
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    if (pcl::io::savePCDFileBinary(filename_new, *cloud) == 0) {
        ROS_INFO("[EXIT] Map saved to %s", filename_new.c_str());
    } else {
        ROS_ERROR("[EXIT] Failed to save map to %s", filename_new.c_str());
    }

}


void triggerCallback(const std_msgs::HeaderConstPtr& msg){
    if(msg->seq ==0) return;
    static int explore_finish_cc = 0;
    static bool save_once = true;

    explore_finish_cc ++;
    if(explore_finish_cc = 5 && save_once){
        save_once = false;
        ROS_WARN("Finshed Explore! Save the .pcd in dir %s.", store_dir.c_str());
        saveCloudMap();
    }
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, lidar_cloud_);

    if (lidar_cloud_.empty()) return;

    // 下采样
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    // vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.setLeafSize(ikdtree_res_, ikdtree_res_, ikdtree_res_);
    vg.setInputCloud(lidar_cloud_.makeShared());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*filtered_points);

    if (filtered_points->empty()) return;

    PointVector pcl_map = filtered_points->points;

    // 插入 ikdtree
    if (first_map_flag_) {
        ikd_Tree_map.set_downsample_param(ikdtree_res_);
        ikd_Tree_map.Build(pcl_map);
        first_map_flag_ = false;
    } else {
        ikd_Tree_map.Add_Points(pcl_map, true);
    }

    // 更新边界框
    for (const auto& pt : pcl_map) {
        map_min_ = map_min_.cwiseMin(Eigen::Vector3d(pt.x, pt.y, pt.z));
        map_max_ = map_max_.cwiseMax(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }

    static int counts = 0;
    counts ++;
    if(counts>5){
        counts = 0;
        // 获取边界框内的点云
        PointVector points;
        BoxPointType map_box;
        for (int i = 0; i < 3; ++i) {
            map_box.vertex_min[i] = map_min_[i] - rog_box_half_[i] ;
            map_box.vertex_max[i] = map_max_[i] + rog_box_half_[i] ;
        }
        ikd_Tree_map.Box_Search(map_box, points);

        // 发布地图点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        map_cloud->points = points;
        map_cloud->width = points.size();
        map_cloud->height = 1;
        map_cloud->is_dense = true;

        sensor_msgs::PointCloud2 pub_msg;
        pcl::toROSMsg(*map_cloud, pub_msg);
        pub_msg.header.frame_id = "world";
        pub_msg.header.stamp = cloud_msg->header.stamp;
        pub_global_map.publish(pub_msg);
    }
    
}

void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = odom_msg->pose.pose.position.x;
    pose.pose.position.y = odom_msg->pose.pose.position.y;
    pose.pose.position.z = odom_msg->pose.pose.position.z;
    pose.pose.orientation.x = odom_msg->pose.pose.orientation.x;
    pose.pose.orientation.y = odom_msg->pose.pose.orientation.y;
    pose.pose.orientation.z = odom_msg->pose.pose.orientation.z;
    pose.pose.orientation.w = odom_msg->pose.pose.orientation.w;
    path.poses.push_back(pose);
    pub_path.publish(path);
}

void saveMapAndExit(int signum) {
    if (!save_pcd) {
        ROS_INFO("[EXIT] save_pcd is false, exiting without saving map.");
        ros::shutdown();
        exit(signum);
    }

    ROS_INFO("[EXIT] Caught signal %d, saving full ikd-tree map...", signum);

    // 获取所有点
    PointVector all_points;

    BoxPointType map_box;
    for (int i = 0; i < 3; ++i) {
        map_box.vertex_min[i] = map_min_[i] - rog_box_half_[i] ;
        map_box.vertex_max[i] = map_max_[i] + rog_box_half_[i] ;
    }
    ikd_Tree_map.Box_Search(map_box, all_points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points = all_points;
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    if (pcl::io::savePCDFileBinary(filename, *cloud) == 0) {
        ROS_INFO("[EXIT] Map saved to %s", filename.c_str());
    } else {
        ROS_ERROR("[EXIT] Failed to save map to %s", filename.c_str());
    }

    ros::shutdown();
    exit(signum);
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "clouds2ikdtree_node");
    ros::NodeHandle nh("~");  // 用 "~" 获取私有命名空间下的参数（即 launch 文件中 node 内部的 param）


    std::string config_file;
    if (nh.getParam("config_name", config_file)) {
        ROS_INFO_STREAM("Loaded config name: " << config_file);
    } else {
        ROS_WARN("Failed to get param 'config_file'. Using default.");
    }

    yaml_loader::YamlLoader loader(config_file);

    loader.LoadParam("clouds2ikdtree/pointcloud_topic", pointcloud_topic_, string("/quad0_pcl_render_node/cloud"));
    loader.LoadParam("clouds2ikdtree/pub_map_topic", pub_map_topic_, string("/global_ikdtree_map"));
    // 需要为 float 
    loader.LoadParam("clouds2ikdtree/ikdtree_res", ikdtree_res_, 0.1f);

    loader.LoadParam("rog_map/ros_callback/odom_topic", odom_topic_, string("/quad_0/lidar_slam/odom"));
    loader.LoadParam("clouds2ikdtree/pub_odom_topic", pub_path_topic_, string("/explored/path"));

    
    loader.LoadParam("clouds2ikdtree/explore_finish_trigger", explore_finish_trigger, string("explore_finish"));

    loader.LoadParam("clouds2ikdtree/save_pcd", save_pcd, false);
    loader.LoadParam("clouds2ikdtree/file_name", base_filename, string("forest.pcd"));

    // lkh tsp solver dir 
    nh.param("pcd/store_dir", store_dir, string("null"));

    filename = store_dir + "/" + base_filename;
    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic_, 10, cloudCallback);
    pub_global_map = nh.advertise<sensor_msgs::PointCloud2>(pub_map_topic_, 10);

    trigger_sub = nh.subscribe<std_msgs::Header>(explore_finish_trigger, 10, triggerCallback);

    odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 10, odomCallback);
    pub_path = nh.advertise<nav_msgs::Path>(pub_path_topic_, 10);


    ikd_Tree_map.set_downsample_param(ikdtree_res_);
    ikd_Tree_map.Set_delete_criterion_param(0.3);
    ikd_Tree_map.Set_balance_criterion_param(0.6);
    
    signal(SIGINT, saveMapAndExit);  // 注册 Ctrl+C 退出时的处理函数

    ros::spin();
    return 0;
}
