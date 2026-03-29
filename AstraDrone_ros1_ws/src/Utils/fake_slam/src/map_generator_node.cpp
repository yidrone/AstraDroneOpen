#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>

class MapGenerator
{
public:
    MapGenerator()
    {
        // 订阅位姿和点云话题
        pose_sub_ = nh_.subscribe("/vlp/lidar/pose", 10, &MapGenerator::poseCallback, this);
        pointcloud_sub_ = nh_.subscribe("/velodyne_points", 10, &MapGenerator::pointCloudCallback, this);

        // 初始化发布器
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 10);

        // 初始化TF广播器
        tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();

        // 初始化计时器定期发布地图（0.2Hz -> 5秒发布一次）
        timer_ = nh_.createTimer(ros::Duration(2.0), &MapGenerator::publishMap, this);
    }

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 获取机器人位姿
        std::lock_guard<std::mutex> lock(mutex_);
        robot_pose_ = msg->pose.pose;

        // 发布TF关系从 camera_init 到 aft_mapped
        publishTF();
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // 将 PointCloud2 转换为 pcl::PointCloud 类型
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // 对点云进行距离过滤和随机降采样
        filterAndRandomlyDownsamplePointCloud(pcl_cloud);

        // 将当前点云转换到全局坐标系并合并到地图中
        addPointCloudToMap(pcl_cloud);
    }

    void filterAndRandomlyDownsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

        // 设定随机采样概率（例如，0.5 表示保留 50% 的点）
        const double sampling_probability = 0.1;

        // 遍历点云进行过滤和随机降采样
        for (const auto& point : cloud.points)
        {
            // 距离过滤：保留距离小于 20 米的点
            if (std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z) > 10.0)
            {
                continue;
            }

            // 随机降采样：使用概率函数决定是否保留点
            if (static_cast<double>(std::rand()) / RAND_MAX > sampling_probability)
            {
                continue;
            }

            filtered_cloud.points.push_back(point);
        }

        // 更新点云数据
        filtered_cloud.width = filtered_cloud.points.size();
        filtered_cloud.height = 1;
        filtered_cloud.is_dense = true;

        cloud = filtered_cloud;
    }


    void publishMap(const ros::TimerEvent&)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // 对全局点云进行体素降采样以降低点云密度
        pcl::PointCloud<pcl::PointXYZ> downsampled_map;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(global_map_.makeShared());

        // 根据需求设置体素大小（例如，0.5米）
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(downsampled_map);

        // 将降采样后的点云转换为ROS消息格式
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(downsampled_map, cloud_msg);

        // 设置消息的时间戳和框架
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "camera_init";

        // 发布点云地图
        map_pub_.publish(cloud_msg);

        ROS_INFO("Published laser cloud map with %zu points (downsampled)", downsampled_map.points.size());
    }


private:
    void publishTF()
    {
        // 创建一个 tf 变换
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(robot_pose_.position.x, robot_pose_.position.y, robot_pose_.position.z));
        tf::Quaternion q(robot_pose_.orientation.x, robot_pose_.orientation.y, robot_pose_.orientation.z, robot_pose_.orientation.w);
        transform.setRotation(q);

        // 发布tf变换
        tf_broadcaster_->sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), "camera_init", "aft_mapped"));
    }

    void addPointCloudToMap(const pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        // 简单地将当前点云加入全局地图（未进行滤波或降采样，可根据需要优化）
        for (const auto& point : cloud.points)
        {
            pcl::PointXYZ transformed_point;
            transformPointToGlobal(point, transformed_point);
            global_map_.points.push_back(transformed_point);
        }
        global_map_.width = global_map_.points.size();
        global_map_.height = 1;
        global_map_.is_dense = true;
    }

    void transformPointToGlobal(const pcl::PointXYZ& point, pcl::PointXYZ& transformed_point)
    {
        // 将点云从局部坐标系转换到全局坐标系
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(robot_pose_.position.x, robot_pose_.position.y, robot_pose_.position.z));
        tf::Quaternion q(robot_pose_.orientation.x, robot_pose_.orientation.y, robot_pose_.orientation.z, robot_pose_.orientation.w);
        transform.setRotation(q);

        tf::Vector3 local_point(point.x, point.y, point.z);
        tf::Vector3 global_point = transform * local_point;

        transformed_point.x = global_point.x();
        transformed_point.y = global_point.y();
        transformed_point.z = global_point.z();
    }

    void filterAndDownsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        // 过滤掉距离超过20米的点
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
        for (const auto& point : cloud.points)
        {
            if (std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z) <= 20.0)
            {
                filtered_cloud.points.push_back(point);
            }
        }

        // 使用Voxel Grid滤波器进行降采样
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(filtered_cloud.makeShared());
        voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f);  // 设置体素大小为0.2米
        voxel_filter.filter(cloud);
    }

    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher map_pub_;
    ros::Timer timer_;

    std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::Pose robot_pose_;
    pcl::PointCloud<pcl::PointXYZ> global_map_;

    std::mutex mutex_; // 保护全局地图和位姿的互斥锁
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_generator_node");

    MapGenerator map_generator;

    ros::spin();

    return 0;
}
