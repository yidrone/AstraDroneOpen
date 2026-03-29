#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <mutex>
#include <memory>
#include <Eigen/Geometry>
#include <lidar_cam_fusion/EstimateDepth.h>  // 修改为当前包名

class CameraLidarFusion {
public:
    CameraLidarFusion() : 
        nh_("~"), 
        static_tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>()),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        camera_info_received_(false),
        tf_publish_timer_(),
        depth_accumulator_(),
        depth_count_(),
        frame_counter_(0)  // 初始化frame_counter_
    {
        // 加载所有参数
        loadParameters();
        
        // 初始化外参
        updateTransform();
        
        // 订阅相机内参话题
        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, 
                                       &CameraLidarFusion::cameraInfoCallback, this);
        
        // 订阅相机和雷达话题
        image_sub_.subscribe(nh_, image_topic_, 1);
        cloud_sub_.subscribe(nh_, cloud_topic_, 1);
        
        // 时间同步器 (近似时间同步)
        sync_.reset(new Sync(SyncPolicy(queue_size_), image_sub_, cloud_sub_));
        sync_->registerCallback(boost::bind(&CameraLidarFusion::fusionCallback, this, _1, _2));
        
        // 发布彩色点云
        colored_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(colored_cloud_topic_, 1);
        
        // 发布深度图像
        depth_image_pub_ = nh_.advertise<sensor_msgs::Image>(depth_image_topic_, 1);
        
        // 发布相机坐标系下的点云
        camera_frame_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(camera_frame_cloud_topic_, 1);
        
        // 设置定时器连续发布静态TF (10Hz)
        tf_publish_timer_ = nh_.createTimer(ros::Duration(0.1), 
                                          &CameraLidarFusion::publishStaticTFTimerCallback, this);
        
        // 初始化深度估计服务
        depth_service_ = nh_.advertiseService("estimate_depth", &CameraLidarFusion::estimateDepthCallback, this);
        
        ROS_INFO("Camera-Lidar fusion node initialized");
        ROS_INFO("Waiting for camera info...");
    }

    
    void loadParameters() {
        // 坐标系参数
        nh_.param<std::string>("camera_frame", camera_frame_, "camera_frame");
        nh_.param<std::string>("lidar_frame", lidar_frame_, "livox_frame");
        
        // 外参参数 (默认值)
        nh_.param<double>("extrinsics/x", x_, 0.0);
        nh_.param<double>("extrinsics/y", y_, 0.0);
        nh_.param<double>("extrinsics/z", z_, 0.0);
        nh_.param<double>("extrinsics/roll", roll_, 0.0);
        nh_.param<double>("extrinsics/pitch", pitch_, 0.0);
        nh_.param<double>("extrinsics/yaw", yaw_, 0.0);
        
        // 话题参数
        nh_.param<std::string>("topics/image", image_topic_, "/camera/image_raw");
        nh_.param<std::string>("topics/cloud", cloud_topic_, "/livox/lidar");
        nh_.param<std::string>("topics/camera_info", camera_info_topic_, "/camera/camera_info");
        nh_.param<std::string>("topics/colored_cloud", colored_cloud_topic_, "/fused/colored_cloud");
        nh_.param<std::string>("topics/depth_image", depth_image_topic_, "/fused/depth_image");
        nh_.param<std::string>("topics/camera_frame_cloud", camera_frame_cloud_topic_, "/fused/camera_frame_cloud");
        
        // 处理参数
        nh_.param<int>("processing/queue_size", queue_size_, 10);
        nh_.param<double>("processing/min_range", min_range_, 0.1);
        nh_.param<double>("processing/max_range", max_range_, 20.0);
        nh_.param<double>("processing/voxel_size", voxel_size_, 0.05);
        nh_.param<int>("processing/target_width", target_width_, 640);
        nh_.param<int>("processing/target_height", target_height_, 480);
        nh_.param<int>("processing/accumulation_frames", accumulation_frames_, 5);
        
        ROS_INFO("Loaded parameters:");
        ROS_INFO("  Frames: camera=%s, lidar=%s", camera_frame_.c_str(), lidar_frame_.c_str());
        ROS_INFO("  Extrinsics: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f",
                x_, y_, z_, roll_, pitch_, yaw_);
        ROS_INFO("  Topics:");
        ROS_INFO("    Image: %s", image_topic_.c_str());
        ROS_INFO("    Cloud: %s", cloud_topic_.c_str());
        ROS_INFO("    Camera info: %s", camera_info_topic_.c_str());
        ROS_INFO("    Output colored cloud: %s", colored_cloud_topic_.c_str());
        ROS_INFO("    Output depth image: %s", depth_image_topic_.c_str());
        ROS_INFO("    Output camera frame cloud: %s", camera_frame_cloud_topic_.c_str());
        ROS_INFO("  Processing:");
        ROS_INFO("    Queue size: %d", queue_size_);
        ROS_INFO("    Range: %.1f-%.1f meters", min_range_, max_range_);
        ROS_INFO("    Voxel size: %.3f meters", voxel_size_);
        ROS_INFO("    Accumulation frames: %d", accumulation_frames_);
        ROS_INFO("    Target depth image size: %dx%d", target_width_, target_height_);
    }

    bool estimateDepthCallback(lidar_cam_fusion::EstimateDepth::Request &req,
        lidar_cam_fusion::EstimateDepth::Response &res) 
    {
        // 首先获取相机信息锁
        std::lock_guard<std::mutex> lock_cam(camera_info_mutex_);
        
        // 检查是否已接收相机信息
        if (!camera_info_received_) {
            res.success = false;
            res.message = "Camera info not received";
            res.depth = 0.0;
            ROS_WARN("Depth estimation request failed: %s", res.message.c_str());
            return true;
        }

        // 检查原始图像尺寸是否有效
        if (image_width_ <= 0 || image_height_ <= 0) {
            res.success = false;
            res.message = "Invalid image dimensions";
            res.depth = 0.0;
            ROS_WARN("Depth estimation request failed: %s", res.message.c_str());
            return true;
        }

        // 现在获取深度图锁
        std::lock_guard<std::mutex> lock_depth(latest_depth_mutex_);

        if (latest_depth_map_.empty()) {
            res.success = false;
            res.message = "No depth data available yet";
            res.depth = 0.0;
            ROS_WARN("Depth estimation request failed: %s", res.message.c_str());
            return true;
        }

        int depth_width = latest_depth_map_.cols;
        int depth_height = latest_depth_map_.rows;

        // 将 RGB 分辨率下的坐标映射到 depth map 分辨率
        int scaled_u = static_cast<int>(req.u * static_cast<float>(depth_width) / image_width_ + 0.5f);
        int scaled_v = static_cast<int>(req.v * static_cast<float>(depth_height) / image_height_ + 0.5f);

        // 检查映射后的坐标是否合法
        if (scaled_u < 0 || scaled_u >= depth_width || scaled_v < 0 || scaled_v >= depth_height) {
            res.success = false;
            res.message = "Mapped coordinates out of bounds";
            res.depth = 0.0;
            ROS_WARN("Depth estimation request failed: %s (scaled_u=%d, scaled_v=%d, bounds=[0-%d, 0-%d])", 
                    res.message.c_str(), scaled_u, scaled_v, depth_width-1, depth_height-1);
            return true;
        }

        float center_depth = latest_depth_map_.at<float>(scaled_v, scaled_u);
        if (center_depth > 0.0f) {
            res.success = true;
            res.depth = center_depth;
            res.message = "Success (direct measurement)";
            ROS_DEBUG("Depth estimate at (%d, %d) [scaled: %d, %d]: %.3f meters", 
                    req.u, req.v, scaled_u, scaled_v, center_depth);
            return true;
        }

        // 在邻域搜索有效深度（已用缩放坐标）
        const int search_radius = 10;  // 缩小搜索半径以提高性能
        std::vector<std::pair<float, float>> distances_and_depths;

        for (int dv = -search_radius; dv <= search_radius; ++dv) {
            for (int du = -search_radius; du <= search_radius; ++du) {
                if (du == 0 && dv == 0) continue;

                int u_neighbor = scaled_u + du;
                int v_neighbor = scaled_v + dv;

                if (u_neighbor < 0 || u_neighbor >= depth_width || v_neighbor < 0 || v_neighbor >= depth_height)
                    continue;

                float neighbor_depth = latest_depth_map_.at<float>(v_neighbor, u_neighbor);
                if (neighbor_depth > 0.0f) {
                    float distance = std::sqrt(static_cast<float>(du * du + dv * dv));
                    distances_and_depths.emplace_back(distance, neighbor_depth);
                }
            }
        }

        if (distances_and_depths.empty()) {
            res.success = false;
            res.depth = 0.0;
            res.message = "No valid depth found in neighborhood";
            ROS_WARN("Depth estimation failed at (%d, %d) [scaled: %d, %d]: no valid neighbor", 
                    req.u, req.v, scaled_u, scaled_v);
            return true;
        }

        std::sort(distances_and_depths.begin(), distances_and_depths.end(),
            [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                return a.first < b.first;
            });

        size_t count = std::min<size_t>(3, distances_and_depths.size());
        float depth_sum = 0.0f;
        for (size_t i = 0; i < count; ++i) {
            depth_sum += distances_and_depths[i].second;
        }

        res.success = true;
        res.depth = depth_sum / count;
        res.message = "Success (average of nearest 3 depths)";
        ROS_DEBUG("Depth estimate at (%d, %d) [scaled: %d, %d]: average of %ld depths = %.3f", 
                req.u, req.v, scaled_u, scaled_v, count, res.depth);
        return true;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg) {
        if(camera_info_received_) return;
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        
        // 检查关键参数有效性
        if (info_msg->K.size() != 9) {
            ROS_ERROR("CameraInfo K matrix must have 9 elements, got %zu", info_msg->K.size());
            return;
        }
        if (info_msg->width <= 0 || info_msg->height <= 0) {
            ROS_ERROR("Invalid image dimensions: %dx%d", info_msg->width, info_msg->height);
            return;
        }

        // 解析相机内参矩阵 (3x3)
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; ++i) {
            camera_matrix_.at<double>(i/3, i%3) = info_msg->K[i];
        }
        
        // 解析畸变系数
        dist_coeffs_ = cv::Mat(info_msg->D.size(), 1, CV_64F);
        for (size_t i = 0; i < info_msg->D.size(); ++i) {
            dist_coeffs_.at<double>(i, 0) = info_msg->D[i];
        }
        
        // 获取图像尺寸
        image_width_ = info_msg->width;
        image_height_ = info_msg->height;
        
        // 计算去畸变映射
        cv::initUndistortRectifyMap(
            camera_matrix_, dist_coeffs_, cv::Mat(),
            camera_matrix_, cv::Size(image_width_, image_height_),
            CV_32FC1, undistort_map1_, undistort_map2_);
        
        // 计算缩放比例
        double scale_x = static_cast<double>(target_width_) / image_width_;
        double scale_y = static_cast<double>(target_height_) / image_height_;
        
        // 计算目标分辨率的内参
        scaled_camera_matrix_ = camera_matrix_.clone();
        scaled_camera_matrix_.at<double>(0, 0) *= scale_x; // fx
        scaled_camera_matrix_.at<double>(1, 1) *= scale_y; // fy
        scaled_camera_matrix_.at<double>(0, 2) *= scale_x; // cx
        scaled_camera_matrix_.at<double>(1, 2) *= scale_y; // cy
        
        // 为深度累加器分配空间
        depth_accumulator_ = cv::Mat::zeros(target_height_, target_width_, CV_32FC1);
        depth_count_ = cv::Mat::zeros(target_height_, target_width_, CV_32SC1);
        
        camera_info_received_ = true;
        
        ROS_INFO("Received camera info:");
        ROS_INFO("  Image size: %dx%d", image_width_, image_height_);
        ROS_INFO("  Camera matrix:\n    [%.3f, %.3f, %.3f]\n    [%.3f, %.3f, %.3f]\n    [%.3f, %.3f, %.3f]",
                 camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(0,1), camera_matrix_.at<double>(0,2),
                 camera_matrix_.at<double>(1,0), camera_matrix_.at<double>(1,1), camera_matrix_.at<double>(1,2),
                 camera_matrix_.at<double>(2,0), camera_matrix_.at<double>(2,1), camera_matrix_.at<double>(2,2));
        ROS_INFO("  Distortion coefficients: [%.4f, %.4f, %.4f, %.4f, %.4f]",
                 dist_coeffs_.at<double>(0), dist_coeffs_.at<double>(1),
                 dist_coeffs_.at<double>(2), dist_coeffs_.at<double>(3),
                 dist_coeffs_.at<double>(4));
        ROS_INFO("  Scaled camera matrix for %dx%d:\n    [%.3f, %.3f, %.3f]\n    [%.3f, %.3f, %.3f]\n    [%.3f, %.3f, %.3f]",
                 target_width_, target_height_,
                 scaled_camera_matrix_.at<double>(0,0), scaled_camera_matrix_.at<double>(0,1), scaled_camera_matrix_.at<double>(0,2),
                 scaled_camera_matrix_.at<double>(1,0), scaled_camera_matrix_.at<double>(1,1), scaled_camera_matrix_.at<double>(1,2),
                 scaled_camera_matrix_.at<double>(2,0), scaled_camera_matrix_.at<double>(2,1), scaled_camera_matrix_.at<double>(2,2));
    }
    
    void updateTransform() {
        // 创建外参变换矩阵 (雷达->相机)
        Eigen::Translation3d translation(x_, y_, z_);
        Eigen::AngleAxisd roll_angle(roll_, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(pitch_, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(yaw_, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;

        lidar_to_cam_transform_ = translation * q;

        // 反向变换 Camera -> Lidar（即发布使用的）
        Eigen::Affine3d cam_to_lidar = lidar_to_cam_transform_.inverse();
        Eigen::Translation3d inv_translation(cam_to_lidar.translation());
        Eigen::Quaterniond inv_q(cam_to_lidar.rotation());

        // 更新 TF 消息
        {
            std::lock_guard<std::mutex> lock(tf_mutex_);
            cached_tf_.header.stamp = ros::Time::now();
            cached_tf_.header.frame_id = lidar_frame_;       // 父坐标系：camera
            cached_tf_.child_frame_id = camera_frame_;         // 子坐标系：livox lidar

            cached_tf_.transform.translation.x = inv_translation.x();
            cached_tf_.transform.translation.y = inv_translation.y();
            cached_tf_.transform.translation.z = inv_translation.z();

            cached_tf_.transform.rotation.x = inv_q.x();
            cached_tf_.transform.rotation.y = inv_q.y();
            cached_tf_.transform.rotation.z = inv_q.z();
            cached_tf_.transform.rotation.w = inv_q.w();
        }

        // 打印外参信息
        ROS_INFO("Updated transform (Lidar->Camera):");
        ROS_INFO("  Translation: [%.3f, %.3f, %.3f]", x_, y_, z_);
        ROS_INFO("  Rotation (RPY): [%.3f, %.3f, %.3f]", roll_, pitch_, yaw_);
    }

    void publishStaticTFTimerCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        cached_tf_.header.stamp = ros::Time::now();
        static_tf_broadcaster_->sendTransform(cached_tf_);
    }
    
    void fusionCallback(const sensor_msgs::ImageConstPtr& img_msg, 
        const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        if (!camera_info_received_) {
            ROS_WARN_THROTTLE(1.0, "Waiting for camera info...");
            return;
        }

        try {
            // 图像处理：去畸变 + 缩放
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
            cv::Mat image = cv_ptr->image;
            cv::Mat undistorted_image, resized_image;
            {
                std::lock_guard<std::mutex> lock(camera_info_mutex_);
                cv::remap(image, undistorted_image, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);
            }
            cv::resize(undistorted_image, resized_image, cv::Size(target_width_, target_height_));

            // 点云转换与滤波
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud_raw);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            preprocessPointCloud(cloud_raw, cloud_filtered);

            // 点云坐标变换（Lidar → Camera）
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud_filtered, *cloud_transformed, lidar_to_cam_transform_);

            // 输出对象准备
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr camera_frame_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cv::Mat depth_map = cv::Mat::zeros(target_height_, target_width_, CV_32FC1);

            // 遍历点云 → 投影到图像平面
            for (const auto& pt : *cloud_transformed) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0.005)
                    continue;

                camera_frame_cloud->push_back(pt);

                cv::Mat pt3d = (cv::Mat_<double>(3, 1) << pt.x, pt.y, pt.z);
                cv::Mat uv;
                {
                    std::lock_guard<std::mutex> lock(camera_info_mutex_);
                    uv = scaled_camera_matrix_ * pt3d;
                }

                double u = uv.at<double>(0) / uv.at<double>(2);
                double v = uv.at<double>(1) / uv.at<double>(2);
                int px = static_cast<int>(u + 0.5);
                int py = static_cast<int>(v + 0.5);

                if (px >= 0 && px < target_width_ && py >= 0 && py < target_height_) {
                    cv::Vec3b color = resized_image.at<cv::Vec3b>(py, px);
                    pcl::PointXYZRGB pt_rgb;
                    pt_rgb.x = pt.x;
                    pt_rgb.y = pt.y;
                    pt_rgb.z = pt.z;
                    pt_rgb.r = color[2];
                    pt_rgb.g = color[1];
                    pt_rgb.b = color[0];
                    colored_cloud->push_back(pt_rgb);

                    float& depth_val = depth_map.at<float>(py, px);
                    if (depth_val == 0 || pt.z < depth_val) {
                        depth_val = pt.z;
                    }
                }
            }

            // 深度图累积逻辑
            {
                std::lock_guard<std::mutex> lock(accumulator_mutex_);
                for (int y = 0; y < target_height_; ++y) {
                    for (int x = 0; x < target_width_; ++x) {
                        float d = depth_map.at<float>(y, x);
                        if (d > 0) {
                            depth_accumulator_.at<float>(y, x) += d;
                            depth_count_.at<int>(y, x) += 1;
                        }
                    }
                }

                frame_counter_++;
                if (frame_counter_ >= accumulation_frames_) {
                    cv::Mat averaged_depth = cv::Mat::zeros(target_height_, target_width_, CV_32FC1);
                    for (int y = 0; y < target_height_; ++y) {
                        for (int x = 0; x < target_width_; ++x) {
                            int count = depth_count_.at<int>(y, x);
                            if (count > 0) {
                                averaged_depth.at<float>(y, x) = depth_accumulator_.at<float>(y, x) / count;
                            }
                        }
                    }

                    {
                        std::lock_guard<std::mutex> lock_depth(latest_depth_mutex_);
                        latest_depth_map_ = averaged_depth.clone();
                    }

                    // 发布深度图
                    cv_bridge::CvImage depth_bridge;
                    depth_bridge.header = img_msg->header;
                    depth_bridge.encoding = "32FC1";
                    depth_bridge.image = latest_depth_map_;
                    depth_image_pub_.publish(depth_bridge.toImageMsg());

                    // 重置累加器
                    frame_counter_ = 0;
                    depth_accumulator_ = cv::Mat::zeros(target_height_, target_width_, CV_32FC1);
                    depth_count_ = cv::Mat::zeros(target_height_, target_width_, CV_32SC1);
                }
            }

            // 发布相机坐标系点云
            sensor_msgs::PointCloud2 camera_cloud_msg;
            pcl::toROSMsg(*camera_frame_cloud, camera_cloud_msg);
            camera_cloud_msg.header = cloud_msg->header;
            camera_cloud_msg.header.frame_id = camera_frame_;
            camera_frame_cloud_pub_.publish(camera_cloud_msg);

            // 发布彩色点云
            sensor_msgs::PointCloud2 colored_msg;
            pcl::toROSMsg(*colored_cloud, colored_msg);
            colored_msg.header = cloud_msg->header;
            colored_msg.header.frame_id = camera_frame_;
            colored_cloud_pub_.publish(colored_msg);

            ROS_INFO_THROTTLE(1.0, "Published %ld colored points, %ld raw points.", 
                        colored_cloud->size(), camera_frame_cloud->size());
        }
        catch (const std::exception& e) {
            ROS_ERROR("Fusion error: %s", e.what());
        }
    }

private:
    void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
        // 创建滤波对象
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(input);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_range_, max_range_);
        pass.filter(*output);
        
        // 体素网格下采样
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(output);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel.filter(*output);
    }
    
    void interpolateDepth(const cv::Mat& depth_map, cv::Mat& filled_depth) {
        // 创建掩膜 (有效深度区域)
        cv::Mat mask = (depth_map > 0);
        
        // 创建坐标网格
        cv::Mat valid_points;
        cv::findNonZero(mask, valid_points);
        
        // 准备插值数据
        cv::Mat points(valid_points.rows, 2, CV_32F);
        cv::Mat values(valid_points.rows, 1, CV_32F);
        
        for (int i = 0; i < valid_points.rows; i++) {
            cv::Point pt = valid_points.at<cv::Point>(i);
            points.at<float>(i, 0) = pt.x;
            points.at<float>(i, 1) = pt.y;
            values.at<float>(i) = depth_map.at<float>(pt.y, pt.x);
        }
        
        // 创建网格用于插值
        cv::Mat grid_x = cv::Mat::zeros(depth_map.size(), CV_32F);
        cv::Mat grid_y = cv::Mat::zeros(depth_map.size(), CV_32F);
        
        for (int r = 0; r < depth_map.rows; r++) {
            for (int c = 0; c < depth_map.cols; c++) {
                grid_x.at<float>(r, c) = c;
                grid_y.at<float>(r, c) = r;
            }
        }
        
        // 最近邻插值
        filled_depth = cv::Mat::zeros(depth_map.size(), CV_32F);
        
        #pragma omp parallel for
        for (int r = 0; r < depth_map.rows; r++) {
            for (int c = 0; c < depth_map.cols; c++) {
                if (mask.at<uchar>(r, c)) {
                    filled_depth.at<float>(r, c) = depth_map.at<float>(r, c);
                } else {
                    // 找到最近的已知点
                    float min_dist = FLT_MAX;
                    float nearest_depth = 0;
                    
                    for (int i = 0; i < points.rows; i++) {
                        float px = points.at<float>(i, 0);
                        float py = points.at<float>(i, 1);
                        float dist = sqrt((px - c) * (px - c) + (py - r) * (py - r));
                        
                        if (dist < min_dist) {
                            min_dist = dist;
                            nearest_depth = values.at<float>(i);
                        }
                    }
                    
                    filled_depth.at<float>(r, c) = nearest_depth;
                }
            }
        }
        
        // 中值滤波平滑结果
        cv::medianBlur(filled_depth, filled_depth, 3);
    }

    ros::NodeHandle nh_;
    
    // 坐标系
    std::string camera_frame_;
    std::string lidar_frame_;
    
    // 外参参数
    double x_, y_, z_;
    double roll_, pitch_, yaw_;
    Eigen::Affine3d lidar_to_cam_transform_;
    
    // TF相关
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::TransformStamped cached_tf_;
    std::mutex tf_mutex_;
    ros::Timer tf_publish_timer_;
    
    // 订阅器和发布器
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher colored_cloud_pub_;
    ros::Publisher depth_image_pub_;
    ros::Publisher camera_frame_cloud_pub_;
    
    // 同步器
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
    
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat scaled_camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat undistort_map1_, undistort_map2_;
    int image_width_ = 0;
    int image_height_ = 0;
    bool camera_info_received_;
    std::mutex camera_info_mutex_;

    // 深度图处理
    cv::Mat depth_accumulator_;
    cv::Mat depth_count_;
    int frame_counter_ = 0;
    int accumulation_frames_;
    int target_width_;
    int target_height_;
    std::mutex accumulator_mutex_;

    // 话题参数
    std::string image_topic_;
    std::string cloud_topic_;
    std::string camera_info_topic_;
    std::string colored_cloud_topic_;
    std::string depth_image_topic_;
    std::string camera_frame_cloud_topic_;
    int queue_size_;
    double min_range_;
    double max_range_;
    double voxel_size_;

    // 深度估计服务相关
    ros::ServiceServer depth_service_;
    cv::Mat latest_depth_map_;
    std::mutex latest_depth_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_lidar_fusion");
    CameraLidarFusion fusion_node;
    ros::spin();
    return 0;
}