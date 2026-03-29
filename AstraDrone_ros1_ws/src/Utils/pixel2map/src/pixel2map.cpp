#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

// 自定义消息类型
#include "pixel2map/PredictResult.h"

// 相机参数结构体
struct CameraParams {
    double fx, fy, cx, cy;
    std::vector<double> rotation;
    std::vector<double> translation;
    double z_depth;
};

// 读取YAML文件
bool loadCameraParams(const std::string& yaml_file, CameraParams& params) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);
        params.fx = config["camera_intrinsic"]["fx"].as<double>();
        params.fy = config["camera_intrinsic"]["fy"].as<double>();
        params.cx = config["camera_intrinsic"]["cx"].as<double>();
        params.cy = config["camera_intrinsic"]["cy"].as<double>();
        params.rotation = config["camera_extrinsic"]["rotation"].as<std::vector<double>>();
        params.translation = config["camera_extrinsic"]["translation"].as<std::vector<double>>();
        params.z_depth = config["z_depth"].as<double>();
        return true;
    } catch (std::exception& e) {
        ROS_ERROR("Failed to load camera parameters: %s", e.what());
        return false;
    }
}

// 像素平面坐标到相机坐标系
geometry_msgs::Point pixelToCamera(double u, double v, const CameraParams& params) {
    geometry_msgs::Point camera_point;
    camera_point.x = (u - params.cx) * params.z_depth / params.fx;
    camera_point.y = (v - params.cy) * params.z_depth / params.fy;
    camera_point.z = params.z_depth;
    return camera_point;
}

// 相机坐标系到机体坐标系
geometry_msgs::Point cameraToBody(const geometry_msgs::Point& camera_point, const CameraParams& params) {
    geometry_msgs::Point body_point;
    body_point.x = params.rotation[0] * camera_point.x + params.rotation[1] * camera_point.y + params.rotation[2] * camera_point.z + params.translation[0];
    body_point.y = params.rotation[3] * camera_point.x + params.rotation[4] * camera_point.y + params.rotation[5] * camera_point.z + params.translation[1];
    body_point.z = params.rotation[6] * camera_point.x + params.rotation[7] * camera_point.y + params.rotation[8] * camera_point.z + params.translation[2];
    return body_point;
}

// 回调函数
void resultCallback(const pixel2map::PredictResult::ConstPtr& msg, tf2_ros::Buffer& tf_buffer, const CameraParams& params,
                    const std::string& base_frame, const std::string& world_frame) {
    for (size_t i = 0; i < msg->centre_x.size(); ++i) {
        // 提取像素坐标
        double u = msg->centre_x[i];
        double v = msg->centre_y[i];

        // 像素平面 -> 相机坐标系
        geometry_msgs::Point camera_point = pixelToCamera(u, v, params);

        // 相机坐标系 -> 机体坐标系
        geometry_msgs::Point body_point = cameraToBody(camera_point, params);

        // 机体坐标系 -> 世界坐标系 (通过TF)
        geometry_msgs::PointStamped body_point_stamped, world_point_stamped;
        body_point_stamped.point = body_point;
        body_point_stamped.header.frame_id = base_frame;
        body_point_stamped.header.stamp = ros::Time(0);

        try {
            tf_buffer.transform(body_point_stamped, world_point_stamped, world_frame, ros::Duration(1.0));
            ROS_INFO("World coordinates: [%.2f, %.2f, %.2f]", world_point_stamped.point.x, world_point_stamped.point.y, world_point_stamped.point.z);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pixel2map");
    ros::NodeHandle nh_private("~");

    // 加载参数
    std::string yaml_file, pixel_result_topic, base_frame, world_frame;
    nh_private.param<std::string>("camera_params_file", yaml_file, "");
    nh_private.param<std::string>("result_topic", pixel_result_topic, "/yolov10_predict_node/rgb/result");
    nh_private.param<std::string>("base_frame", base_frame, "base_link");
    nh_private.param<std::string>("world_frame", world_frame, "world");

    // 可用于是否成功传参
    // std::cout<<"yaml_file = "<<yaml_file<<std::endl;

    // 加载相机参数
    CameraParams params;
    if (!loadCameraParams(yaml_file, params)) {
        return -1;
    }

    // TF监听器
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // 订阅话题
    ros::Subscriber sub = nh_private.subscribe<pixel2map::PredictResult>(
        pixel_result_topic, 10, boost::bind(resultCallback, _1, boost::ref(tf_buffer), boost::ref(params), base_frame, world_frame));

    ros::spin();
    return 0;
}
