#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <astra_custom_msgs/MarkerMeasurement.h>
#include <astra_custom_msgs/MarkerMeasurementArray.h>
#include <std_srvs/Trigger.h>

#include <experimental/filesystem>

#include <sensor_msgs/CameraInfo.h>  
#include <yaml-cpp/yaml.h>  

#include <tf/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>  

namespace aruco_localizer {

    class ArucoLocalizer
    {
    public:
        ArucoLocalizer();

    private:

        // aruco::MarkerDetector::Params mDetector_;
        int CORNER_LINES=1;
        // int{CORNER_SUBPIX=0,CORNER_LINES=1,CORNER_NONE=2};

        
        // ROS node handles
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // image transport pub/sub
        image_transport::ImageTransport it_;
        image_transport::CameraSubscriber image_sub_;
        image_transport::Publisher image_pub_;

        // ROS tf listener and broadcaster
        tf::TransformListener tf_listener_;
        tf::TransformBroadcaster tf_br_;

        // Bias for the roll and pitch components of camera to body
        tf::Quaternion quat_att_bias_;

        // ROS publishers and subscribers
        ros::Publisher estimate_pub_;
        ros::Publisher meas_pub_;
        ros::Publisher states_pub_;
        ros::ServiceServer calib_attitude_;

        // ArUco Map Detector
        double markerSize_ = 0.1;
        // sw_size/real_size    
        // sw=2.608
        float k_SW2Real = 2.173;
        std::vector<double> markerSize__;    // Read in ROS params
        std::vector<std::vector<float>> markerSize = {{}};

        aruco::MarkerMap mmConfig_;
        aruco::MarkerDetector mDetector_;
        aruco::MarkerMapPoseTracker mmPoseTracker_;
        aruco::CameraParameters camParams_;

        bool showOutputVideo_;
        bool debugSaveInputFrames_;
        bool debugSaveOutputFrames_;
        std::string debugImagePath_;
        sensor_msgs::CameraInfo cam_info;  



        //
        // Methods
        //

        // image_transport camera subscriber
        void cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo);

        // service handlers
        bool calibrateAttitude(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // This is where the real ArUco processing is done
        void processImage(cv::Mat& frame, bool drawDetections);

        // Convert ROS CameraInfo message to ArUco style CameraParameters
        aruco::CameraParameters ros2arucoCamParams(const sensor_msgs::CameraInfo& cinfo);

        // functions to convert to tf and then broadcast
        tf::Quaternion rodriguesToTFQuat(const cv::Mat& rvec);
        tf::Transform aruco2tf(const cv::Mat& rvec, const cv::Mat& tvec);
        void sendtf(const cv::Mat& rvec, const cv::Mat& tvec);

        // Save the current frame to file. Useful for debugging
        void saveInputFrame(const cv::Mat& frame);
        void saveOutputFrame(const cv::Mat& frame);
        void saveFrame(const cv::Mat& frame, std::string format_spec, unsigned int img_num);
        void loadCameraParams(const std::string& yaml_file_path, sensor_msgs::CameraInfo& cam_info);
        geometry_msgs::PointStamped camera_to_world (const geometry_msgs::PointStamped& p_c);

    };

}