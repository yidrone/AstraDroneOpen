#include "aruco_localization/ArucoLocalizer.h"
#include <markerdetector.h>
#include <sensor_msgs/CameraInfo.h>  
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include "astra_custom_msgs/States.h" 
#include <tf2_ros/buffer.h>  
#include <tf2_ros/transform_listener.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  
#include <geometry_msgs/PointStamped.h> 
#include <tf/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

namespace aruco_localizer {

// ----------------------------------------------------------------------------
ArucoLocalizer::ArucoLocalizer() :
    nh_(ros::NodeHandle()), nh_private_("~"), it_(nh_)
{
    std::string mmConfigFile = nh_private_.param<std::string>("markermap_config", "");
    std::string calibration_file = nh_private_.param<std::string>("calibration_file", "");


    nh_private_.param<bool>("show_output_video", showOutputVideo_, false);
    nh_private_.param<bool>("debug_save_input_frames", debugSaveInputFrames_, false);
    nh_private_.param<bool>("debug_save_output_frames", debugSaveOutputFrames_, false);
    nh_private_.param<std::string>("debug_image_path", debugImagePath_, "/tmp/arucoimages");
    nh_private_.param<float>("k_SW2Real", k_SW2Real, 3.26);


    // Subscribe to input video feed and publish output video feed
    it_ = image_transport::ImageTransport(nh_);
    image_sub_ = it_.subscribeCamera("input_image", 1, &ArucoLocalizer::cameraCallback, this);
    image_pub_ = it_.advertise("output_image", 1);
    
    //读取相机内参文件
    ROS_INFO("Ready to read yaml.");
    YAML::Node camera_info = YAML::LoadFile(calibration_file);  
    
    //解析加载内参
    loadCameraParams(calibration_file, cam_info);  
    //std::cout << cam_info << std::endl;
    ROS_INFO("Read yaml complete. ");

    // Create ROS publishers
    estimate_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("/aruco/PoseStamped/measurements", 1);
    // meas_pub_ = nh_private_.advertise<astra_custom_msgs::MarkerMeasurement>("measurements", 1);
    states_pub_ = nh_private_.advertise<astra_custom_msgs::States>( "/states", 10) ; 
    
    // Create ROS services
    calib_attitude_ = nh_private_.advertiseService("calibrate_attitude", &ArucoLocalizer::calibrateAttitude, this);

    // Set up the Marker Map dimensions, spacing, dictionary, etc from the YAML
    mmConfig_.readFromFile(k_SW2Real,mmConfigFile);
    mDetector_.setDictionary(mmConfig_.getDictionary());
    mDetector_.getParameters().setCornerRefinementMethod((aruco::CornerRefinementMethod) 1);

    // set markmap size to meter. Convert to meters if necessary
    if (mmConfig_.isExpressedInPixels())
        //really, it's function is mm -> m.
        mmConfig_ = mmConfig_.convertToMeters_vector(markerSize);

    // Initialize the attitude bias to zero
    quat_att_bias_.setRPY(0, 0, 0);

    // Create the `debug_image_path` if it doesn't exist
    std::experimental::filesystem::create_directories(debugImagePath_);
}
    
bool ArucoLocalizer::calibrateAttitude(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    // Get the latest attitude in the body frame (is this in the body frame?)
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("base", "chiny", ros::Time(0), transform);

    // Store the old bias correction term to correctly capture the original biased attitude
    tf::Quaternion q0(quat_att_bias_.x() ,quat_att_bias_.y(), quat_att_bias_.z(), quat_att_bias_.w());

    // extract the inverse of the current attitude, unbiased using the current quat bias term
    // Get the original biased attitude by multiplying by the old bias correction term
    quat_att_bias_ = transform.getRotation()*q0;

    // Let the caller know what the new zeroed setpoint is
    double r, p, y;
    tf::Matrix3x3(quat_att_bias_).getRPY(r,p,y);
    res.success = true;
    res.message = "Zeroed at: R=" + std::to_string(r*(180.0/M_PI)) + ", P=" + std::to_string(p*(180.0/M_PI));
    return true;
}

geometry_msgs::PointStamped ArucoLocalizer::camera_to_world (const geometry_msgs::PointStamped& p_c)
{	
    //  mavros/local_position/odom -- local_origin 
    std::string map_frame = "fcu";
    // 自行修改
    std::string camera_frame = "down_cam_frame";
    
    tf::TransformListener* listener_ptr;
    tf::TransformListener *tfListener_;
    tfListener_ = new tf::TransformListener();

    geometry_msgs::PointStamped camera;
    geometry_msgs::PointStamped world;
    camera.header.frame_id = camera_frame;
    camera.point.x = p_c.point.x;
    camera.point.y = p_c.point.y;
    camera.point.z = p_c.point.z;

    if (!tfListener_->waitForTransform(map_frame, camera_frame, ros::Time(0), ros::Duration(1)))
	{
		ROS_ERROR("Could not get transform from %s to %s after 1 second!", map_frame.c_str(), camera_frame.c_str());
	}
    tfListener_->transformPoint(map_frame,camera,world);
    //cout<<"world"<<world.point.x << ","<<world.point.y << ","<<world.point.z << endl;
    world.header.frame_id = map_frame;
    return world;
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::sendtf(const cv::Mat& rvec, const cv::Mat& tvec) {
    // We want all transforms to use the same exact time
    ros::Time now = ros::Time::now();

    // std::cout<<"rvec"<<""<<rvec<<std::endl;
    // std::cout<<"tvec"<<tvec<<std::endl;
    // Create the transform from the camera to the ArUco Marker Map
    tf::Transform transform = aruco2tf(rvec, tvec);
    
    // Note that `transform` is a measurement of the ArUco map w.r.t the camera,
    // therefore the inverse gives the transform from `aruco` to `camera`.
    // tf_br_.sendTransform(tf::StampedTransform(transform.inverse(), now, "aruco", "camera"));
    tf_br_.sendTransform(tf::StampedTransform(transform, now, "camera", "target"));

    geometry_msgs::PoseStamped poseMsg;
    tf::poseTFToMsg(transform, poseMsg.pose);

    //转换成y p r看看
    // double yaw,pitch,roll;
    // tf::Quaternion q(
    //     poseMsg.pose.orientation.x,
    //     poseMsg.pose.orientation.y,
    //     poseMsg.pose.orientation.z,
    //     poseMsg.pose.orientation.w
    // );
    // tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    // std::cout<<"yaw:"<<yaw<<"pitch:"<<pitch<<"roll:"<<roll<<std::endl;

    poseMsg.header.frame_id = "camera";
    poseMsg.header.stamp = now;
    estimate_pub_.publish(poseMsg);
}



// 处理图像，重要函数之一
void ArucoLocalizer::processImage(cv::Mat& frame, bool drawDetections) {

    // Detection of the board
    std::vector<aruco::Marker> detected_markers = mDetector_.detect(frame);

    //画出边界
    if (drawDetections) {
        // print the markers detected that belongs to the markerset
        for (auto idx : mmConfig_.getIndices(detected_markers))
            detected_markers[idx].draw(frame, cv::Scalar(0, 0, 255), 1);
            // cv::imshow("frame",frame);
    }

    // Calculate pose of each individual marker w.r.t the camera
    astra_custom_msgs::States states_msg;
    astra_custom_msgs::MarkerMeasurementArray measurement_msg;
    cv::Point2f center_x_y_pix;

    measurement_msg.header.frame_id = "camera";
    measurement_msg.header.stamp = ros::Time::now();
    astra_custom_msgs::MarkerMeasurement msg_pub;

    //可能作者觉得官方的函数不好用，所以自己写的求解位姿的代码，下面的位姿估计作用也是一样（猜测）
    for (auto marker : detected_markers) {

        // markerSize_ = markerSize[0][0];
        // Create Tvec, Rvec based on the camera and marker geometry

        marker.calculateExtrinsics(markerSize_, camParams_, false);

        // std::cout<<"_rvec2->"<<marker.Rvec<<std::endl;
        // std::cout<<"_tvec2->"<<marker.Tvec<<std::endl;

        // Create the ROS pose message and add to the array
        astra_custom_msgs::MarkerMeasurement msg;
        // msg.position.x = marker.Tvec.at<float>(0);
        msg.position.y = -marker.Tvec.at<float>(0);
        // msg.position.y = marker.Tvec.at<float>(1);
        msg.position.x = -marker.Tvec.at<float>(1);
        msg.position.z = -marker.Tvec.at<float>(2);

        // pluging states_msg 's x and y in pix form.
        center_x_y_pix = marker.getCenter();
        states_msg.Xc = center_x_y_pix.x;
        states_msg.Yc = center_x_y_pix.y;

        // Represent Rodrigues parameters as a quaternion
        tf::Quaternion quat = rodriguesToTFQuat(marker.Rvec);
        tf::quaternionTFToMsg(quat, msg.orientation);

        // Extract Euler angles
        double r, p, y;
        tf::Matrix3x3(quat).getRPY(r,p,y);

        msg.euler.x = r*180/M_PI;
        msg.euler.y = p*180/M_PI;
        msg.euler.z = y*180/M_PI;

        states_msg.Theta = y*180/M_PI;

        // attach the ArUco ID to this measurement
        // std::cout<<msg.position<<std::endl;
        // std::cout<<msg.aruco_id<<std::endl;

        measurement_msg.poses.push_back(msg);
        msg_pub = msg;
    }

    //发布测量值
    // meas_pub_.publish(msg_pub);
    //state msg push
    states_pub_.publish(states_msg);

    //相对位姿估计
    // If the Pose Tracker was properly initialized, find 3D pose information
    if (mmPoseTracker_.isValid()) {
        // std::cout <<"mmPoseTracker_ is_Valid"<< std::endl;
        if (mmPoseTracker_.estimatePose(detected_markers)) {
            // std::cout <<"mmPoseTracker_ estimatePose"<< std::endl;
            if (drawDetections)
                aruco::CvDrawingUtils::draw3dAxis(frame, camParams_, mmPoseTracker_.getRvec(), mmPoseTracker_.getTvec(), mmConfig_[0].getMarkerSize()*2);
            sendtf(mmPoseTracker_.getRvec(), mmPoseTracker_.getTvec());
        }
    }
}

// ----------------------------------------------------------------------------
//图像处理回调
void ArucoLocalizer::cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Configure the Pose Tracker if it has not been configured before
    if (!mmPoseTracker_.isValid() && mmConfig_.isExpressedInMeters()) {
       ROS_WARN("Init  the Pose Tracker");
        // Extract ROS camera_info (i.e., K and D) for ArUco library
        camParams_ = ros2arucoCamParams(cam_info);
        // Now, if the camera params have been ArUco-ified, set up the tracker
        if (camParams_.isValid())
        {
            ROS_WARN("Init  the Pose Tracker error try again.");
            mmPoseTracker_.setParams(camParams_, mmConfig_);
            ROS_WARN("Init  the Pose Tracker ok.");
        }

    }
        
    // Process the incoming video frame

    // Get image as a regular Mat
    cv::Mat frame = cv_ptr->image;

    if (debugSaveInputFrames_) saveInputFrame(frame);

    // Process the image and do ArUco localization on it
    //main process 
    processImage(frame, showOutputVideo_);

    if (debugSaveOutputFrames_) saveOutputFrame(frame);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}


aruco::CameraParameters ArucoLocalizer::ros2arucoCamParams(const sensor_msgs::CameraInfo& cinfo) {
    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    cv::Mat distortionCoeff(4, 1, CV_64FC1);
    cv::Size size(cinfo.height, cinfo.width);

    // Make a regular 3x3 K matrix from CameraInfo
    for(int i=0; i<9; ++i)
        cameraMatrix.at<double>(i%3, i-(i%3)*3) = cinfo.K[i];

    // The ArUco library requires that there are only 4 distortion params (k1, k2, p1, p2, 0) 
    if (cinfo.D.size() == 4 || cinfo.D.size() == 5) {

        // Make a regular 4x1 D matrix from CameraInfo
        for(int i=0; i<4; ++i)
            distortionCoeff.at<double>(i, 0) = cinfo.D[i];

    } else {

        ROS_WARN("[aruco] Length of distortion matrix is not 4, assuming zero distortion.");
        for(int i=0; i<4; ++i)
            distortionCoeff.at<double>(i, 0) = 0;
    }
    return aruco::CameraParameters(cameraMatrix, distortionCoeff, size);
}

// From camera frame to ArUco marker
tf::Transform ArucoLocalizer::aruco2tf(const cv::Mat& rvec, const cv::Mat& tvec) {
    // convert tvec to a double
    cv::Mat tvec64; tvec.convertTo(tvec64, CV_64FC1);

    // Convert Rodrigues paramaterization of the rotation to quat
    tf::Quaternion q1 = rodriguesToTFQuat(rvec);

    tf::Vector3 origin(tvec64.at<double>(0), tvec64.at<double>(1), tvec64.at<double>(2));

    // The measurements coming from the ArUco lib are vectors from the
    // camera coordinate system pointing at the center of the ArUco board.
    return tf::Transform(q1, origin);
}

tf::Quaternion ArucoLocalizer::rodriguesToTFQuat(const cv::Mat& rvec) {
    // convert rvec to double
    cv::Mat rvec64; rvec.convertTo(rvec64, CV_64FC1);

    // Unpack Rodrigues paramaterization of the rotation
    cv::Mat rot(3, 3, CV_64FC1);
    cv::Rodrigues(rvec64, rot);

    // Convert OpenCV to tf matrix
    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                         rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                         rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

    // convert rotation matrix to an orientation quaternion
    tf::Quaternion quat;
    tf_rot.getRotation(quat);
    return quat;
}

void ArucoLocalizer::saveInputFrame(const cv::Mat& frame) {
    static unsigned int counter = 0;
    saveFrame(frame, "aruco%03i_in.png", counter++);
}

void ArucoLocalizer::saveOutputFrame(const cv::Mat& frame) {
    static unsigned int counter = 0;
    saveFrame(frame, "aruco%03i_out.png", counter++);
}

void ArucoLocalizer::saveFrame(const cv::Mat& frame, std::string format_spec, unsigned int img_num) {
    // Create a filename
    std::stringstream ss;
    char buffer[100];
    sprintf(buffer, format_spec.c_str(), img_num);
    ss << debugImagePath_ << "/" << buffer;
    std::string filename = ss.str();
}

//load camera's matrix
void ArucoLocalizer::loadCameraParams(const std::string& yaml_file_path, sensor_msgs::CameraInfo& cam_info) {  
    YAML::Node config = YAML::LoadFile(yaml_file_path);  
    int i=0;
    if (!config["camera_name"]) {  
        ROS_ERROR("Camera name not found in YAML file.");  
        return;  
    }  
    cam_info.header.frame_id = config["camera_name"].as<std::string>();  
    if (!config["camera_matrix"]) {  
        ROS_ERROR("Camera matrix not found in YAML file.");  
        return;  
    }  

    // 加载相机矩阵 K 
   ROS_INFO("read K.");
    for (const auto& row : config["camera_matrix"]) {  
        for (const auto& element : row.second) {  
            cam_info.K[i] = element.as<double>();  
            i++;
        }  
    }  
    i=0;
    // std::cout << config["distortion_coefficients"] << std::endl;  

    // 加载畸变系数 D  
  ROS_INFO("read D.");
    for (const auto& row : config["distortion_coefficients"]) {  
        for (const auto& element : row.second) {  
            // std::cout << element.as<double>() << std::endl;  
            cam_info.D.push_back(element.as<double>());  
            i++;
        }  
    }  
    i=0;

    // 加载畸变系数 R
  ROS_INFO("read R.");
    for (const auto& row : config["rectification_matrix"]) {  
        for (const auto& element : row.second) {  
            cam_info.R[i] = element.as<double>();  
            i++;

        }  
    }  
    i=0;

    // 加载畸变系数 P
  ROS_INFO("read P.");
    for (const auto& row : config["projection_matrix"]) {  
        for (const auto& element : row.second) {  
            cam_info.P[i] = element.as<double>();  
            i++;
        }  
    }  
    i=0;

    // 设置其他参数  
    cam_info.height = config["image_height"].as<uint32_t>(0);  
    cam_info.width = config["image_width"].as<uint32_t>(0);  
    cam_info.distortion_model = config["distortion_model"].as<std::string>("");  
    // 这里可以继续加载其他相机参数，例如畸变系数、投影矩阵等...  
}

}

