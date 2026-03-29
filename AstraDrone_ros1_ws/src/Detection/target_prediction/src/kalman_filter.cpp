/**
 *  @file kalman_filter_dynamic.cpp
 *  @author luli (luli.gptt@gmail.com)
 *  @brief LKF for target
 *  @version 0.1
 *  @date 11-03-2024
 */

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <astra_custom_msgs/MarkerMeasurement.h>
#include <astra_custom_msgs/MarkerMeasurementArray.h>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>  
#include <tf2_ros/transform_listener.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  
#include <geometry_msgs/PointStamped.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/PoseStamped.h>


# define M_PI 3.14159265358979323846  /* pi */

using namespace Eigen;

class Kalman
{
    private:
        // Private class attributes
        ros::NodeHandle po_nh;
        ros::Publisher pub;
        ros::Timer plan_timer_;

        MatrixXd T; // Posteriori estimate covariance matrix
        MatrixXd Q; // Covariance of the process noise
        MatrixXd R; // Covariance of the observation noise
        MatrixXd A; // State transition matrix
        MatrixXd H; // Observation model
        MatrixXd X; // State vector
        MatrixXd Z; // Innovation vectot
        MatrixXd S1; // Covariance of the innovation
        MatrixXd Kg; // Kalman gain
        
        float dt; // Delta of time
        float prediction_time = 1.5; // Delta of time
        float debug_hz = 40; // Delta of time
        int first_iter; // variable to check the first iteration of the algorithm
        // mavros/local_position/odom -- local_origin 

        // 设定在某个坐标系下的测量值
        // std::string map_frame = "local_origin";
        std::string map_frame = "map";
        // 自行修改
        // std::string aruco_frame = "fcu";
        std::string aruco_frame = "aruco";
        tf::TransformListener tfListener;

    public:
        // Public class attributes and methods
        // 定义矩阵大小
        Kalman(ros::NodeHandle ao_nh) : po_nh( ao_nh ), first_iter(0), dt(1), T(12,12), Q(12,12), R(6,6), 
                                        A(12,12), H(6,12), X(12,1), Z(6,1), S1(6,6), Kg(6,6)
        {
			po_nh.getParam("/prediction_time", prediction_time);
			po_nh.getParam("/debug_hz", debug_hz);

            std::cout<<"prediction_time = "<<prediction_time<<std::endl;
            std::cout<<"debug_hz = "<<debug_hz<<std::endl;
            
            // Publisher type target_prediction::States, it publishes in /predicted_states topic
            pub = po_nh.advertise<astra_custom_msgs::MarkerMeasurement>( "/aruco/predicted_pose", 10 ) ;

            // Subscriber to /states topic from target_prediction/States
            // sub = po_nh.subscribe("/aruco/measurements", 10, &Kalman::predictionsDetectedCallback, this); 
            plan_timer_ = po_nh.createTimer(ros::Duration(1.0 / debug_hz), &Kalman::timer_callback, this);

            // Delta of time for the transition matrix
            this->dt = 0.1;
            this->first_iter = 0;

            // Posteriori estimate covariance matrix initialization
            // 状态的初始协方差
            // this->T <<  2,0,0,0,0,0,0,0,0,0, 0,2,0,0,0,0,0,0,0,0, 
            //             0,0,5,0,0,0,0,0,0,0, 0,0,0,5,0,0,0,0,0,0, 
            //             0,0,0,0,5.625,0,0,0,0,0, 0,0,0,0,0,1e-3,0,0,0,0, 
            //             0,0,0,0,0,0,1e-3,0,0,0, 0,0,0,0,0,0,0,1e-3,0,0, 
            //             0,0,0,0,0,0,0,0,1e-3,0, 0,0,0,0,0,0,0,0,0,1e-3;

            this->T <<  1,0,0,0,0,0,0,0,0,0,0,0, 
                        0,1,0,0,0,0,0,0,0,0,0,0,  
                        0,0,1,0,0,0,0,0,0,0,0,0, 
                        0,0,0,0.5,0,0,0,0,0,0,0,0,  
                        0,0,0,0,0.5,0,0,0,0,0,0,0,  
                        0,0,0,0,0,0.5,0,0,0,0,0,0,  
                        0,0,0,0,0,0,1e-3,0,0,0,0,0,  
                        0,0,0,0,0,0,0,1e-3,0,0,0,0,  
                        0,0,0,0,0,0,0,0,1e-3,0,0,0,  
                        0,0,0,0,0,0,0,0,0,1e-3,0,0,
                        0,0,0,0,0,0,0,0,0,0,1e-3,0,
                        0,0,0,0,0,0,0,0,0,0,0,1e-3;

            // std::cout<<"T= \n"<<T<<std::endl;

            /* Covariance matrix 
            * Xc    [1 0 0  0  0  0  0   0   0  0  0  0]
            * Yc    [0 1 0  0  0  0  0   0   0  0  0  0]
            * Zc    [0 0 1  0  0  0  0   0   0  0  0  0]
            * Roll  [0 0 0  0.5  0  0  0   0   0  0  0  0]
            * Pitch [0 0 0  0  0.5  0  0   0   0  0  0  0]
            * Yaw   [0 0 0  0  0  0.5  0   0   0  0  0  0]
            * Xc'   [0 0 0  0  0  0  1e-3 0   0  0  0  0]
            * Yc'   [0 0 0  0  0  0  0   1e-3   0  0  0  0]
            * Zc'   [0 0 0  0  0  0  0   0   1e-3  0  0  0]
            * Roll' [0 0 0  0  0  0  0   0   0  1e-3  0  0]
            * Pitch'[0 0 0  0  0  0  0   0   0  0  1e-3  0] 
            * Yaw'  [0 0 0  0  0  0  0   0   0  0  0  1e-3] 
            */

            // Covariance of the process noise initialization
            // 过程噪声
            this->Q = 1e-4*MatrixXd::Identity(12,12);
            // Covariance of the observation noise initialization
            // 观测噪声
            this->R = 1e-2*MatrixXd::Identity(6,6);
            this->R(0, 0) = 1e-4;
            this->R(1, 1) = 1e-4;
            this->R(2, 2) = 1e-4;

            // State vector initialization
            // 状态矩阵初始化
            this->X = MatrixXd::Zero(12,1); 
            // Innovation vectot initialization
            // 观测矩阵初始化
            this->Z = MatrixXd::Zero(6,1); 
            // Covariance of the innovation initialization
            // 观测的协方差
            this->S1 = MatrixXd::Zero(6,6); 
            // Kalman gain initialization
            // 卡尔曼增益
            this->Kg = MatrixXd::Zero(6,6); 

            // State transition matrix initialization
            this->A <<  1,0,0,0,0,0,dt,0,0,0,0,0,
                        0,1,0,0,0,0,0,dt,0,0,0,0, 
                        0,0,1,0,0,0,0,0,dt,0,0,0, 
                        0,0,0,1,0,0,0,0,0,dt,0,0, 
                        0,0,0,0,1,0,0,0,0,0,dt,0, 
                        0,0,0,0,0,1,0,0,0,0,0,dt, 
                        0,0,0,0,0,0,1,0,0,0,0,0, 
                        0,0,0,0,0,0,0,1,0,0,0,0, 
                        0,0,0,0,0,0,0,0,1,0,0,0, 
                        0,0,0,0,0,0,0,0,0,1,0,0,
                        0,0,0,0,0,0,0,0,0,0,1,0,
                        0,0,0,0,0,0,0,0,0,0,0,1;
            
            // std::cout<<"A= \n"<<A<<std::endl;
              
            /* Transition model 
            * Xc   [1 0 0  0  0  0  dt   0   0  0  0  0]
            * Yc   [0 1 0  0  0  0  0   dt   0  0  0  0]
            * Zc   [0 0 1  0  0  0  0   0   dt  0  0  0]
            * Roll [0 0 0  1  0  0  0   0   0   dt  0  0]
            * Pitch[0 0 0  0  1  0  0   0   0   0  dt  0]
            * Yaw  [0 0 0  0  0  1  0   0   0   0  0  dt]
            * Xc'  [0 0 0  0  0  1  0   0   0  0]
            * Yc'  [0 0 0  0  0  0  1   0   0  0]
            * W'   [0 0 0  0  0  0  0   1   0  0]
            * H'   [0 0 0  0  0  0  0   0   1  0]
            * Th'  [0 0 0  0  0  0  0   0   0  1] 
            */

            // Observation model initialization
            // 预测矩阵
            this->H <<  1,0,0,0,0,0,0,0,0,0,0,0, 
                        0,1,0,0,0,0,0,0,0,0,0,0, 
                        0,0,1,0,0,0,0,0,0,0,0,0, 
                        0,0,0,1,0,0,0,0,0,0,0,0, 
                        0,0,0,0,1,0,0,0,0,0,0,0,
                        0,0,0,0,0,1,0,0,0,0,0,0;

            // std::cout<<"H= \n"<<H<<std::endl;

            /* Transition model 
            * Xc   [1 0 0  0  0  0  0   0   0  0  0  0]
            * Yc   [0 1 0  0  0  0  0   0   0  0  0  0]
            * Yc   [0 0 1  0  0  0  0   0   0  0  0  0]
            * Roll [0 0 0  1  0  0  0   0   0  0  0  0]
            * Pitch[0 0 0  0  1  0  0   0   0  0  0  0]
            * Yaw  [0 0 0  0  0  1  0   0   0  0  0  0]
            */
        }
        geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped& input_pose, const std::string& from_frame, const std::string& to_frame)
        {
            // Create a TF2 buffer and listener
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener(tf_buffer);

            // ros::Time time_begin = ros::Time::now();

            // Wait for the transform to be available
            // ros::Rate rate(30.0);
            while (!tf_buffer.canTransform(to_frame, from_frame, ros::Time(0)))
            {
                // ROS_WARN("Waiting for transform from %s to %s", from_frame.c_str(), to_frame.c_str());
                // rate.sleep();
            }

            // ros::Time time_end = ros::Time::now();
            // ros::Duration duration = time_end - time_begin;
            // ROS_INFO("\033[34m Slept for %lf secs \033[0m", duration.toSec());

            try
            {
                // Get the transform from 'from_frame' to 'to_frame'
                geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(to_frame, from_frame, ros::Time(0));

                // Transform the pose
                geometry_msgs::PoseStamped transformed_pose;
                tf2::doTransform(input_pose, transformed_pose, transform);

                return transformed_pose;
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN("Transform error: %s", ex.what());
                return geometry_msgs::PoseStamped();
            }
        }

        void poseToEuler(const geometry_msgs::PoseStamped& pose, tf2::Vector3& position, tf2::Vector3& euler_angles)
        {
            position.setX(pose.pose.position.x);
            position.setY(pose.pose.position.y);
            position.setZ(pose.pose.position.z);

            tf2::Quaternion q(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            );

            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            euler_angles.setX(roll);
            euler_angles.setY(pitch);
            euler_angles.setZ(yaw);
        }

        astra_custom_msgs::MarkerMeasurement aruco_to_world ()
        {	
            geometry_msgs::PoseStamped  aruco;
            astra_custom_msgs::MarkerMeasurement world;
            tf2::Vector3 position, euler_angles;

            // 实际上是aruco的坐标系原点
            aruco.header.frame_id = aruco_frame;
            aruco.pose.position.x = 0.0;
            aruco.pose.position.y = 0.0;
            aruco.pose.position.z = 0.0;
            aruco.pose.orientation.x = 0.0;
            aruco.pose.orientation.y = 0.0;
            aruco.pose.orientation.z = 0.0;
            aruco.pose.orientation.w = 1.0;

            // 转换坐标及位姿
            geometry_msgs::PoseStamped transformed_pose = transformPose(aruco, aruco_frame, map_frame);
            // 获得位置和欧拉角
            poseToEuler(transformed_pose, position, euler_angles);

            // 填充结果
            world.position = transformed_pose.pose.position;
            world.euler.x = euler_angles[0];
            world.euler.y = euler_angles[1];
            world.euler.z = euler_angles[2];
            world.orientation = transformed_pose.pose.orientation;

            return world;
        }

        // Subscriber callback
        void timer_callback(const ros::TimerEvent& event)
        {
            // wait aruco detect
            if (!tfListener.waitForTransform(map_frame, aruco_frame, ros::Time(0), ros::Duration(2)))
                return;

            astra_custom_msgs::MarkerMeasurement world_measurement;
            world_measurement = Kalman::aruco_to_world();

            // std::cout<<"aruco_under_map = \n"<<world_measurement<<std::endl;

            // Creation of a States object to publish the info
            astra_custom_msgs::MarkerMeasurement predictions;

            MatrixXd measurement(6,1); // Vector to store the observations
            measurement = MatrixXd::Zero(6,1);


            // If it is the first iteration, initialize the states with the first observation
            if(this->first_iter==0)
            {
                this->X(0,0) = world_measurement.position.x; // State x
                this->X(1,0) = world_measurement.position.y; // State Y
                this->X(2,0) = world_measurement.position.z; // State z
                this->X(3,0) = world_measurement.euler.x; // State roll 
                this->X(4,0) = world_measurement.euler.y; // State pitch
                this->X(5,0) = world_measurement.euler.z; // State yaw
                this->X(6,0) = 0; // 
                this->X(7,0) = 0; // 
                this->X(8,0) = 0; // 
                this->X(9,0) = 0; // 
                this->X(10,0) = 0; // 
                this->X(11,0) = 0; // 
                this->first_iter = 1; //flag
            }
            // std::cout<<"v of x ,y:"<<this->X(5,0)<<","<<this->X(6,0)<<std::endl;

            // Prediction step
            this->X = this->A*this->X; 
            this->T = ((this->A*this->T)*this->A.transpose())+this->Q;

            // Update step, assign the observations to the measurement vector 
            measurement(0,0) = world_measurement.position.x;
            measurement(1,0) = world_measurement.position.y;
            measurement(2,0) = world_measurement.position.z;
            measurement(3,0) = world_measurement.euler.x;
            measurement(4,0) = world_measurement.euler.y;
            measurement(5,0) = world_measurement.euler.z;

            // If there is a valid measurement (the detector found the coordinates)
            if((measurement(0) != 0)&&(measurement(1) != 0)&&(measurement(2) != 0)&&(measurement(3) != 0)) 
            {
                // Update step
                this->Z = measurement - this->H*this->X; 
                this->S1 = ((this->H*this->T)*this->H.transpose())+R; 
                this->Kg = (this->T*this->H.transpose())*this->S1.inverse(); 
                this->X = this->X + this->Kg*this->Z; 
                this->T = (MatrixXd::Identity(12,12)-(this->Kg*this->H))*this->T; 
            }

            // Assign the predictions to the publisher object
            predictions.position.x = this->X(0,0) + this->X(6,0)*prediction_time;//x
            predictions.position.y = this->X(1,0) + this->X(7,0)*prediction_time;//y
            predictions.position.z = this->X(2,0) + this->X(8,0)*0.0;//z
            

            predictions.euler.x = this->X(3,0); //roll
            predictions.euler.y = this->X(4,0); //pitch
            predictions.euler.z = this->X(5,0); //yaw

            predictions.orientation = world_measurement.orientation;//orientation -> no prediction

            predictions.linear.x = this->X(6,0); //x'
            predictions.linear.y = this->X(7,0); //y'
            predictions.linear.z = this->X(8,0); //z'

            pub.publish(predictions);
        }
};


int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "KF_predictor"); 
    ros::NodeHandle n;

    Kalman kf(n);
    ros::spin();

    return 0;
}   
