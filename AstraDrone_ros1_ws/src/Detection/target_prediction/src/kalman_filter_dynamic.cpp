/**
 *  @file kalman_filter_dynamic.cpp
 *  @author luli (luli.gptt@gmail.com)
 *  @brief LKF for target
 *  @version 0.1
 *  @date 11-03-2024
 */

#include <ros/ros.h>
#include <astra_custom_msgs/MarkerMeasurement.h>
#include <astra_custom_msgs/MarkerMeasurementArray.h>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>  
#include <tf2_ros/transform_listener.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  
#include <geometry_msgs/PointStamped.h> 
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
        ros::Subscriber sub;
        ros::Publisher pub;

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
        int first_iter; // variable to check the first iteration of the algorithm
            //  mavros/local_position/odom -- local_origin 
            // std::string map_frame = "local_origin";
            std::string map_frame = "map";
            // 自行修改
            // std::string camera_frame = "fcu";
            std::string camera_frame = "base_link";
            tf::TransformListener tfListener;

    public:
        // Public class attributes and methods
        Kalman(ros::NodeHandle ao_nh) : po_nh( ao_nh ), first_iter(0), dt(1), T(10,10), Q(10,10), R(5,5), 
                                        A(10,10), H(5,10), X(10,1), Z(5,1), S1(5,5), Kg(5,5)
        {
            // Publisher type target_prediction::States, it publishes in /predicted_states topic
            pub = po_nh.advertise<astra_custom_msgs::MarkerMeasurement>( "/predicted_measurements_pose", 10 ) ;
            
            // Subscriber to /states topic from target_prediction/States
            sub = po_nh.subscribe("/aruco/measurements", 10, &Kalman::predictionsDetectedCallback, this); 
            // Delta of time for the transition matrix
            this->dt = 0.1;
            this->first_iter = 0;

            // Posteriori estimate covariance matrix initialization
            this->T <<  2,0,0,0,0,0,0,0,0,0, 0,2,0,0,0,0,0,0,0,0, 
                        0,0,5,0,0,0,0,0,0,0, 0,0,0,5,0,0,0,0,0,0, 
                        0,0,0,0,5.625,0,0,0,0,0, 0,0,0,0,0,1e-3,0,0,0,0, 
                        0,0,0,0,0,0,1e-3,0,0,0, 0,0,0,0,0,0,0,1e-3,0,0, 
                        0,0,0,0,0,0,0,0,1e-3,0, 0,0,0,0,0,0,0,0,0,1e-3;

            /* Covariance matrix 
            * Xc  [2 0 0  0  0  0  0   0   0  0]
            * Yc  [0 2 0  0  0  0  0   0   0  0]
            * W   [0 0 5  0  0  0  0   0   0  0]
            * H   [0 0 0  5  0  0  0   0   0  0]
            * Th  [0 0 0  0  5.625  0  0   0   0  0]
            * Xc' [0 0 0  0  0  1e-3  0   0   0  0]
            * Yc' [0 0 0  0  0  0  1e-3   0   0  0]
            * W'  [0 0 0  0  0  0  0   1e-3   0  0]
            * H'  [0 0 0  0  0  0  0   0   1e-3  0]
            * Th' [0 0 0  0  0  0  0   0   0  1e-3] 
            */

            // Covariance of the process noise initialization
            this->Q = 1e-4*MatrixXd::Identity(10,10);
            // Covariance of the observation noise initialization
            this->R = 1e-2*MatrixXd::Identity(5,5);
            this->R(0, 0) = 1e-4;
            this->R(1, 1) = 1e-4;

            // State vector initialization
            this->X = MatrixXd::Zero(10,1); 
            // Innovation vectot initialization
            this->Z = MatrixXd::Zero(5,1); 
            // Covariance of the innovation initialization
            this->S1 = MatrixXd::Zero(5,5); 
            // Kalman gain initialization
            this->Kg = MatrixXd::Zero(5,5); 

            // State transition matrix initialization
            this->A << 1,0,0,0,0,dt,0,0,0,0, 0,1,0,0,0,0,dt,0,0,0, 
                        0,0,1,0,0,0,0,dt,0,0, 0,0,0,1,0,0,0,0,dt,0, 
                        0,0,0,0,1,0,0,0,0,dt, 0,0,0,0,0,1,0,0,0,0, 
                        0,0,0,0,0,0,1,0,0,0, 0,0,0,0,0,0,0,1,0,0, 
                        0,0,0,0,0,0,0,0,1,0, 0,0,0,0,0,0,0,0,0,1;
              
            /* Transition model 
            * Xc  [1 0 0  0  0  dt  0   0   0  0]
            * Yc  [0 1 0  0  0  0  dt   0   0  0]
            * W   [0 0 1  0  0  0  0   dt   0  0]
            * H   [0 0 0  1  0  0  0   0   dt  0]
            * Th  [0 0 0  0  1  0  0   0   0  dt]
            * Xc' [0 0 0  0  0  1  0   0   0  0]
            * Yc' [0 0 0  0  0  0  1   0   0  0]
            * W'  [0 0 0  0  0  0  0   1   0  0]
            * H'  [0 0 0  0  0  0  0   0   1  0]
            * Th' [0 0 0  0  0  0  0   0   0  1] 
            */

            // Observation model initialization
            this->H << 1,0,0,0,0,0,0,0,0,0, 
                        0,1,0,0,0,0,0,0,0,0, 
                        0,0,1,0,0,0,0,0,0,0, 
                        0,0,0,1,0,0,0,0,0,0, 
                        0,0,0,0,1,0,0,0,0,0;

            /* Transition model 
            * Xc  [1 0 0  0  0  0  0   0   0  0]
            * Yc  [0 1 0  0  0  0  0   0   0  0]
            * W   [0 0 1  0  0  0  0   0   0  0]
            * H   [0 0 0  1  0  0  0   0   0  0]
            * Th  [0 0 0  0  1  0  0   0   0  0]
            */
        }

        geometry_msgs::PointStamped camera_to_world (const geometry_msgs::PointStamped& p_c)
        {	
 
            geometry_msgs::PointStamped camera;
            geometry_msgs::PointStamped world;
            camera.header.frame_id = camera_frame;
            camera.point.x = p_c.point.x;
            camera.point.y = p_c.point.y;
            camera.point.z = p_c.point.z;

            if (!tfListener.waitForTransform(map_frame, camera_frame, ros::Time(0), ros::Duration(2)))
            {
                ROS_ERROR("Could not get transform from %s to %s after 1 second!", map_frame.c_str(), camera_frame.c_str());
            }
            tfListener.transformPoint(map_frame,camera,world);
            //cout<<"world"<<world.point.x << ","<<world.point.y << ","<<world.point.z << endl;
            world.header.frame_id = map_frame;
            return world;
        }

        // Subscriber callback
        void predictionsDetectedCallback(const astra_custom_msgs::MarkerMeasurement& msg)
        {	
            geometry_msgs::PointStamped world_measurement;
            geometry_msgs::PointStamped camera_measurement;
            camera_measurement.point.x = msg.position.x; 
            camera_measurement.point.y = msg.position.y; 
            camera_measurement.point.z = msg.position.z; 
            world_measurement = Kalman::camera_to_world(camera_measurement);
            std::cout<<world_measurement<<std::endl;


            // Creation of a States object to publish the info
            astra_custom_msgs::MarkerMeasurement predictions;

            MatrixXd measurement(5,1); // Vector to store the observations
            measurement = MatrixXd::Zero(5,1);

            // If it is the first iteration, initialize the states with the first observation
            if(this->first_iter==0)
            {
                this->X(0,0) = world_measurement.point.x; // State x
                this->X(1,0) = world_measurement.point.y; // State Y
                this->X(2,0) = world_measurement.point.z; // State z
                this->X(3,0) = msg.euler.x; // State pitch ?
                this->X(4,0) = msg.euler.z; // State yaw
                this->X(5,0) = 0; // State X'
                this->X(6,0) = 0; // State Y'
                this->X(7,0) = 0; // State z'
                this->X(8,0) = 0; // State '
                this->X(9,0) = 0; // State yaw'
                this->first_iter = 1;
            }
            std::cout<<"v of x ,y:"<<this->X(5,0)<<","<<this->X(6,0)<<std::endl;

            // Prediction step
            this->X = this->A*this->X; 
            this->T = ((this->A*this->T)*this->A.transpose())+this->Q;

            // Update step, assign the observations to the measurement vector 
            measurement(0,0) = world_measurement.point.x;
            measurement(1,0) = world_measurement.point.y;
            measurement(2,0) = world_measurement.point.z;
            measurement(3,0) = msg.euler.x;
            measurement(4,0) = msg.euler.z;

            // If there is a valid measurement (the detector found the coordinates)
            if((measurement(0) != 0)&&(measurement(1) != 0)&&(measurement(2) != 0)&&(measurement(3) != 0)) 
            {
                // Update step
                this->Z = measurement - this->H*this->X; 
                this->S1 = ((this->H*this->T)*this->H.transpose())+R; 
                this->Kg = (this->T*this->H.transpose())*this->S1.inverse(); 
                this->X = this->X + this->Kg*this->Z; 
                this->T = (MatrixXd::Identity(10,10)-(this->Kg*this->H))*this->T; 
            }

            // Assign the predictions to the publisher object
            predictions.position.x = this->X(0,0) + this->X(5,0)*1.5;
            predictions.position.y = this->X(1,0) + this->X(6,0)*1.5;
            predictions.position.z = this->X(2,0);
            predictions.euler.x = this->X(0,0); //origin position x for land
            predictions.euler.y = this->X(1,0); //origin position y 
            predictions.euler.z = this->X(4,0);

            // Uncomment these lines to print the results on console
            /* printf("The Centroid predicted with KF are (%f,%f)\n The Width is (%f)\n The Height is (%f)\n Theta is (%f)\n", 

            	predictions.Xc, predictions.Yc,
            	predictions.W, predictions.H,
            	predictions.Theta);
            */
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
