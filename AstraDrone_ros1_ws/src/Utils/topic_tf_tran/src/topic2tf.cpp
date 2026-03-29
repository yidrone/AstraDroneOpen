#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>

#define GREEN_TEXT "\033[32m"  // 定义绿色文本
#define RESET_TEXT "\033[0m"   // 重置文本颜色

class TfPublisher
{
public:
    TfPublisher(ros::NodeHandle& nh_private)
    {
        // 通过私有句柄读取launch文件中的参数
        nh_private.param<std::string>("topic_name", topic_name_, std::string("/pose_topic"));
        nh_private.param<std::string>("parent_frame", parent_frame_, std::string("world"));
        nh_private.param<std::string>("child_frame", child_frame_, std::string("robot"));
        nh_private.param<bool>("print_flag", print_flag, false);

        std::cout << "print_flag: " << print_flag << std::endl;
        std::cout << "topic_name: " << topic_name_ << std::endl;

        first_message_printed = false;  // 初始化为false，只提示一次消息类型

        // 订阅不同的消息类型，并选择合适的回调函数
        if (ros::topic::waitForMessage<nav_msgs::Odometry>(topic_name_, nh_private, ros::Duration(1.0))) {
            sub_ = nh_.subscribe(topic_name_, 10, &TfPublisher::odometryCallback, this);
        } else if (ros::topic::waitForMessage<geometry_msgs::PoseStamped>(topic_name_, nh_private, ros::Duration(1.0))) {
            sub_ = nh_.subscribe(topic_name_, 10, &TfPublisher::poseCallback, this);
        } else if (ros::topic::waitForMessage<sensor_msgs::Imu>(topic_name_, nh_private, ros::Duration(1.0))) {
            sub_ = nh_.subscribe(topic_name_, 10, &TfPublisher::imuCallback, this);
        } else if (ros::topic::waitForMessage<geometry_msgs::TransformStamped>(topic_name_, nh_private, ros::Duration(1.0))) {
            sub_ = nh_.subscribe(topic_name_, 10, &TfPublisher::transformStampedCallback, this);
        } else if (ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(topic_name_, nh_private, ros::Duration(1.0))) {
            sub_ = nh_.subscribe(topic_name_, 10, &TfPublisher::poseWithCovarianceCallback, this);
        } else {
            ROS_ERROR("No valid messages found on topic %s", topic_name_.c_str());
        }
    }

private:
    bool print_flag;
    bool first_message_printed;  // 标志位，用于控制消息类型提示只打印一次
    ros::NodeHandle nh_;  // 公共句柄
    ros::Subscriber sub_;  // 订阅器
    tf::TransformBroadcaster tf_broadcaster_;  // TF广播器
    std::string topic_name_, parent_frame_, child_frame_;

    // 处理 geometry_msgs::PoseStamped 消息
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (print_flag) {ROS_INFO("Received PoseStamped message");}
        publishTransform(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                         msg->pose.orientation.x, msg->pose.orientation.y, 
                         msg->pose.orientation.z, msg->pose.orientation.w, "PoseStamped");
    }

    // 处理 nav_msgs::Odometry 消息
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if (print_flag) {ROS_INFO("Received Odometry message");}
        publishTransform(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                         msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, "Odometry");
    }

    // 处理 sensor_msgs::Imu 消息
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        if (print_flag) {ROS_INFO("Received Imu message");}
        publishTransform(0.0, 0.0, 0.0,
                         msg->orientation.x, msg->orientation.y, 
                         msg->orientation.z, msg->orientation.w, "Imu");
    }

    // 处理 geometry_msgs::TransformStamped 消息
    void transformStampedCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
        if (print_flag) {ROS_INFO("Received TransformStamped message");}
        publishTransform(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z,
                         msg->transform.rotation.x, msg->transform.rotation.y, 
                         msg->transform.rotation.z, msg->transform.rotation.w, "TransformStamped");
    }

    // 处理 geometry_msgs::PoseWithCovarianceStamped 消息
    void poseWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        if (print_flag) {ROS_INFO("Received PoseWithCovarianceStamped message");}
        publishTransform(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                         msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, "PoseWithCovarianceStamped");
    }

    // 发布TF并打印信息
    void publishTransform(double x, double y, double z, double qx, double qy, double qz, double qw, const std::string& msg_type)
    {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        tf::Quaternion q(qx, qy, qz, qw);
        transform.setRotation(q);

        // 发布TF
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_, child_frame_));

        // 如果print_flag为true，打印位置信息和四元数（使用绿色）
        if (print_flag) {
            ROS_INFO("Publishing transform from %s to %s", parent_frame_.c_str(), child_frame_.c_str());
            std::cout << GREEN_TEXT << "Position: x = " << x << ", y = " << y << ", z = " << z << RESET_TEXT << std::endl;
            std::cout << GREEN_TEXT << "Orientation (Quaternion): qx = " << qx << ", qy = " << qy << ", qz = " << qz << ", qw = " << qw << RESET_TEXT << std::endl;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_publisher_node");
    ros::NodeHandle nh_private("~");
    std::cout << "topic2tf" << std::endl;

    TfPublisher tf_publisher(nh_private);  // 创建类的实例
    ros::spin();  // 保持节点运行

    return 0;
}
