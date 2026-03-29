#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

class TFToOdometryNode
{
public:
    TFToOdometryNode(ros::NodeHandle& nh_private)
    {
        // Initialize ROS node
        ros::NodeHandle  nh_;

        // Initialize TF listener
        tf_listener_ = new tf::TransformListener();

        // 通过私有句柄读取launch文件中的参数
        nh_private.param<std::string>("base_link_frame_id_", base_link_frame_id_, std::string("aft_mapped"));
        nh_private.param<std::string>("map_frame_id_", map_frame_id_, std::string("camera_init"));
        nh_private.param<std::string>("odom_pub_topic", odom_pub_topic, std::string("/akm_car/odom"));
        nh_private.param<std::string>("pose_pub_topic", pose_pub_topic, std::string("/akm_car/pose"));
        
        // 新增参数：控制发布类型
        // 0: 只发布Odometry, 1: 只发布Pose, 2: 两者都发布
        nh_private.param<int>("publish_mode", publish_mode_, 0);

        // 根据发布模式初始化发布器
        if (publish_mode_ == 0 || publish_mode_ == 2) {
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_pub_topic, 10);
            ROS_INFO("Odometry publisher initialized on topic: %s", odom_pub_topic.c_str());
        }
        
        if (publish_mode_ == 1 || publish_mode_ == 2) {
            pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_pub_topic, 10);
            ROS_INFO("Pose publisher initialized on topic: %s", pose_pub_topic.c_str());
        }

        // 打印当前配置
        ROS_INFO("Publish mode: %d (0=Odom only, 1=Pose only, 2=Both)", publish_mode_);
        ROS_INFO("Map frame: %s, Base link frame: %s", map_frame_id_.c_str(), base_link_frame_id_.c_str());

        // Set the rate for publishing
        ros::Rate rate(10.0);

        while (ros::ok())
        {
            // Wait for the transform to be available
            try
            {
                tf::StampedTransform transform;
                tf_listener_->waitForTransform(map_frame_id_, base_link_frame_id_, ros::Time(0), ros::Duration(5.0));
                tf_listener_->lookupTransform(map_frame_id_, base_link_frame_id_, ros::Time(0), transform);

                ros::Time current_time = ros::Time::now();

                // 发布Odometry消息
                if (publish_mode_ == 0 || publish_mode_ == 2) {
                    nav_msgs::Odometry odom;
                    odom.header.stamp = current_time;
                    odom.header.frame_id = map_frame_id_;
                    odom.child_frame_id = base_link_frame_id_;

                    // Position
                    odom.pose.pose.position.x = transform.getOrigin().x();
                    odom.pose.pose.position.y = transform.getOrigin().y();
                    odom.pose.pose.position.z = transform.getOrigin().z();

                    // Orientation (quaternion)
                    tf::Quaternion q = transform.getRotation();
                    odom.pose.pose.orientation.x = q.x();
                    odom.pose.pose.orientation.y = q.y();
                    odom.pose.pose.orientation.z = q.z();
                    odom.pose.pose.orientation.w = q.w();

                    // Velocity (set to zero for this example)
                    odom.twist.twist.linear.x = 0.0;
                    odom.twist.twist.linear.y = 0.0;
                    odom.twist.twist.linear.z = 0.0;
                    odom.twist.twist.angular.x = 0.0;
                    odom.twist.twist.angular.y = 0.0;
                    odom.twist.twist.angular.z = 0.0;

                    odom_pub_.publish(odom);
                }

                // 发布Pose消息
                if (publish_mode_ == 1 || publish_mode_ == 2) {
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = current_time;
                    pose.header.frame_id = map_frame_id_;

                    // Position
                    pose.pose.position.x = transform.getOrigin().x();
                    pose.pose.position.y = transform.getOrigin().y();
                    pose.pose.position.z = transform.getOrigin().z();

                    // Orientation (quaternion)
                    tf::Quaternion q = transform.getRotation();
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();

                    pose_pub_.publish(pose);
                }
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            // Sleep for the specified rate
            rate.sleep();
        }
    }

    ~TFToOdometryNode()
    {
        delete tf_listener_;
    }

private:
    ros::NodeHandle nh_;
    tf::TransformListener* tf_listener_;
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    std::string base_link_frame_id_;
    std::string map_frame_id_;
    std::string odom_pub_topic;
    std::string pose_pub_topic;
    int publish_mode_;  // 0: Odom only, 1: Pose only, 2: Both
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_to_odometry_node");
    ros::NodeHandle nh_private("~");

    TFToOdometryNode tf_to_odometry_node(nh_private);
    ros::spin();
    return 0;
}