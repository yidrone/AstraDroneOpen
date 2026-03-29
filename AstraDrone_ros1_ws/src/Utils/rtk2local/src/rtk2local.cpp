#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/RCIn.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>

class RCTransformNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber rc_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber flu_cmd_sub_;
    ros::Subscriber flu_raw_sub_; 
    ros::Publisher pose_pub_;
    ros::Publisher cmd_pub_;
    ros::Publisher raw_cmd_pub_;
    ros::Subscriber odom_sub_; 

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    geometry_msgs::PoseStamped origin_pose_;
    double origin_yaw_ = 0.0;
    bool triggered_ = false;
    bool origin_set_ = false;
    // 新增：触发条件相关变量
    bool use_rc_trigger_ = false;
    bool use_timer_trigger_ = false;
    bool first_flag = true;
    double max_position_variance_ = 0.1; // 位置方差阈值 (m^2)
    double min_trigger_time_ = 60.0;     // 最小触发时间 (秒)
    ros::Time node_start_time_;          // 节点启动时间
    
    // 新增：存储当前定位方差
    double position_variance_ = std::numeric_limits<double>::max();
    double yaw_variance_ = std::numeric_limits<double>::max();
public:
    RCTransformNode() : node_start_time_(ros::Time::now()) {
        // 读取参数
        ros::NodeHandle private_nh("~");
        private_nh.param("use_rc_trigger", use_rc_trigger_, false);
        private_nh.param("use_timer_trigger", use_timer_trigger_, false);
        private_nh.param("max_position_variance", max_position_variance_, 0.1);
        private_nh.param("min_trigger_time", min_trigger_time_, 60.0);
        
        rc_sub_ = nh_.subscribe("/mavros/rc/in", 10, &RCTransformNode::rcCallback, this);
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &RCTransformNode::poseCallback, this);
        flu_cmd_sub_ = nh_.subscribe("/mavros/setpoint_position/local_flu", 10, &RCTransformNode::fluCmdCallback, this);
        flu_raw_sub_ = nh_.subscribe("/mavros/setpoint_raw/local_flu", 10, &RCTransformNode::fluRawCallback, this);
        odom_sub_ = nh_.subscribe("/mavros/odometry/in", 10, &RCTransformNode::odomCallback, this); // 新增订阅

        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose_flu", 10);
        cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        raw_cmd_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        publishStaticTransform();
        ROS_INFO("RCTransformNode initialized. RC trigger: %s, Timer trigger: %s, Timer Max: %lf s", 
                use_rc_trigger_ ? "enabled" : "disabled",
                use_timer_trigger_ ? "enabled" : "disabled",
                min_trigger_time_);
    }

    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
        // 遥控器触发
        if (!triggered_ && msg->channels.size() > 9 && msg->channels[9] > 1500 && use_rc_trigger_) {
            ROS_INFO("RC Channel 10 > 1500, ready to record origin pose on next pose message.");
            triggered_ = true;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if(first_flag){
            node_start_time_ = ros::Time::now();
            first_flag = false;
            return;
        }
        // 提取位置方差（假设在协方差矩阵的前三个元素）
        position_variance_ = std::max({msg->pose.covariance[0], 
                                    msg->pose.covariance[7], 
                                    msg->pose.covariance[14]});

        // 检查是否满足方差触发条件
        if (!triggered_ && use_timer_trigger_) {
            ros::Duration elapsed = ros::Time::now() - node_start_time_;
            double elapsed_sec = elapsed.toSec();
            
            bool variance_ok = (position_variance_ <= max_position_variance_);
            bool time_ok = ((elapsed_sec >= min_trigger_time_));
            // if (variance_ok && time_ok) {
            if (time_ok) {
                ROS_INFO("Auto-triggered due to low variance and elapsed time %.1fs", 
                         elapsed_sec);
                triggered_ = true;
            }
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {  
        if (!triggered_)
            return;

        if (!origin_set_) {
            origin_pose_ = *msg;

            tf2::Quaternion q_orig;
            tf2::fromMsg(origin_pose_.pose.orientation, q_orig);
            tf2::Matrix3x3 m(q_orig);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            origin_yaw_ = yaw;

            origin_set_ = true;
            ROS_INFO("Origin pose recorded with yaw: %.3f", origin_yaw_);
            return;
        }

        // 当前姿态 yaw
        tf2::Quaternion q_curr;
        tf2::fromMsg(msg->pose.orientation, q_curr);
        tf2::Matrix3x3 m_curr(q_curr);
        double roll, pitch, yaw;
        m_curr.getRPY(roll, pitch, yaw);
        double relative_yaw = yaw - origin_yaw_;

        // 坐标转换
        double dx = msg->pose.position.x - origin_pose_.pose.position.x;
        double dy = msg->pose.position.y - origin_pose_.pose.position.y;

        double x_new = cos(-origin_yaw_) * dx - sin(-origin_yaw_) * dy;
        double y_new = sin(-origin_yaw_) * dx + cos(-origin_yaw_) * dy;
        double z_new = msg->pose.position.z - origin_pose_.pose.position.z;

        tf2::Quaternion q_new;
        q_new.setRPY(roll, pitch, relative_yaw);

        geometry_msgs::PoseStamped pose_out;
        pose_out.header.stamp = msg->header.stamp;
        pose_out.header.frame_id = "map_flu";
        pose_out.pose.position.x = x_new;
        pose_out.pose.position.y = y_new;
        pose_out.pose.position.z = z_new;
        pose_out.pose.orientation = tf2::toMsg(q_new);
        pose_pub_.publish(pose_out);

        // tf map_flu -> body_flu
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = "map_flu";
        tf.child_frame_id = "body_flu";
        tf.transform.translation.x = x_new;
        tf.transform.translation.y = y_new;
        tf.transform.translation.z = z_new;
        tf.transform.rotation = tf2::toMsg(q_new);
        tf_broadcaster_.sendTransform(tf);
    }

    void fluRawCallback(const mavros_msgs::PositionTarget::ConstPtr& msg) {
        if (!origin_set_) return;

        mavros_msgs::PositionTarget target_out = *msg;
        target_out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 设置目标坐标系为ENU

        // 位置转换 (FLU -> ENU)
        if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_PX) &&
            !(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_PY)) {
            double x_flu = msg->position.x;
            double y_flu = msg->position.y;
            target_out.position.x = cos(origin_yaw_) * x_flu - sin(origin_yaw_) * y_flu + origin_pose_.pose.position.x;
            target_out.position.y = sin(origin_yaw_) * x_flu + cos(origin_yaw_) * y_flu + origin_pose_.pose.position.y;
        }
        if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_PZ)) {
            target_out.position.z = msg->position.z + origin_pose_.pose.position.z;
        }

        // 速度转换 (FLU -> ENU)
        if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_VX) &&
            !(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_VY)) {
            double vx_flu = msg->velocity.x;
            double vy_flu = msg->velocity.y;
            target_out.velocity.x = cos(origin_yaw_) * vx_flu - sin(origin_yaw_) * vy_flu;
            target_out.velocity.y = sin(origin_yaw_) * vx_flu + cos(origin_yaw_) * vy_flu;
        }

        // 加速度转换 (FLU -> ENU)
        if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_AFX) &&
            !(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_AFY)) {
            double afx_flu = msg->acceleration_or_force.x;
            double afy_flu = msg->acceleration_or_force.y;
            target_out.acceleration_or_force.x = cos(origin_yaw_) * afx_flu - sin(origin_yaw_) * afy_flu;
            target_out.acceleration_or_force.y = sin(origin_yaw_) * afx_flu + cos(origin_yaw_) * afy_flu;
        }

        // 偏航角转换 (相对FLU -> 绝对ENU)
        if (!(msg->type_mask & mavros_msgs::PositionTarget::IGNORE_YAW)) {
            target_out.yaw = msg->yaw + origin_yaw_;
        }

        raw_cmd_pub_.publish(target_out);
    }

    void fluCmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!origin_set_) return;

        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double z = msg->pose.position.z;

        // 旋转回原坐标系
        double x_map = cos(origin_yaw_) * x - sin(origin_yaw_) * y + origin_pose_.pose.position.x;
        double y_map = sin(origin_yaw_) * x + cos(origin_yaw_) * y + origin_pose_.pose.position.y;
        double z_map = z + origin_pose_.pose.position.z;

        // yaw 处理
        tf2::Quaternion q_flu;
        tf2::fromMsg(msg->pose.orientation, q_flu);
        double roll, pitch, yaw_flu;
        tf2::Matrix3x3(q_flu).getRPY(roll, pitch, yaw_flu);
        double yaw_map = yaw_flu + origin_yaw_;
        tf2::Quaternion q_map;
        q_map.setRPY(roll, pitch, yaw_map);

        geometry_msgs::PoseStamped cmd_out;
        cmd_out.header.stamp = msg->header.stamp;
        cmd_out.header.frame_id = "map";
        cmd_out.pose.position.x = x_map;
        cmd_out.pose.position.y = y_map;
        cmd_out.pose.position.z = z_map;
        cmd_out.pose.orientation = tf2::toMsg(q_map);
        cmd_pub_.publish(cmd_out);
    }

    void publishStaticTransform() {
        geometry_msgs::TransformStamped static_tf;
        static_tf.header.stamp = ros::Time::now();
        static_tf.header.frame_id = "body_flu";
        static_tf.child_frame_id = "camera_flu";
        static_tf.transform.translation.x = 0.0;
        static_tf.transform.translation.y = 0.0;
        static_tf.transform.translation.z = 0.0;
        static_tf.transform.rotation.x = 0.0;
        static_tf.transform.rotation.y = 0.0;
        static_tf.transform.rotation.z = 0.0;
        static_tf.transform.rotation.w = 1.0;

        static_tf_broadcaster_.sendTransform(static_tf);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rc_pose_transform_node");
    RCTransformNode node;
    ros::spin();
    return 0;
}