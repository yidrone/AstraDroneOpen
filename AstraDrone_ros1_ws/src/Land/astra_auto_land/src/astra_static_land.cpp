// precision_landing_node.cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <astra_auto_land/PrecisionLandingActivate.h>
#include <cmath>
#include <chrono>
#include <deque>

template<typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

enum class PrecisionLandingState {
    Start,
    HorizontalApproach,
    DescendAboveTarget,
    Search,
    Fallback,
    Done
};

struct TargetPose {
    double absolute_x;
    double absolute_y;
    double absolute_z;
    double absolute_yaw; 
    bool is_absolute_position_valid;
    bool is_valid;
    std::chrono::steady_clock::time_point timestamp;
};

class PrecisionLandingNode {
private:
    ros::NodeHandle node_handle_, private_node_handle_;
    ros::Publisher setpoint_publisher_;
    ros::Publisher velocity_publisher_;
    ros::Subscriber state_subscriber_, extended_state_subscriber_, local_position_subscriber_, global_position_subscriber_, target_pose_subscriber_;
    ros::ServiceClient arming_client_, set_mode_client_, land_client_, disarm_client_;
    ros::ServiceServer activate_service_;
    ros::Timer control_timer_;
    ros::Subscriber tag_measurement_subscriber_;
    ros::Time last_measurement_time_;

    // 添加 fallback 状态触底检测变量
    ros::Time fallback_check_start_time_;
    geometry_msgs::Point fallback_initial_position_;
    bool fallback_check_started_ = false;

    PrecisionLandingState current_state_;
    std::chrono::steady_clock::time_point state_start_time_, point_reached_time_, target_acquired_time_;
    mavros_msgs::State mavros_state_;
    mavros_msgs::ExtendedState extended_state_;
    geometry_msgs::PoseStamped current_pose_;
    sensor_msgs::NavSatFix global_position_;
    TargetPose target_pose_;
    bool is_target_pose_updated_;

    double search_altitude_, horizontal_acceptance_radius_;
    double target_timeout_, search_timeout_;
    double max_horizontal_velocity_, max_horizontal_acceleration_;
    double descent_z_position_, ground_check_interval_, ground_check_duration_;
    double two_stage_z_threshold_, two_stage_error_percent_, three_stage_z_threshold_;
    double motor_lock_z_threshold_;
    double max_descent_speed_;
    double max_yaw_speed_;  // 最大Yaw角速度（弧度/秒）
    double yaw_acceptance_threshold_;  // Yaw角接受阈值（弧度）

    bool is_activated_ = false;
    geometry_msgs::Point previous_setpoint_, previous_previous_setpoint_;
    std::chrono::steady_clock::time_point last_slewrate_time_;
    ros::Time ground_check_start_time_;
    double last_altitude_ = 0.0;
    bool is_ground_check_active_ = false;
    float error_percent_ = 1.0;
    float descent_percent_ = 1.0;
    float recorded_z_position_ = 0.0;
    bool should_record_z_position_ = false;
    bool need_ground_check_ = true;
    double z_difference;
    double current_target_yaw_;  // 当前目标Yaw角（逐步调整）
    bool is_current_target_yaw_initialized_;  //  目标Yaw是否已初始化

    // 添加 fallback 状态触底检测变量
    std::deque<std::pair<ros::Time, double>> fallback_z_history_;

public:
    PrecisionLandingNode() : 
        node_handle_(), 
        private_node_handle_("~"), 
        current_state_(PrecisionLandingState::Start), 
        is_target_pose_updated_(false),
        is_current_target_yaw_initialized_(false)  // 初始化Yaw角状态
    {
        initializeParameters();
        initializeROS();
        initializeState();
        ROS_INFO("Precision Landing Node initialized. Call the /astra_auto_land/activate service to start.");
    }

private:
    void initializeParameters() {
        // 读取所有参数
        private_node_handle_.param("search_altitude", search_altitude_, 8.0);
        private_node_handle_.param("horizontal_acceptance_radius", horizontal_acceptance_radius_, 0.3);
        private_node_handle_.param("target_timeout", target_timeout_, 2.0);
        private_node_handle_.param("search_timeout", search_timeout_, 30.0);
        private_node_handle_.param("base_descent_z", descent_z_position_, 0.3);
        private_node_handle_.param("ground_check_interval", ground_check_interval_, 0.05);
        private_node_handle_.param("ground_check_duration", ground_check_duration_, 10.0);
        private_node_handle_.param("lock_motor_z_threshold", motor_lock_z_threshold_, 0.1);
        private_node_handle_.param("z_two_stage", two_stage_z_threshold_, 1.5);
        private_node_handle_.param("z_three_stage", three_stage_z_threshold_, 30.0);
        private_node_handle_.param("need_ground_check", need_ground_check_, true);
        private_node_handle_.param("max_descent_z", max_descent_speed_, 0.8);
        
        // 新增Yaw角控制参数
        private_node_handle_.param("max_yaw_speed", max_yaw_speed_, 0.5);  // 默认~28.6度/秒
        private_node_handle_.param("yaw_acceptance_threshold", yaw_acceptance_threshold_, 0.087);  // 默认5度
        
        // 打印参数值
        ROS_INFO("Loaded parameters:");
        ROS_INFO("  search_altitude: %.2f m", search_altitude_);
        ROS_INFO("  horizontal_acceptance_radius: %.2f m", horizontal_acceptance_radius_);
        ROS_INFO("  target_timeout: %.2f s", target_timeout_);
        ROS_INFO("  search_timeout: %.2f s", search_timeout_);
        ROS_INFO("  base_descent_z: %.2f m/s", descent_z_position_);
        ROS_INFO("  max_descent_z: %.2f m/s", max_descent_speed_);
        ROS_INFO("  lock_motor_z_threshold(z_one_stage): %.2f m", motor_lock_z_threshold_);
        ROS_INFO("  z_two_stage: %.2f m", two_stage_z_threshold_);
        ROS_INFO("  z_three_stage: %.2f m", three_stage_z_threshold_);
        ROS_INFO("  ground_check_interval: %.2f s", ground_check_interval_);
        ROS_INFO("  ground_check_duration: %.2f s", ground_check_duration_);
        ROS_INFO("  max_yaw_speed: %.2f rad/s (%.1f deg/s)", max_yaw_speed_, max_yaw_speed_ * 180.0 / M_PI);
        ROS_INFO("  yaw_acceptance_threshold: %.3f rad (%.1f deg)", 
                yaw_acceptance_threshold_, yaw_acceptance_threshold_ * 180.0 / M_PI);
    }

    void initializeROS() {
        setpoint_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        velocity_publisher_ = node_handle_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
        state_subscriber_ = node_handle_.subscribe("/mavros/state", 10, &PrecisionLandingNode::stateCallback, this);
        extended_state_subscriber_ = node_handle_.subscribe("/mavros/extended_state", 10, &PrecisionLandingNode::extendedStateCallback, this);
        local_position_subscriber_ = node_handle_.subscribe("/mavros/local_position/pose", 10, &PrecisionLandingNode::localPoseCallback, this);
        global_position_subscriber_ = node_handle_.subscribe("/mavros/global_position/global", 10, &PrecisionLandingNode::globalPoseCallback, this);
        target_pose_subscriber_ = node_handle_.subscribe("/EKF/PoseStamped/estimate", 10, &PrecisionLandingNode::targetPoseCallback, this);

        arming_client_ = node_handle_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = node_handle_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        land_client_ = node_handle_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
        disarm_client_ = node_handle_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

        activate_service_ = node_handle_.advertiseService("/astra_auto_land/activate", &PrecisionLandingNode::activateServiceCallback, this);
        control_timer_ = node_handle_.createTimer(ros::Duration(0.1), &PrecisionLandingNode::controlLoop, this);  // 10Hz控制频率
        tag_measurement_subscriber_ = node_handle_.subscribe("/apriltag/Point/pixel", 10, &PrecisionLandingNode::tagMeasurementCallback, this);
        last_measurement_time_ = ros::Time(0);
    }

    void initializeState() {
        current_state_ = PrecisionLandingState::Start;
        state_start_time_ = std::chrono::steady_clock::now();
        target_pose_.is_valid = false;
        target_pose_.is_absolute_position_valid = false;
        is_current_target_yaw_initialized_ = false;  // 确保Yaw角未初始化
        ROS_DEBUG("Initialized state: Start");
    }

    // 添加丢失目标检测函数
    void tagMeasurementCallback(const geometry_msgs::Point::ConstPtr& msg) {
        last_measurement_time_ = ros::Time::now();
    }

    // 从四元数获取Yaw角
    double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quat, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    // 优化Yaw角差值（确保选择最短旋转路径）
    double normalizeYawDifference(double diff) {
        // 将角度差标准化到[-π, π]范围内
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        return diff;
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) { 
        mavros_state_ = *msg; 
    }
    
    void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg) { 
        extended_state_ = *msg; 
    }
    
    void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) { 
        current_pose_ = *msg; 
    }
    
    void globalPoseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) { 
        global_position_ = *msg; 
    }

// 目标位姿回调：更新目标的三维位置和偏航角
    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 【新增】只要收到位姿，就证明看到了目标，更新时间戳！
        last_measurement_time_ = ros::Time::now(); 

        target_pose_.absolute_x = msg->pose.position.x;
        target_pose_.absolute_y = msg->pose.position.y;
        target_pose_.absolute_z = msg->pose.position.z;
        double yaw_from_quat = getYawFromQuaternion(msg->pose.orientation);

        target_pose_.absolute_yaw = yaw_from_quat;
        is_current_target_yaw_initialized_ = true;
        target_pose_.is_absolute_position_valid = true;
        target_pose_.is_valid = true;
        target_pose_.timestamp = std::chrono::steady_clock::now();
        is_target_pose_updated_ = true;
    }


    bool activateServiceCallback(astra_auto_land::PrecisionLandingActivate::Request &, astra_auto_land::PrecisionLandingActivate::Response &response) {
        if (is_activated_) {
            response.success = false;
            response.message = "Already active";
            return true;
        }
        // if (extended_state_.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR) {
        //     response.success = false;
        //     response.message = "Drone not in air";
        //     return true;
        // }
        // if ((ros::Time::now() - last_measurement_time_).toSec() > target_timeout_) {
        //     response.success = false;
        //     response.message = "No target!";
        //     return true;
        // }
        is_activated_ = true;
        switchToState(PrecisionLandingState::HorizontalApproach);
        response.success = true;
        response.message = "Activated";
        ROS_INFO("Precision landing activated");
        return true;
    }

    bool isYawAligned() {
        if (!is_current_target_yaw_initialized_) {
            ROS_WARN_THROTTLE(1.0, "Target Yaw not initialized yet");
            return false;
        }

        double current_yaw = getYawFromQuaternion(current_pose_.pose.orientation);
        double yaw_diff = normalizeYawDifference(target_pose_.absolute_yaw - current_yaw);

        // 使用缩放后的容差判断
        double scaled_yaw_threshold = yaw_acceptance_threshold_ * error_percent_;
        bool aligned = std::abs(yaw_diff) < scaled_yaw_threshold;
        
        ROS_INFO_THROTTLE(1.0, "Yaw align check: |%.1f degree| < %.1f degree ? %s",
                        yaw_diff * 180.0 / M_PI,
                        scaled_yaw_threshold * 180.0 / M_PI,
                        aligned ? "YES" : "NO");
        return aligned;
    }

    double calculateDescentPercent(double z_diff) {
        if (z_diff >= three_stage_z_threshold_) {
            return 30.0;
        } else if (z_diff >= two_stage_z_threshold_) {
            double ratio = (z_diff - two_stage_z_threshold_) / (three_stage_z_threshold_ - two_stage_z_threshold_);
            return 1.0 + ratio * (30.0 - 1.0);
        } else if (z_diff >= motor_lock_z_threshold_) {
            return 1.0;
        }
        return 1.0;  // motor_lock_z_threshold_
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!is_activated_) return;

        auto current_time = std::chrono::steady_clock::now();

        // 检查目标是否丢失
        if ((ros::Time::now() - last_measurement_time_).toSec() > target_timeout_) {
            if (current_state_ == PrecisionLandingState::HorizontalApproach || current_state_ == PrecisionLandingState::DescendAboveTarget) {
                ROS_WARN("Target lost, switching to Search state");
                switchToState(PrecisionLandingState::Search);
            }
        }
        else
        {
            if (current_state_ == PrecisionLandingState::Fallback) {
                ROS_WARN("Target get, switching to HorizontalApproach state");
                switchToState(PrecisionLandingState::HorizontalApproach);
            }
        }


        z_difference = fabs(current_pose_.pose.position.z - target_pose_.absolute_z);
        error_percent_ = calculateDescentPercent(z_difference);

        // std::cout<<"extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND: "<<extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND<<std::endl;
        // std::cout<<"current_pose_.pose.position.z < motor_lock_z_threshold_ "<<current_pose_.pose.position.z < motor_lock_z_threshold_<<std::endl;
        // // 检查是否已经着陆
        // if (extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND && current_pose_.pose.position.z < motor_lock_z_threshold_ ) {
        //     switchToState(PrecisionLandingState::Done);
        //     return;
        // }

        switch (current_state_) {
            case PrecisionLandingState::HorizontalApproach: 
                runStateHorizontalApproach(); 
                break;
            case PrecisionLandingState::DescendAboveTarget: 
                runStateDescendAboveTarget(); 
                break;
            case PrecisionLandingState::Search:
                runStateSearch();
                break;
            case PrecisionLandingState::Fallback: 
                runStateFallback();
                break;
            case PrecisionLandingState::Done: 
                is_activated_ = false; 
                ROS_INFO("Precision landing completed");
                break;
            default: 
                ROS_WARN_THROTTLE(1.0, "Unknown state");
                break;
        }
        is_target_pose_updated_ = false;
    }

    void switchToState(PrecisionLandingState new_state) {
        const char* state_names[] = {
            "Start", "HorizontalApproach", "DescendAboveTarget", 
            "Search", "Fallback", "Done"
        };
        
        ROS_INFO("State transition: %s -> %s", 
                state_names[static_cast<int>(current_state_)],
                state_names[static_cast<int>(new_state)]);
        
        current_state_ = new_state;
        state_start_time_ = std::chrono::steady_clock::now();
        is_ground_check_active_ = false;
        
        // 状态特定的初始化
        switch (new_state) {
            case PrecisionLandingState::DescendAboveTarget:
                ROS_INFO("Starting descent above target");
                break;
            case PrecisionLandingState::HorizontalApproach:
                should_record_z_position_ = false;
                ROS_INFO("Starting horizontal approach");
                break;
            case PrecisionLandingState::Search:
                ROS_WARN("Starting search attempt at %.1f m", search_altitude_);
                break;
            default:
                break;
        }
    }

    // fallback 为查询过去10秒内Z变化
    void runStateFallback() {
        double current_z = current_pose_.pose.position.z;
        geometry_msgs::Point current_pos = current_pose_.pose.position;

        publishPositionSetpoint(current_pos.x, current_pos.y, current_z - 0.3);
        ROS_WARN_THROTTLE(1.0, "Fallback descent in progress");

        ros::Time now = ros::Time::now();
        fallback_z_history_.emplace_back(now, current_z);

        // 保留最近10秒内的z历史
        while (!fallback_z_history_.empty() && (now - fallback_z_history_.front().first).toSec() > 10.0) {
            fallback_z_history_.pop_front();
        }

        double z_max = -1e6, z_min = 1e6;
        for (const auto& entry : fallback_z_history_) {
            z_max = std::max(z_max, entry.second);
            z_min = std::min(z_min, entry.second);
        }

        double z_change = z_max - z_min;
        ROS_INFO_THROTTLE(1.0, "Fallback check: Z change=%.3f, current Z=%.3f", z_change, current_z);

        if (z_change < 0.05 && current_z < 5.0) {
            ROS_INFO("Fallback check passed. Locking motors.");
            lockMotors();
            switchToState(PrecisionLandingState::Done);
        }
    }

    void runStateSearch() {
        double current_z = current_pose_.pose.position.z;

        // 等待目标重新出现
        double search_elapsed = (std::chrono::steady_clock::now() - state_start_time_).count() / 1e9;
        if ((ros::Time::now() - last_measurement_time_).toSec() <= target_timeout_) {
            ROS_INFO("Target re-acquired during search");
            switchToState(PrecisionLandingState::HorizontalApproach);
            return;
        }

        if (search_elapsed >= search_timeout_) {
            ROS_WARN("Search timeout exceeded (%.1f s). Initiating fallback landing.", search_elapsed);
            switchToState(PrecisionLandingState::Fallback);
            return;
        }
        
        ROS_INFO_THROTTLE(1.0, "[Search] wait time(max %.2f s): %.2f s, Target search altitude: %.2f", search_timeout_, search_elapsed, search_altitude_);

        // 悬停等待
        publishPositionSetpoint(current_pose_.pose.position.x, current_pose_.pose.position.y, search_altitude_);
    }

    void runStateHorizontalApproach() {
        if (!target_pose_.is_valid || !target_pose_.is_absolute_position_valid) {
            ROS_WARN_THROTTLE(1.0, "No valid target for horizontal approach");
            return;
        }

        // 记录开始水平逼近时的高度
        if (!should_record_z_position_) {
            recorded_z_position_ = current_pose_.pose.position.z;
            should_record_z_position_ = true;
            ROS_INFO("Recorded Z position for horizontal approach: %.2f m", recorded_z_position_);
        }

        // 计算水平距离
        double delta_x = target_pose_.absolute_x - current_pose_.pose.position.x;
        double delta_y = target_pose_.absolute_y - current_pose_.pose.position.y;
        double horizontal_distance = sqrt(delta_x * delta_x + delta_y * delta_y);
        
        ROS_INFO_THROTTLE(1.0, "Horizontal approach: dist=%.2f m (threshold=%.2f m)", 
                          horizontal_distance, horizontal_acceptance_radius_ * error_percent_);
        
        // 检查是否到达水平接受半径
        if (horizontal_distance < horizontal_acceptance_radius_ * error_percent_ && isYawAligned()) {
            ROS_INFO("Reached horizontal acceptance radius (%.2f < %.2f m)", 
                    horizontal_distance, horizontal_acceptance_radius_ * error_percent_);
            switchToState(PrecisionLandingState::DescendAboveTarget);
            return;
        }
        
        // 发布水平逼近设定点（包含Yaw角控制）
        publishPositionSetpoint(target_pose_.absolute_x, target_pose_.absolute_y, recorded_z_position_, target_pose_.absolute_yaw);
        ROS_INFO_THROTTLE(1.0, "Publishing horizontal approach setpoint: x=%.2f, y=%.2f, z=%.2f, yaw=%.1f deg", 
                          target_pose_.absolute_x, target_pose_.absolute_y, recorded_z_position_, 
                          target_pose_.absolute_yaw * 180.0 / M_PI);
    }

    void runStateDescendAboveTarget() {
        if (!target_pose_.is_valid || !target_pose_.is_absolute_position_valid) {
            ROS_WARN("Target lost during descent");
            switchToState(PrecisionLandingState::HorizontalApproach);
            return;
        }
        
        // 计算水平偏移距离
        double delta_x = target_pose_.absolute_x - current_pose_.pose.position.x;
        double delta_y = target_pose_.absolute_y - current_pose_.pose.position.y;
        double horizontal_distance = sqrt(delta_x * delta_x + delta_y * delta_y);

        ROS_INFO_THROTTLE(2.0, "Descending: xy_dist=%.2f m, z_current=%.2f m, z_target=%.2f m", 
                          horizontal_distance, current_pose_.pose.position.z, target_pose_.absolute_z);

        // 检查水平位置是否在允许范围内
        if ((horizontal_distance > horizontal_acceptance_radius_ * error_percent_ || !isYawAligned())  && z_difference >= motor_lock_z_threshold_) {
            ROS_WARN("Horizontal deviation too large (%.2f > %.2f m) - restarting approach",
                    horizontal_distance, horizontal_acceptance_radius_ * error_percent_);
            switchToState(PrecisionLandingState::HorizontalApproach);
            return;
        }

        // 判断是否进入锁桨阶段
        if (z_difference < motor_lock_z_threshold_) {
            ROS_INFO_THROTTLE(0.5, "\033[31mIn motor lock position: xy_dist=%.3f m, z_diff=%.3f m\033[0m", 
                              horizontal_distance, z_difference);
            // 开始地面检查
            if (!is_ground_check_active_ && need_ground_check_) {
                is_ground_check_active_ = true;
                ground_check_start_time_ = ros::Time::now();
                last_altitude_ = current_pose_.pose.position.z;
                ROS_INFO("\033[31mStarting ground check: current height=%.3f m\033[0m", last_altitude_);
                return;
            }
            
            // 执行地面检查逻辑
            double elapsed_time = (ros::Time::now() - ground_check_start_time_).toSec();
            if (elapsed_time >= ground_check_duration_ || !need_ground_check_) {
                double current_altitude = current_pose_.pose.position.z;
                double altitude_change = fabs(current_altitude - last_altitude_);
                ROS_INFO("\033[31mcurrent_alt:%.3f last_alt:%.3fm\033[0m", current_altitude,last_altitude_);

                if (altitude_change < ground_check_interval_ || !need_ground_check_) {
                    if (need_ground_check_) {
                        ROS_INFO("\033[31mGround check passed: altitude change=%.3f m\033[0m", altitude_change);
                    } else {
                        ROS_INFO("\033[31mGround check skipped\033[0m");
                    }
                    
                    lockMotors();
                    switchToState(PrecisionLandingState::Done);
                    return;
                } else {
                    ROS_WARN("\033[31mGround check failed: altitude changed %.3f m - resetting\033[0m", altitude_change);
                    ground_check_start_time_ = ros::Time::now();
                    last_altitude_ = current_altitude;
                }
            }
            
            // 地面检查进行中
            if (need_ground_check_) {
                ROS_INFO_THROTTLE(1.0, "Ground check in progress: %.1f%%", 
                                  (elapsed_time / ground_check_duration_) * 100.0);
            }
        }

        // 计算下降目标高度
        double current_z = current_pose_.pose.position.z;
        if (z_difference <= two_stage_z_threshold_ ) {descent_percent_ = 0.3;}
        
        double descent_speed = descent_z_position_ * descent_percent_;
        
        // 对下降速度进行限幅
        descent_speed = std::min(descent_speed, max_descent_speed_);
        
        double target_z = current_z - descent_speed;

        // 如果在锁桨阶段且不需要地面检查，保持当前高度
        if (z_difference <= motor_lock_z_threshold_ && !need_ground_check_) {
            target_z = current_z;
            ROS_INFO_THROTTLE(1.0, "Motor lock stage: maintaining current altitude %.2f m", current_z);
        }

        // 发布下降设定点（包含Yaw角控制）
        publishPositionSetpoint(target_pose_.absolute_x, target_pose_.absolute_y, target_z, target_pose_.absolute_yaw);
        ROS_INFO_THROTTLE(1.0, "Descending to z=%.2f m (speed=%.3f m/s, limited=%.3f m/s)", 
                          target_z, descent_z_position_ * error_percent_, descent_speed);
    }

    // 修改：添加Yaw角控制参数
    void publishPositionSetpoint(double x, double y, double z, double target_yaw = std::numeric_limits<double>::quiet_NaN()) {
        geometry_msgs::PoseStamped setpoint;
        setpoint.header.stamp = ros::Time::now();
        setpoint.header.frame_id = "map";
        
        // 获取当前无人机位置
        double current_x = current_pose_.pose.position.x;
        double current_y = current_pose_.pose.position.y;
        double current_z = current_pose_.pose.position.z;
        
        // 计算到目标的偏移量
        double delta_x = x - current_x;
        double delta_y = y - current_y;
        double delta_z = z - current_z;
        
        // 计算3D距离
        double distance_3d = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
        
        // 如果距离超过1米，限制在1米范围内（安全限制）
        if (distance_3d > 0.2) {
            double scale_factor = 1.0 / distance_3d;
            setpoint.pose.position.x = current_x + delta_x * scale_factor;
            setpoint.pose.position.y = current_y + delta_y * scale_factor;
            setpoint.pose.position.z = current_z + delta_z * scale_factor;
        } else {
            setpoint.pose.position.x = x;
            setpoint.pose.position.y = y;
            setpoint.pose.position.z = z;
        }
        
        // ========================= Yaw角控制开始 =========================
        // 如果提供了目标Yaw角，则进行Yaw角对齐
        if (!std::isnan(target_yaw)) {
            // 当前Yaw角
            current_target_yaw_ = getYawFromQuaternion(current_pose_.pose.orientation);

            // std::cout<<"target_yaw - current_target_yaw_ = "<<target_yaw<<" - "<<current_target_yaw_<<" = "<<target_yaw - current_target_yaw_<<std::endl;
            
            // 计算Yaw角差值（优化方向）
            double yaw_diff = normalizeYawDifference(target_yaw - current_target_yaw_);

            // 应用最大Yaw角速度限制
            double max_yaw_change = max_yaw_speed_ ;  // 0.1秒的控制周期
            yaw_diff = clamp(yaw_diff, -max_yaw_change, max_yaw_change);
            
            // 创建四元数
            tf2::Quaternion q;
            q.setRPY(0, 0, current_target_yaw_ + yaw_diff);
            setpoint.pose.orientation = tf2::toMsg(q);
            
            // ROS_DEBUG_THROTTLE(1.0, "Yaw control: current=%.1f°, target=%.1f°, diff=%.1f°",
            //                   current_target_yaw_ * 180.0 / M_PI,
            //                   target_yaw * 180.0 / M_PI,
            //                   yaw_diff * 180.0 / M_PI);
        } else {
            // 没有目标Yaw角，保持当前姿态
            setpoint.pose.orientation = current_pose_.pose.orientation;
        }
        // ========================= Yaw角控制结束 =========================
        
        setpoint_publisher_.publish(setpoint);
    }

    void lockMotors() {
        mavros_msgs::CommandLong command;
        command.request.broadcast = false;
        command.request.command = 400; // MAV_CMD_DO_FLIGHTTERMINATION
        command.request.confirmation = 0;
        command.request.param1 = 0.0; // 0=terminate flight (lock motors)
        command.request.param2 = 21196.0; // Magic number for confirmation
        command.request.param3 = 0.0;
        command.request.param4 = 0.0;
        command.request.param5 = 0.0;
        command.request.param6 = 0.0;
        command.request.param7 = 0.0;
        
        if (disarm_client_.call(command)) {
            if (command.response.success) {
                ROS_INFO("Vehicle disarmed and motors locked!");
            } else {
                ROS_ERROR("Disarm command rejected: %d", command.response.result);
            }
        } else {
            ROS_ERROR("Failed to call disarm service");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astra_auto_land_node");
    PrecisionLandingNode node;
    ros::spin();
    return 0;
}