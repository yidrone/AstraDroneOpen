/**
 * @file astra_control.cpp
 * @brief 无人机机柜巡检控制主程序 (V5.5 - 适配 SDK V5.4 协议变更)
 * @details 
 * 1. 飞定点模式复用巡航逻辑
 * 2. 悬停时云台执行点头动作 (+Pitch -> -Pitch)
 * 3. 航点 Yaw 接口替换为 变焦(Zoom)
 * 4. 新增相机曝光与变焦控制接口
 * @author luli (Updated by Assistant)
 * @date 2025-12-19
 */

#include "astra_control/astra_control.h"
#include <tf/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>       
#include <geometry_msgs/Twist.h> 
#include <std_msgs/Float32.h> // 【新增】用于变焦和曝光控制
#include <astra_auto_land/PrecisionLandingActivate.h> 

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

// ================= 全局变量 =================
int times_detect = 0;  
bool flag_takeoff_done = 0;

// 自动解锁相关
bool flag_offboard_done = false; 
bool flag_arm_done = false;      
ros::ServiceClient arming_client_global; 

// 状态标志位
bool first_call = true;
bool have_planner_cmd = false, flag_land = false, have_waypoint_mark = false, have_land_mark = false;
ros::Time start_time;

geometry_msgs::PoseStamped uav_pose;
geometry_msgs::PoseStamped last_mavros_point_cmd;
// geometry_msgs::PoseStamped fly_to_goal_pose; // 【修改】飞定点复用巡航，不再需要单独缓存

Eigen::Vector3f takeoff_point;
Eigen::Vector3f last_pub_point={0,0,0};
Eigen::Vector3f now_pub_point;
Eigen::Vector3f uav_newest_position;
Eigen::Vector4f adjust_target_position;

// 规划器双路输入
bool have_planner_pose_cmd = false;              
bool have_planner_vel_cmd  = false;              
geometry_msgs::PoseStamped  planner_pose_cmd;        
geometry_msgs::TwistStamped planner_vel_cmd;         
ros::Time last_planner_pose_time, last_planner_vel_time; 

ros::Subscriber fastplanner_vel_sub_;                
ros::Publisher  mavros_velocity_cmd_pub;             
ros::Subscriber fly_to_cmd_sub_global;
bool planner_off_sent = false;                        

// 静态回调：规划器速度指令
static void plannerVelCallback(const geometry_msgs::TwistStamped& msg) {
  have_planner_vel_cmd = true;
  planner_vel_cmd = msg;
  last_planner_vel_time = ros::Time::now();
}

namespace astra_control {

// ================= 构造与初始化 =================

LLController::LLController(ros::NodeHandle nh):nh_(nh) {   
    initializeNode();
    std::cout << "\033[47;30m ---------------------------------- Astra Mission Start (V5.5) ---------------------------------- \033[0m" << std::endl;
}

LLController::~LLController(){}

void LLController::initializeNode() {
    load_params();
    
    // 【修改】发布云台控制指令
    gimbal_ctrl_pub_ = nh_.advertise<geometry_msgs::Point>("/gimbal/cmd_angle", 10);
    
    // 【新增】相机控制接口 (话题与 uav_bridge_sdk.py 保持一致)
    camera_zoom_pub_ = nh_.advertise<std_msgs::Float32>("/zoom_value", 10);
    camera_exposure_pub_ = nh_.advertise<std_msgs::Float32>("/camera/cmd_exposure", 10);

    waypoint_now = -1;
    waypoint_next = 0; // 从第0个点开始

    // 订阅
    pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("/mavros/local_position/pose", 1,&LLController::positionCallback, this);
    fastplanner_cmd_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("/fastplanner/setpoint_position/local", 1,&LLController::plannercmdCallback, this);
    fastplanner_vel_sub_ = nh_.subscribe<const geometry_msgs::TwistStamped&>("/fastplanner/setpoint_velocity/cmd_vel", 1, plannerVelCallback);
    
    // 控制接口
    mode_switch_sub_ = nh_.subscribe<const std_msgs::Int8::ConstPtr&>("/astra_control/mode_switch", 1, &LLController::modeSwitchCallback, this);
    manual_cmd_sub_ = nh_.subscribe<const geometry_msgs::Twist::ConstPtr&>("/astra_control/manual_cmd", 1, &LLController::manualCmdCallback, this);
    
    // fly_to_cmd_sub_global 依然保留订阅，防止报错，但在本逻辑中不再用于单独模式
    fly_to_cmd_sub_global = nh_.subscribe<const geometry_msgs::PoseStamped::ConstPtr&>("/astra_control/fly_to_cmd", 1, &LLController::flyToCmdCallback, this);
    
    // 订阅 Fast-Planner 修正后的安全目标点
    new_goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/planner/new_goal_by_planner", 1, &LLController::newGoalCallback, this);

    // 视觉辅助
    detect_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("/detect/waypoint_mark", 1,&LLController::waypointMarkCallback, this);
    land_mark_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("/detect/land_mark", 1,&LLController::landMarkCallback, this);
    
    // 发布
    mavros_point_cmd_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 50);
    mavros_velocity_cmd_pub  = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 50);
    setplanner_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/fastplanner/goal", 1);
    offplanner_pub_ = nh_.advertise<std_msgs::Bool>("/planner/off", 1);

    // 服务客户端
    land_client = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    arming_client_global = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    land_activate_client_ = nh_.serviceClient<astra_auto_land::PrecisionLandingActivate>("/astra_auto_land/activate");

    // 定时器
    cmd_timer = nh_.createTimer(ros::Duration(0.05), &LLController::cmdCallback, this);
    
    precision_landing_active_ = false; 
}

// ================= 回调函数集 =================

void LLController::newGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 【修改】飞定点模式现已合并入 Run_point，统一处理
    if (Drone_mode == Run_point) {
        if (waypoint_next >= 0 && waypoint_next < waypoint_list.size()) {
            ROS_WARN("\033[33m[Control] Obstacle detected! Planner adjusted Waypoint %d.\033[0m", waypoint_next);
            ROS_INFO("Old Target: [%.2f, %.2f, %.2f]", waypoint_list[waypoint_next].x, waypoint_list[waypoint_next].y, waypoint_list[waypoint_next].z);
            
            waypoint_list[waypoint_next].x = msg->pose.position.x;
            waypoint_list[waypoint_next].y = msg->pose.position.y;
            waypoint_list[waypoint_next].z = msg->pose.position.z;
            ROS_INFO("New Target: [%.2f, %.2f, %.2f]", waypoint_list[waypoint_next].x, waypoint_list[waypoint_next].y, waypoint_list[waypoint_next].z);

        }
    } 
}
//保留函数头防止报错
void LLController::flyToCmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 即使在复用逻辑下，保留此回调以防止发布者端报错，实际逻辑已由 YAML 重载接管
    // fly_to_goal_pose = *msg; 
}

/**
 * @brief 模式切换回调
 */
void LLController::modeSwitchCallback(const std_msgs::Int8::ConstPtr& msg) {
    if (msg->data == 2) {
        if (precision_landing_active_) {
            ROS_WARN("\033[31m[MODE] EMERGENCY LAND! Breaking Precision Landing connection.\033[0m");
            precision_landing_active_ = false; 
        }
        Drone_mode = Emergency_Land;
        emergency_land_lock_pose = uav_pose; 
        // 【新增】初始化落地检测变量
        land_check_anchor_z = uav_pose.pose.position.z; // 以当前高度为初始锚点
        land_check_start_time = ros::Time::now();       // 开始计时
        ROS_WARN("\033[31m[MODE] Switched to Emergency Land (Forced Descent)!\033[0m");
        return;
    }

    bool valid_switch = false;
    if (msg->data == 1 || msg->data == 3 || msg->data == 4 || msg->data == 5) {
        valid_switch = true;
    }

    if (valid_switch && precision_landing_active_) {
        ROS_WARN("[Control] Interrupting Precision Landing for Mode Switch %d", msg->data);
        precision_landing_active_ = false;
    }

    switch (msg->data) {
        case 1: // 紧急返航
            Drone_mode = Emergency_Return;
            ROS_WARN("\033[31m[MODE] Switched to Emergency Return!\033[0m");
            break;
            
        case 3: // 手动观察模式
            if (Drone_mode != Manual_Control) {
                Drone_mode = Manual_Control;
                patrol_cmd = uav_pose; 
                ROS_WARN("\033[33m[MODE] Switched to Manual Control (SDK)!\033[0m");
            }
            break;
        
        case 4: // 飞定点模式
            // 【需求1】飞定点模式复用巡航逻辑
            // SDK 已经重写了 YAML 并触发了逻辑，这里只需切换到 Run_point
            Drone_mode = Run_point; 
            // 如果 SDK 写入了新的 YAML，通常需要重新加载参数，但这里假设 SDK 每次 mission 都会重启节点或重新赋值
            // 如果节点不重启，需要在此处调用 load_params() 重新读取 YAML 中的新任务
            load_params(); 
            // 重置航点索引
            waypoint_next = 0; 
            flag_takeoff_done = 0; // 如果需要重新起飞逻辑
            // 针对已经在空中的情况，flag_takeoff_done 的逻辑在 positionCallback 中处理
            // 如果已经在空中，直接 NextPoint()
            if (uav_pose.pose.position.z > 0.5) {
                 flag_takeoff_done = 1;
                 // 重新规划到第0点（通常是起飞点，或者 SDK 设定的第一个目标）
                 // SDK 构造的任务：0=起飞点(0,0,1), 1=目标点, 2=降落点
                 waypoint_next = 0; 
                 NextPoint(); // 切到 0
            }

            ROS_INFO("\033[35m[MODE] Switched to Fly to Point (Using Patrol Logic)!\033[0m");
            break;

        case 5: // 恢复任务
            if (Drone_mode == Emergency_Land || Drone_mode == Emergency_Return) {
                 ROS_WARN("[MODE] Cannot resume from Emergency State.");
            } else {
                 Drone_mode = Run_point;
                 have_planner_cmd = false;
                 ROS_INFO("\033[32m[MODE] Resuming Mission... Continuing to Waypoint %d\033[0m", waypoint_next);
            }
            break;
            
        default:
            ROS_WARN("Unknown mode switch command: %d", msg->data);
            break;
    }
}

void LLController::manualCmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (Drone_mode != Manual_Control) return;
    double current_yaw = tf::getYaw(uav_pose.pose.orientation);
    double step_val = 1.0; 
    
    double input_x = msg->linear.x; 
    double input_y = msg->linear.y; 
    double input_z = msg->linear.z; 
    double input_yaw = msg->angular.z; 

    // --- 2. 简单的硬编码限幅 (Hardcoded Limits) ---
    // 限制前后左右上下步进最大为 0.2 米
    double max_linear_limit = 0.2;
    
    // 限制偏航角步进最大为15度 (约 0.2615 弧度)
    double max_yaw_limit = 15.0 * 3.1415926 / 180.0; // ~0.523 rad

    // [Input X Limit]
    if (input_x > max_linear_limit) input_x = max_linear_limit;
    else if (input_x < -max_linear_limit) input_x = -max_linear_limit;

    // [Input Y Limit]
    if (input_y > max_linear_limit) input_y = max_linear_limit;
    else if (input_y < -max_linear_limit) input_y = -max_linear_limit;

    // [Input Z Limit]
    if (input_z > max_linear_limit) input_z = max_linear_limit;
    else if (input_z < -max_linear_limit) input_z = -max_linear_limit;

    // [Input Yaw Limit]
    if (input_yaw > max_yaw_limit) input_yaw = max_yaw_limit;
    else if (input_yaw < -max_yaw_limit) input_yaw = -max_yaw_limit;

    double world_dx = (input_x * step_val) * cos(current_yaw) - (input_y * step_val) * sin(current_yaw);
    double world_dy = (input_x * step_val) * sin(current_yaw) + (input_y * step_val) * cos(current_yaw);
    double world_dz = input_z * step_val;

    patrol_cmd.pose.position.x += world_dx;
    patrol_cmd.pose.position.y += world_dy;
    patrol_cmd.pose.position.z += world_dz;

    if (patrol_cmd.pose.position.z < 0.2) patrol_cmd.pose.position.z = 0.2;

    double step_yaw_rad = input_yaw; 
    double target_yaw = tf::getYaw(patrol_cmd.pose.orientation) + step_yaw_rad;
    patrol_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw);
}
void LLController::positionCallback(const geometry_msgs::PoseStamped& msg) {
    uav_pose = msg;
    uav_newest_position = LLController::toEigen(msg.pose.position);  
    
    if(!flag_takeoff_done){
        if(debug) ROS_INFO_THROTTLE(1, "\033[34m[Takeoff] Ascending... Dist: %.2f m \033[0m ", (uav_newest_position - takeoff_point).norm());
        if((uav_newest_position - takeoff_point).norm() < takeoff_threshould){
            flag_takeoff_done = 1;
            ROS_INFO("\033[32m[Takeoff] Done. Moving to Waypoint 0.\033[0m");
            NextPoint();
            Drone_mode = Run_point;
        }
    }
    else{
        if (Drone_mode != Emergency_Land && Drone_mode != Manual_Control) {
            patrol();
        }
    }
}

/**
 * @brief 核心巡航逻辑
 */
void LLController::patrol(){
    geometry_msgs::PoseStamped next_position_msg;
    float dis_to_next_position = 0; 
    double yaw;
    double current_yaw = tf::getYaw(uav_pose.pose.orientation);

    switch(Drone_mode) {
        case Run_point:  
            dis_to_next_position = distance3d(uav_newest_position[0], uav_newest_position[1], uav_newest_position[2], 
            waypoint_list[waypoint_next].x ,waypoint_list[waypoint_next].y, waypoint_list[waypoint_next].z);

            next_position_msg.pose.position.x = waypoint_list[waypoint_next].x;
            next_position_msg.pose.position.y = waypoint_list[waypoint_next].y;
            next_position_msg.pose.position.z = waypoint_list[waypoint_next].z;
            yaw = waypoint_list[waypoint_next].yaw;
            next_position_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            //NextPoint()里面添加了云台的俯仰和变焦控制，这里不用再加了


            arrive_goal_threshould = waypoint_threshould;
            
            if (dis_to_next_position <= arrive_goal_threshould && abs(current_yaw - yaw) <= arrive_yaw_threshould) {
                switch (Point_mode) {
                    case Hover_detect: 
                        Drone_mode = Aligning; 
                        is_hovering_ = false; 
                        ROS_INFO("\033[34m[Patrol] Arrived at Point %d. Starting Gimbal Action & Hover.\033[0m", waypoint_next);
                        break;
                    
                    case Pass_through: 
                        ROS_INFO("[Patrol] Arrived at Point %d (Pass-through). Moving Next.", waypoint_next);
                        NextPoint(); 
                        break;
                        
                    case Land_point: 
                        ROS_INFO("\033[32m[Patrol] Arrived at Land Point. Starting Landing Sequence.\033[0m");
                        Drone_mode = Land;
                        break;
                }
            } else {
                pub_goal(next_position_msg); 
            }
            break;
        case Aligning:   
            // 悬停状态：锁定在当前点
            patrol_cmd.pose.position.x = waypoint_list[waypoint_next].x;
            patrol_cmd.pose.position.y = waypoint_list[waypoint_next].y;
            patrol_cmd.pose.position.z = waypoint_list[waypoint_next].z;
            patrol_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[waypoint_next].yaw);
            
            // 悬停计时与云台逻辑
            if (!is_hovering_) {
                hover_start_time_ = ros::Time::now();
                is_hovering_ = true;
                // 【新增】每次进入悬停状态时，重置标志位，允许发送一次指令
                gimbal_action_sent_ = false; 
            } else {
                double elapsed = (ros::Time::now() - hover_start_time_).toSec();
                double target_dwell = waypoint_list[waypoint_next].dwell_time;
                double target_pitch = waypoint_list[waypoint_next].gimbal_pitch; // 获取 YAML 中的 pitch
                
                // 【需求】云台点头逻辑 (Up -> Down)
                geometry_msgs::Point gimbal_msg;
                gimbal_msg.y = 0; // Yaw 已改为变焦，此处 Yaw 设为 0
                gimbal_msg.z = 0;
                // 悬停 0.12 秒后执行向下动作
                if (elapsed < 0.12) {
                     // 等待期，只打印日志防止刷屏
                     // ROS_WARN_THROTTLE(1, "Waiting for gimbal action...");
                } else {
                    // 【关键修改】检查标志位，如果没发送过才发送
                    if (!gimbal_action_sent_) {
                        gimbal_msg.x = -1.0 * target_pitch; 
                        gimbal_ctrl_pub_.publish(gimbal_msg); 
                        
                        // 发送后立即置为 true，防止下一帧重复发送
                        gimbal_action_sent_ = true; 
                        
                        ROS_INFO("\033[32m[Patrol] Gimbal Pitch Down Triggered: %.1f (Sent Once)\033[0m", gimbal_msg.x);
                    }
                }

                if (elapsed >= target_dwell) {
                    ROS_INFO("\033[32m[Patrol] Hover Complete (%.1fs). Moving to Next Point.\033[0m", elapsed);
                    NextPoint();
                    Drone_mode = Run_point;
                    is_hovering_ = false;
                }
                else {
                    ROS_INFO_THROTTLE(1, "[Patrol] Hovering... %.1f / %.1f s", elapsed, target_dwell);
                }
            }
            break;

        case Land:   
            // 1. 持续锁定目标位置（保持悬停姿态）
            patrol_cmd.pose.position.x = waypoint_list[waypoint_next].x;
            patrol_cmd.pose.position.y = waypoint_list[waypoint_next].y;
            patrol_cmd.pose.position.z = waypoint_list[waypoint_next].z; 
            patrol_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[waypoint_next].yaw);
            
            // 2. 计算距离
            dis_to_next_position = distance3d(uav_newest_position[0], uav_newest_position[1], uav_newest_position[2],
            waypoint_list[waypoint_next].x, waypoint_list[waypoint_next].y, waypoint_list[waypoint_next].z);

            // 3. 到达目标点范围，开始逻辑
            if (dis_to_next_position <= waypoint_threshould) {
                
                // 【新增】如果还没有开始悬停计时，初始化计时器
                if (!is_hovering_) {
                    hover_start_time_ = ros::Time::now();
                    is_hovering_ = true;
                    double wait_time = waypoint_list[waypoint_next].dwell_time;
                    ROS_INFO("\033[32m[Land] Arrived at Land Point. Hovering for %.1f s before descent.\033[0m", wait_time);
                } 
                else {
                    // 计算已悬停时间
                    double elapsed = (ros::Time::now() - hover_start_time_).toSec();
                    double target_dwell = waypoint_list[waypoint_next].dwell_time;

                    // 【新增】判断时间是否满足
                    if (elapsed < target_dwell) {
                        // 时间未到，保持悬停，打印倒计时
                        ROS_INFO_THROTTLE(1, "[Land] Waiting... %.1f / %.1f s", elapsed, target_dwell);
                    } 
                    else {
                        // 时间已到，执行原来的降落/精准降落逻辑
                        if (!precision_landing_active_) {
                            ROS_INFO_THROTTLE(2, "\033[33m[Land] Hover finished. Requesting Precision Landing Activation...\033[0m");
                            astra_auto_land::PrecisionLandingActivate srv;
                            srv.request.activate = true;
                            if (land_activate_client_.call(srv)) {
                                if (srv.response.success) {
                                    ROS_WARN("\033[32m[Land] Precision Landing Handover Success!\033[0m");
                                    precision_landing_active_ = true; 
                                    is_hovering_ = false; // 任务交接完成，重置标志位
                                } else {
                                    ROS_WARN_THROTTLE(1, "[Land] Wait for target: %s", srv.response.message.c_str());
                                }
                            }
                        }
                    }
                }
            }
            break;
        case Emergency_Return:
            {
                if (waypoint_list.empty()) return;
                int land_idx = waypoint_list.size() - 1; 

                geometry_msgs::PoseStamped return_goal;
                return_goal.pose.position.x = waypoint_list[land_idx].x;
                return_goal.pose.position.y = waypoint_list[land_idx].y;
                return_goal.pose.position.z = waypoint_list[land_idx].z;
                return_goal.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[land_idx].yaw);
                
                dis_to_next_position = distance3d(uav_newest_position[0], uav_newest_position[1], uav_newest_position[2], 
                                                  waypoint_list[land_idx].x, waypoint_list[land_idx].y, waypoint_list[land_idx].z);

                if (dis_to_next_position <= waypoint_threshould) {
                    ROS_WARN("[Return] Arrived at Land Point (via Planner). Switching to Precision Landing.");
                    
                    Drone_mode = Land;
                    Point_mode = Land_point; 
                    waypoint_next = land_idx;          
                    precision_landing_active_ = false; 
                } else {
                    pub_goal(return_goal);
                }
            }
            break;       
        
        // 【修改】移除了单独的 Fly_to_Point case，因为逻辑已合并到 Run_point

        default:
            break;
    }
}

void LLController::plannercmdCallback(const geometry_msgs::PoseStamped& msg) {
    have_planner_cmd = true;            
    have_planner_pose_cmd = true;       
    planner_cmd = msg;
    planner_pose_cmd = msg;
    last_planner_pose_time = ros::Time::now();
}

void LLController::waypointMarkCallback(const geometry_msgs::PoseStamped& msg) {
    have_waypoint_mark = true;
    waypoint_mark = msg;
}

void LLController::landMarkCallback(const geometry_msgs::PoseStamped& msg) {
    have_land_mark = true;
    land_mark = msg;
}

bool isQuaternionNormalized(const geometry_msgs::Quaternion& q, double tolerance = 1e-6) {
    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return std::abs(norm - 1.0) < tolerance;
}

// ================= 核心控制循环 (50Hz - 20Hz) =================
void LLController::cmdCallback(const ros::TimerEvent& event) {
    if (precision_landing_active_ && Drone_mode != Manual_Control && Drone_mode != Emergency_Land) {
        return; 
    }
    
    if(!isQuaternionNormalized(uav_pose.pose.orientation)){
        return;
    }

    switch(Drone_mode) {
        case Takeoff:  
            mavros_point_cmd.pose.position.x = takeoff_point[0];
            mavros_point_cmd.pose.position.y = takeoff_point[1];
            mavros_point_cmd.pose.position.z = takeoff_point[2];
            mavros_point_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[0].yaw);
            
            {
                static int start_stream_count = 0;
                start_stream_count++;
                if (start_stream_count > 20) {
                    if (!flag_offboard_done) {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                            ROS_INFO("\033[32m [Auto] Offboard Enabled \033[0m");
                            flag_offboard_done = true;
                        }
                    } 
                    else if (!flag_arm_done) {
                        if (!arm_wait_started_) {
                            arm_wait_start_time_ = ros::Time::now();
                            arm_wait_started_ = true;
                            ROS_INFO("Pre-Arming Wait: 8 seconds...");
                            return; 
                        }
                        double wait_time = (ros::Time::now() - arm_wait_start_time_).toSec();
                        if (wait_time < 8.0) return;

                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;
                        if (arming_client_global.call(arm_cmd) && arm_cmd.response.success) {
                            ROS_INFO("\033[32m [Auto] Vehicle ARMED \033[0m");
                            flag_arm_done = true;
                        }
                    }
                }
            }
            break;

        case Run_point:
        case Emergency_Return:  
             if(!flag_planner_px4){
                ros::Time now = ros::Time::now();
                bool pose_fresh = have_planner_pose_cmd && (now - last_planner_pose_time).toSec() <= planner_cmd_stale_threshould;
                bool vel_fresh  = have_planner_vel_cmd  && (now - last_planner_vel_time).toSec() <= planner_cmd_stale_threshould;

                if(pose_fresh){
                    mavros_point_cmd = planner_pose_cmd;   
                } else if (vel_fresh) {
                    geometry_msgs::TwistStamped tw = planner_vel_cmd;
                    tw.header.stamp = now;
                    if(tw.header.frame_id.empty()) tw.header.frame_id = "camera_init";
                    mavros_velocity_cmd_pub.publish(tw);
                    return; 
                } else {
                    mavros_point_cmd = last_mavros_point_cmd; 
                    ROS_WARN_THROTTLE(1, "Planner CMD Stale! Holding position.");
                }
            } else {
                mavros_point_cmd = patrol_cmd;
            }
            break;
            
        case Aligning:   
        case Land:       
            have_planner_cmd = false;
            mavros_point_cmd = patrol_cmd;
            break;

        case Manual_Control:
            have_planner_cmd = false;
            have_planner_vel_cmd = false;
            mavros_point_cmd = patrol_cmd; 
            ROS_INFO_THROTTLE(2, "\033[33m[Manual] Direct Control. Planner Bypassed.\033[0m");
            break;

        case Emergency_Land:
            have_planner_cmd = false;
            have_planner_vel_cmd = false;
            
            mavros_point_cmd.pose.position.x = emergency_land_lock_pose.pose.position.x;
            mavros_point_cmd.pose.position.y = emergency_land_lock_pose.pose.position.y;
            mavros_point_cmd.pose.position.z = -0.50; 
            mavros_point_cmd.pose.orientation = emergency_land_lock_pose.pose.orientation;
            
            // ROS_WARN_THROTTLE(1, "\033[31m[Emergency] FORCING DESCEND (Offboard Z=-0.3).\033[0m");
            
            double current_z = uav_pose.pose.position.z;
            double diff = std::abs(current_z - land_check_anchor_z);
            
            // 如果高度变化超过 3cm (0.03)，说明还在动
            if (diff > 0.03) {
                land_check_anchor_z = current_z; // 更新锚点
                land_check_start_time = ros::Time::now(); // 重置计时
                ROS_WARN_THROTTLE(1,"\033[32m[Emergency]wait for lock!\033[0m");
            } 
            else {
                // 高度变化很小，检查持续了多久
                if ((ros::Time::now() - land_check_start_time).toSec() > 1.0) {
                    ROS_WARN_THROTTLE(1,"\033[32m[Emergency] Touchdown Detected (Stable for 1s). Locking!\033[0m");
                    Lock(); 
                }
            }
            break;
    }

    
    // Z轴平滑下降限制
    double max_descent_speed = 5.0; 
    double loop_dt = 0.05;          
    double max_z_step = max_descent_speed * loop_dt; 

    if (Drone_mode == Land || Drone_mode == Emergency_Land) {
        double current_z = uav_pose.pose.position.z;
        if (mavros_point_cmd.pose.position.z < current_z - max_z_step) {
            mavros_point_cmd.pose.position.z = current_z - max_z_step;
        }
    }

    Eigen::Vector3d current_pos(uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z);
    Eigen::Vector3d target_pos(mavros_point_cmd.pose.position.x, mavros_point_cmd.pose.position.y, mavros_point_cmd.pose.position.z);
    
    // 距离限幅
    double distance_to_target = (target_pos - current_pos).norm();
    if (distance_to_target > px4_max_distance) {
        Eigen::Vector3d direction = (target_pos - current_pos).normalized();
        Eigen::Vector3d new_pos = current_pos + direction * px4_max_distance;
        mavros_point_cmd.pose.position.x = new_pos.x();
        mavros_point_cmd.pose.position.y = new_pos.y();
        mavros_point_cmd.pose.position.z = new_pos.z();
    }

    if (std::isnan(mavros_point_cmd.pose.orientation.x) || std::isnan(mavros_point_cmd.pose.orientation.y) || 
        std::isnan(mavros_point_cmd.pose.orientation.z) || std::isnan(mavros_point_cmd.pose.orientation.w) ||
        (fabs(mavros_point_cmd.pose.orientation.x) < 1e-9 && fabs(mavros_point_cmd.pose.orientation.y) < 1e-9 && 
        fabs(mavros_point_cmd.pose.orientation.z) < 1e-9 && fabs(mavros_point_cmd.pose.orientation.w) < 1e-9)) {
        
        mavros_point_cmd.pose.orientation = uav_pose.pose.orientation;
        ROS_WARN_THROTTLE(2.0, "Invalid target quaternion detected, using current orientation");
    }
    
    double current_yaw = tf::getYaw(uav_pose.pose.orientation);
    double target_yaw = tf::getYaw(mavros_point_cmd.pose.orientation);
    double yaw_diff = atan2(sin(target_yaw - current_yaw), cos(target_yaw - current_yaw));
    
    if (fabs(yaw_diff) > max_yaw_change) {
        yaw_diff = (yaw_diff > 0) ? max_yaw_change : -max_yaw_change;
    }

    double interpolated_yaw = current_yaw + yaw_diff;
    mavros_point_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(interpolated_yaw);

    mavros_point_cmd.header.stamp = ros::Time::now();
    mavros_point_cmd.header.frame_id = "camera_init";
    mavros_point_cmd_pub.publish(mavros_point_cmd);
    last_mavros_point_cmd = mavros_point_cmd;

    if(Drone_mode == Land && uav_pose.pose.position.z <= 0.05){
        Lock();
        if(!planner_off_sent){
            std_msgs::Bool off; off.data = true;
            offplanner_pub_.publish(off);
            planner_off_sent = true;
            ROS_INFO_THROTTLE(2, "Published /planner/off = true after landing.");
        }
    }
}

void LLController::Lock() {
    mavros_msgs::CommandLong cmd;
    cmd.request.broadcast = false;
    cmd.request.command = 400;          
    cmd.request.confirmation = 0;       
    cmd.request.param1 = 0.0;           
    cmd.request.param2 = 21196.0;       
    if(land_client.call(cmd) && cmd.response.success){ROS_WARN_THROTTLE(2, "Vehicle DisArmed!");}
}

void LLController::CallLand() {
    flag_land = true; 
    ROS_INFO_THROTTLE(2, "[CallLand] Landing flag set. Ensuring Offboard control.");
}

void LLController::load_params() {
    flag_planner_px4 = nh_.param("switch/flag_planner_px4", 1);
    flag_landing_detect = nh_.param("switch/flag_landing_detect", 1);
    auto_land = nh_.param("switch/auto_land", 0);
    
    takeoff_threshould = nh_.param("threshould/takeoff_threshould", 0.3);
    planner_min_pub_threshould = nh_.param("threshould/planner_min_pub_threshould", 0.02);
    waypoint_threshould = nh_.param("threshould/waypoint_threshould", 0.3);
    aligning_threshould = nh_.param("threshould/aligning_threshould", 0.15);
    landing_threshould = nh_.param("threshould/landing_threshould", 0.15);
    arrive_yaw_threshould = nh_.param("threshould/arrive_yaw_threshould", 0.3);
    times_detect_threshould = nh_.param("threshould/times_detect_threshould", 30);
    waypoint_adjust_max_second_threshould = nh_.param("threshould/waypoint_adjust_max_second_threshould", 10);
    land_adjust_max_second_threshould = nh_.param("threshould/land_adjust_max_second_threshould", 10);
    planner_cmd_stale_threshould = nh_.param("threshould/planner_cmd_stale_threshould", 0.3);
    
    emergency_land_lock_offset = nh_.param("threshould/emergency_land_lock_offset", 0.20);
    land_height = nh_.param("land_height", 0.3);
    px4_max_distance = nh_.param("px4_max_distance", 1.2);
    max_yaw_change = nh_.param("max_yaw_change", 0.3);

    nh_.getParam("/debug", debug);
    
    // 【修改】每次调用都清空航点列表，确保重载生效
    waypoint_list.clear(); 

    XmlRpc::XmlRpcValue waypoint_list_temp;
    if (nh_.getParam("waypoints", waypoint_list_temp))
    {
        if (waypoint_list_temp.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (int i = 0; i < waypoint_list_temp.size(); ++i)
            {
                Waypoint wp;
                wp.x = static_cast<double>(waypoint_list_temp[i]["x"]);
                wp.y = static_cast<double>(waypoint_list_temp[i]["y"]);
                wp.z = static_cast<double>(waypoint_list_temp[i]["z"]);
                wp.yaw = static_cast<double>(waypoint_list_temp[i]["yaw"]) * 3.1415 / 180;
                
                if (waypoint_list_temp[i].hasMember("dwell_time")) {
                    wp.dwell_time = static_cast<double>(waypoint_list_temp[i]["dwell_time"]);
                } else {
                    wp.dwell_time = 0.0;
                }
                
                if (waypoint_list_temp[i].hasMember("gimbal_pitch")) {
                    wp.gimbal_pitch = static_cast<double>(waypoint_list_temp[i]["gimbal_pitch"]);
                } else {
                    wp.gimbal_pitch = 0.0;
                }

                if (waypoint_list_temp[i].hasMember("cam_zoom")) {
                    wp.cam_zoom = static_cast<double>(waypoint_list_temp[i]["cam_zoom"]); 
                } else {
                    wp.cam_zoom = 1.0; // 默认变焦倍数 1.0
                }

                wp.pointmode = static_cast<std::string>(waypoint_list_temp[i]["pointmode"]);

                if (wp.pointmode == "Land_point") {
                    wp.mode_enum = Land_point;
                } else if (wp.dwell_time > 0) {
                    wp.mode_enum = Hover_detect;
                } else {
                    wp.mode_enum = Pass_through;
                }

                waypoint_list.push_back(wp);

                if(debug) ROS_INFO("Loaded WP[%d]: x=%.2f, time=%.1f, zoom=%.1f", i, wp.x, wp.dwell_time, wp.cam_zoom);
            }
        }
    }

    if(!waypoint_list.empty()){
        takeoff_point[0] = waypoint_list[0].x;
        takeoff_point[1] = waypoint_list[0].y;
        takeoff_point[2] = waypoint_list[0].z;
    } else {
        ROS_WARN("No waypoints loaded! Using default takeoff point.");
        takeoff_point << 0, 0, 1.0;
    }
}

void LLController::pub_goal(geometry_msgs::PoseStamped goal_msg){
    goal_msg.header.frame_id="camera_init";
    goal_msg.header.stamp = ros::Time::now();

    if(!flag_planner_px4){
        now_pub_point[0] = goal_msg.pose.position.x;
        now_pub_point[1] = goal_msg.pose.position.y;
        now_pub_point[2] = goal_msg.pose.position.z;

        if ((last_pub_point - now_pub_point).norm() >= planner_min_pub_threshould){
            goal_msg.header.seq +=1;
            setplanner_goal_pub_.publish(goal_msg);
            last_pub_point[0]=goal_msg.pose.position.x;
            last_pub_point[1]=goal_msg.pose.position.y;
            last_pub_point[2]=goal_msg.pose.position.z;
        }
    }else{
        goal_msg.header.seq +=1;
        patrol_cmd = goal_msg;
    }
}

void LLController::NextPoint() {
    waypoint_now = waypoint_next;
    waypoint_next = waypoint_now +1;
    if(waypoint_next >= waypoint_list.size()-1){  waypoint_next = waypoint_list.size()-1; }
    
    Point_mode = waypoint_list[waypoint_next].mode_enum;
    
    // 【需求3 & 4】发布云台 Pitch 和 相机变焦
    // 1. 云台控制 (仅使用 x=pitch, y=yaw=0)
    geometry_msgs::Point gimbal_msg;
    gimbal_msg.x = waypoint_list[waypoint_next].gimbal_pitch;
    gimbal_msg.y = 0.0; // Yaw 已被变焦取代，此处设为0
    gimbal_msg.z = 0.0; 
    gimbal_ctrl_pub_.publish(gimbal_msg);

    // 2. 变焦控制
    std_msgs::Float32 zoom_msg;
    zoom_msg.data = waypoint_list[waypoint_next].cam_zoom;
    camera_zoom_pub_.publish(zoom_msg);

    ROS_INFO_STREAM("\033[32m >> Switching to Waypoint: " << waypoint_next 
                << " | Pitch: " << gimbal_msg.x 
                << " | Zoom: " << zoom_msg.data
                << " | Mode: " << (Point_mode==Hover_detect ? "Hover" : (Point_mode==Land_point ? "Land" : "Pass"))
                << " | Dwell: " << waypoint_list[waypoint_next].dwell_time << "s \033[0m");

}

} // namespace
