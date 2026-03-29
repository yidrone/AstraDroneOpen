/**
 * @file astra_control.cpp
 * @brief 无人机机柜巡检控制主程序 (Safe Return Logic Version)
 * @details 
 * 该程序负责无人机的顶层逻辑控制，包括：
 * 1. 状态机管理：起飞、巡航、悬停、降落、手动接管、紧急返航、紧急迫降。
 * 2. 路径规划接口：接收 Fast-Planner 的避障指令，或直接发送航点指令。
 * 3. 动态修正：当规划器发现原航点不可达时，接收并更新为安全航点。
 * * @author luli (Updated by Assistant)
 * @date 2025-12-09
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
#include <astra_auto_land/PrecisionLandingActivate.h> 

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

// ================= 全局变量与标志位 =================
int times_detect = 0;          // 视觉检测计数器（用于悬停对准或降落检测）
bool flag_takeoff_done = 0;    // 起飞完成标志

// --- 自动解锁相关 ---
bool flag_offboard_done = false; // 是否已切换到 OFFBOARD 模式
bool flag_arm_done = false;      // 是否已解锁 (ARMED)
ros::ServiceClient arming_client_global; 

// --- 状态标志 ---
bool first_call = true;        // 首次调用标志（用于计时器重置）
bool have_planner_cmd = false; // 是否收到规划器的指令
bool flag_land = false;        // 是否正在降落
bool have_waypoint_mark = false; // 是否检测到航点地标（视觉）
bool have_land_mark = false;     // 是否检测到降落地标（视觉）
ros::Time start_time;          // 通用计时器起始时间

// --- 位姿数据缓存 ---
geometry_msgs::PoseStamped uav_pose;             // 无人机当前位姿 (来自 MAVROS)
geometry_msgs::PoseStamped last_mavros_point_cmd;// 上一次发送给飞控的指令（用于规划器失效时的保持）
geometry_msgs::PoseStamped fly_to_goal_pose;     // "飞向点"模式的目标点缓存

// --- 航点相关 ---
Eigen::Vector3d takeoff_point;    // 起飞点坐标（通常为第一个航点）
Eigen::Vector3d last_pub_point={0,0,0}; // 上一次发布给规划器的目标
Eigen::Vector3d now_pub_point;
Eigen::Vector3d uav_newest_position;    // 无人机当前位置 (Eigen格式)
Eigen::Vector4d adjust_target_position; // 视觉修正后的目标点

// --- 规划器双路输入接口 (位置/速度) ---
// Fast-Planner 可能输出位置指令，也可能输出速度指令（取决于后端优化结果）
bool have_planner_pose_cmd = false;              
bool have_planner_vel_cmd  = false;              
geometry_msgs::PoseStamped  planner_pose_cmd;    // 规划器计算出的下一步期望位置
geometry_msgs::TwistStamped planner_vel_cmd;     // 规划器计算出的下一步期望速度    
ros::Time last_planner_pose_time;                // 最后一次收到位置指令的时间
ros::Time last_planner_vel_time;                 // 最后一次收到速度指令的时间

// --- ROS 通讯对象 ---
ros::Subscriber fastplanner_vel_sub_;                
ros::Publisher  mavros_velocity_cmd_pub;             
ros::Subscriber fly_to_cmd_sub_global;
bool planner_off_sent = false;                   // 降落后是否已发送关闭规划器的指令

// ================= 回调函数：规划器速度指令 =================
// 这是一个静态回调，用于接收 Fast-Planner 的速度控制流
static void plannerVelCallback(const geometry_msgs::TwistStamped& msg) {
  have_planner_vel_cmd = true;
  planner_vel_cmd = msg;
  last_planner_vel_time = ros::Time::now(); // 更新时间戳，用于超时检测
}

namespace astra_control {

// ================= 构造与初始化 =================

LLController::LLController(ros::NodeHandle nh):nh_(nh) {   
    initializeNode();
    std::cout << "\033[47;30m ---------------------------------- Astra Mission Start ---------------------------------- \033[0m" << std::endl;
}

LLController::~LLController(){}

void LLController::initializeNode() {
    load_params(); // 加载 yaml 参数

    // 发布云台控制指令 (x=pitch, y=yaw)
    gimbal_ctrl_pub_ = nh_.advertise<geometry_msgs::Point>("/gimbal/cmd_angle", 10);

    waypoint_now = -1;
    waypoint_next = 0; // 从第0个航点开始

    // --- 订阅话题 ---
    // 1. 无人机状态
    pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("/mavros/local_position/pose", 1,&LLController::positionCallback, this);
    
    // 2. 规划器反馈 (位置控制流 + 速度控制流)
    fastplanner_cmd_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("/fastplanner/setpoint_position/local", 1,&LLController::plannercmdCallback, this);
    fastplanner_vel_sub_ = nh_.subscribe<const geometry_msgs::TwistStamped&>("/fastplanner/setpoint_velocity/cmd_vel", 1, plannerVelCallback);
    
    // 3. 外部控制指令 (SDK/地面站)
    mode_switch_sub_ = nh_.subscribe<const std_msgs::Int8::ConstPtr&>("/astra_control/mode_switch", 1, &LLController::modeSwitchCallback, this);
    manual_cmd_sub_ = nh_.subscribe<const geometry_msgs::Twist::ConstPtr&>("/astra_control/manual_cmd", 1, &LLController::manualCmdCallback, this);
    fly_to_cmd_sub_global = nh_.subscribe<const geometry_msgs::PoseStamped::ConstPtr&>("/astra_control/fly_to_cmd", 1, &LLController::flyToCmdCallback, this);
    
    // 4. 【关键】订阅 Fast-Planner 修正后的安全目标点
    // 当原目标点在障碍物内时，规划器会发布修正后的坐标到这里
    new_goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/planner/new_goal_by_planner", 1, &LLController::newGoalCallback, this);

    // 5. 视觉辅助 (地标检测)
    detect_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("/detect/waypoint_mark", 1,&LLController::waypointMarkCallback, this);
    land_mark_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("/detect/land_mark", 1,&LLController::landMarkCallback, this);
    
    // --- 发布话题 ---
    // 1. 发送给 PX4 飞控的指令
    mavros_point_cmd_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 50);
    mavros_velocity_cmd_pub  = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 50);
    
    // 2. 发送给 Fast-Planner 的目标点
    setplanner_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/fastplanner/goal", 1);
    
    // 3. 控制信号
    offplanner_pub_ = nh_.advertise<std_msgs::Bool>("/planner/off", 1); // 控制规划器开关

    // --- 服务客户端 ---
    land_client = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    arming_client_global = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    land_activate_client_ = nh_.serviceClient<astra_auto_land::PrecisionLandingActivate>("/astra_auto_land/activate");

    // --- 核心控制定时器 (50Hz = 0.02s，当前设置为 0.05s = 20Hz) ---
    cmd_timer = nh_.createTimer(ros::Duration(0.05), &LLController::cmdCallback, this);
    
    precision_landing_active_ = false; 
    
    // 初始化 FlyTo 目标缓存
    fly_to_goal_pose.pose.position.x = 0;
    fly_to_goal_pose.pose.position.y = 0;
    fly_to_goal_pose.pose.position.z = 1.0;
    fly_to_goal_pose.pose.orientation.w = 1.0;
}

// ================= 回调函数集 =================

/**
 * @brief 处理规划器返回的安全目标点 (避障修正)
 * @details 当规划器发现原目标点不可达时，会计算一个最近的安全点发过来。
 * 这里接收后，直接修改当前航点坐标，防止无人机卡死。
 */
void LLController::newGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 只有在巡航模式下才更新航点列表
    if (Drone_mode == Run_point) {
        if (waypoint_next >= 0 && waypoint_next < waypoint_list.size()) {
            ROS_WARN("\033[33m[Control] Obstacle detected! Planner adjusted Waypoint %d.\033[0m", waypoint_next);
            
            // 更新当前目标点坐标为安全坐标
            waypoint_list[waypoint_next].x = msg->pose.position.x;
            waypoint_list[waypoint_next].y = msg->pose.position.y;
            waypoint_list[waypoint_next].z = msg->pose.position.z;
            
            // 此时 patrol() 函数中的 distance3d 计算将基于新坐标
            // 当无人机飞到新坐标时，距离 < 阈值，即可正常切换到下一个航点
        }
    } 
    // 如果是飞单点模式，直接修改单点目标
    else if (Drone_mode == Fly_to_Point) {
        ROS_WARN("[Control] FlyTo target adjusted by planner.");
        fly_to_goal_pose.pose.position = msg->pose.position;
    }
}

// 接收 SDK 发来的单点飞行指令
void LLController::flyToCmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    fly_to_goal_pose = *msg;
    ROS_INFO("\033[35m[FlyTo] Received Goal: [%.2f, %.2f, %.2f]\033[0m", 
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

/**
 * @brief 模式切换回调 (接收 SDK / 地面站指令)
 * @param msg 1:紧急返航, 2:紧急降落, 3:手动, 4:飞定点, 5:恢复任务
 */
void LLController::modeSwitchCallback(const std_msgs::Int8::ConstPtr& msg) {
    // 【优先级 1】紧急降落 (最高优先级)
    if (msg->data == 2) {
        if (precision_landing_active_) {
            ROS_WARN("\033[31m[MODE] EMERGENCY LAND! Breaking Precision Landing connection.\033[0m");
            precision_landing_active_ = false; // 打断精准降落
        }
        Drone_mode = Emergency_Land;
        emergency_land_lock_pose = uav_pose; // 记录当前位置作为锁定点
        ROS_WARN("\033[31m[MODE] Switched to Emergency Land (Forced Descent)!\033[0m");
        return;
    }

    // --- 状态清理逻辑 ---
    // 如果收到有效指令，且当前正在进行精准降落，则打断它
    bool valid_switch = false;
    if (msg->data == 1 || msg->data == 3 || msg->data == 4 || msg->data == 5) valid_switch = true;

    if (valid_switch && precision_landing_active_) {
        ROS_WARN("[Control] Interrupting Precision Landing for Mode Switch %d", msg->data);
        precision_landing_active_ = false;
    }

    // 【优先级 2】常规模式切换
    switch (msg->data) {
        case 1: // 紧急返航
            Drone_mode = Emergency_Return;
            // 【关键修改】切换模式时，不清除 planner 标志位，允许规划器工作
            // 但为了安全，先置空旧指令，等待新指令生成
            have_planner_cmd = false; 
            ROS_WARN("\033[31m[MODE] Switched to Emergency Return (Planner Active)!\033[0m");
            break;
            
        case 3: // 手动观察模式 (绕过规划器)
            if (Drone_mode != Manual_Control) {
                Drone_mode = Manual_Control;
                patrol_cmd = uav_pose; // 初始化手动目标为当前位置
                ROS_WARN("\033[33m[MODE] Switched to Manual Control (SDK)!\033[0m");
            }
            break;
        
        case 4: // 飞定点模式
            Drone_mode = Fly_to_Point;
            ROS_INFO("\033[35m[MODE] Switched to Fly to Point (FastPlanner)!\033[0m");
            break;

        case 5: // 恢复任务 / 开启任务
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

/**
 * @brief 手动控制回调 (SDK 键盘控制)
 * @details 步进为 5cm，相对机身坐标系控制
 */
void LLController::manualCmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (Drone_mode != Manual_Control) return;

    double current_yaw = tf::getYaw(uav_pose.pose.orientation);
    double step_val = 0.05; // 每次按键移动 5cm
    
    double input_x = msg->linear.x; 
    double input_y = msg->linear.y; 
    double input_z = msg->linear.z; 
    double input_yaw = msg->angular.z; 

    // 机身坐标系转换到世界坐标系
    double world_dx = (input_x * step_val) * cos(current_yaw) - (input_y * step_val) * sin(current_yaw);
    double world_dy = (input_x * step_val) * sin(current_yaw) + (input_y * step_val) * cos(current_yaw);
    double world_dz = input_z * step_val;

    // 累加到目标点
    patrol_cmd.pose.position.x += world_dx;
    patrol_cmd.pose.position.y += world_dy;
    patrol_cmd.pose.position.z += world_dz;

    // 地面保护
    if (patrol_cmd.pose.position.z < 0.2) patrol_cmd.pose.position.z = 0.2;

    // Yaw 控制
    double step_yaw_rad = input_yaw * 0.1; 
    double target_yaw = tf::getYaw(patrol_cmd.pose.orientation) + step_yaw_rad;
    patrol_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw);
}

// 接收 MAVROS 发布的无人机位姿
void LLController::positionCallback(const geometry_msgs::PoseStamped& msg) {
    uav_pose = msg;
    uav_newest_position = LLController::toEigen(msg.pose.position);  
    
    // 起飞逻辑检测
    if(!flag_takeoff_done){
        if(debug) ROS_INFO_THROTTLE(1, "\033[34m[Takeoff] Ascending... Dist: %.2f m \033[0m ", (uav_newest_position - takeoff_point).norm());
        // 到达起飞高度
        if((uav_newest_position - takeoff_point).norm() < takeoff_threshould){
            flag_takeoff_done = 1;
            ROS_INFO("\033[32m[Takeoff] Done. Moving to Waypoint 0.\033[0m");
            NextPoint();
            Drone_mode = Run_point; // 自动进入巡航
        }
    }
    else{
        // 仅在非特殊模式下计算自动飞行逻辑 (Run_point, Aligning, Land, Emergency_Return)
        if (Drone_mode != Emergency_Land && Drone_mode != Manual_Control) {
            patrol();
        }
    }
}

/**
 * @brief 核心业务逻辑：计算距离、判断到达、状态流转
 * @details 负责向规划器发布目标点 (pub_goal)
 */
void LLController::patrol(){
    geometry_msgs::PoseStamped next_position_msg;
    float dis_to_next_position = 0; 
    double yaw;
    double current_yaw = tf::getYaw(uav_pose.pose.orientation);

    switch(Drone_mode) {
        // --- 1. 自动巡航模式 ---
        case Run_point:  
            // 计算到当前航点的距离
            dis_to_next_position = distance3d(uav_newest_position[0], uav_newest_position[1], uav_newest_position[2], 
            waypoint_list[waypoint_next].x ,waypoint_list[waypoint_next].y, waypoint_list[waypoint_next].z);

            // 构造目标点消息
            next_position_msg.pose.position.x = waypoint_list[waypoint_next].x;
            next_position_msg.pose.position.y = waypoint_list[waypoint_next].y;
            next_position_msg.pose.position.z = waypoint_list[waypoint_next].z;
            yaw = waypoint_list[waypoint_next].yaw;
            next_position_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

            arrive_goal_threshould = waypoint_threshould;
            
            // 判断是否到达
            if (dis_to_next_position <= arrive_goal_threshould && abs(current_yaw - yaw) <= arrive_yaw_threshould) {
                // 根据航点类型执行动作
                switch (Point_mode) {
                    case Hover_detect: // 悬停模式
                        Drone_mode = Aligning; 
                        is_hovering_ = false;  
                        ROS_INFO("\033[34m[Patrol] Arrived at Point %d. Hovering for %.1f s.\033[0m", waypoint_next, waypoint_list[waypoint_next].dwell_time);
                        break;
                    case Pass_through: // 通过模式
                        ROS_INFO("[Patrol] Arrived at Point %d (Pass-through). Moving Next.", waypoint_next);
                        NextPoint(); 
                        break;
                    case Land_point:   // 降落点
                        ROS_INFO("\033[32m[Patrol] Arrived at Land Point. Starting Landing Sequence.\033[0m");
                        Drone_mode = Land;
                        break;
                }
            } else {
                // 未到达，持续发布给规划器
                pub_goal(next_position_msg); 
            }
            break;

        // --- 2. 悬停模式 ---
        case Aligning:   
            // 锁定在当前航点位置
            patrol_cmd.pose.position.x = waypoint_list[waypoint_next].x;
            patrol_cmd.pose.position.y = waypoint_list[waypoint_next].y;
            patrol_cmd.pose.position.z = waypoint_list[waypoint_next].z;
            patrol_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[waypoint_next].yaw);
            
            // 计时逻辑
            if (!is_hovering_) {
                hover_start_time_ = ros::Time::now();
                is_hovering_ = true;
            } else {
                double elapsed = (ros::Time::now() - hover_start_time_).toSec();
                double target_dwell = waypoint_list[waypoint_next].dwell_time;
                if (elapsed >= target_dwell) {
                    ROS_INFO("\033[32m[Patrol] Hover Complete (%.1fs). Moving to Next Point.\033[0m", elapsed);
                    NextPoint();
                    Drone_mode = Run_point;
                    is_hovering_ = false;
                } else {
                    ROS_INFO_THROTTLE(1, "[Patrol] Hovering... %.1f / %.1f s", elapsed, target_dwell);
                }
            }
            break;

        // --- 3. 降落准备模式 ---
        case Land:   
            // 飞向降落点上方
            patrol_cmd.pose.position.x = waypoint_list[waypoint_next].x;
            patrol_cmd.pose.position.y = waypoint_list[waypoint_next].y;
            patrol_cmd.pose.position.z = waypoint_list[waypoint_next].z; 
            patrol_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[waypoint_next].yaw);
            
            dis_to_next_position = distance3d(uav_newest_position[0], uav_newest_position[1], uav_newest_position[2],
            waypoint_list[waypoint_next].x, waypoint_list[waypoint_next].y, waypoint_list[waypoint_next].z);

            // 到达后，激活精准降落服务
            if (dis_to_next_position <= waypoint_threshould) {
                 if (!precision_landing_active_) {
                    ROS_INFO_THROTTLE(2, "\033[33m[Land] Requesting Precision Landing Activation...\033[0m");
                    astra_auto_land::PrecisionLandingActivate srv;
                    srv.request.activate = true;
                    if (land_activate_client_.call(srv)) {
                        if (srv.response.success) {
                            ROS_WARN("\033[32m[Land] Precision Landing Handover Success!\033[0m");
                            precision_landing_active_ = true; 
                        } else {
                            ROS_WARN_THROTTLE(1, "[Land] Wait for target: %s", srv.response.message.c_str());
                        }
                    }
                }
            }
            break;

        // --- 4. 紧急返航模式 (逻辑重构) ---
        case Emergency_Return:
            // 计算到起飞点的距离
            dis_to_next_position = distance3d(uav_newest_position[0], uav_newest_position[1], 0, 
                                              takeoff_point[0], takeoff_point[1], 0);

            // 设定目标：水平回到起飞点，高度保持或设为安全高度
            next_position_msg.pose.position.x = takeoff_point[0];
            next_position_msg.pose.position.y = takeoff_point[1];
            next_position_msg.pose.position.z = takeoff_point[2]; // 使用起飞高度作为返航高度
            next_position_msg.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[0].yaw); 
            
            // 【关键】强制发布给规划器进行避障规划
            // 这里我们绕过 flag_planner_px4 的检查，直接发布
            next_position_msg.header.frame_id = "camera_init";
            next_position_msg.header.stamp = ros::Time::now();
            setplanner_goal_pub_.publish(next_position_msg); 
            
            ROS_INFO_THROTTLE(2, "\033[31m[Return] Sending Home Goal to Planner... Dist: %.2f m\033[0m", dis_to_next_position);

            // 到达起飞点附近后，自动降落
            if (dis_to_next_position <= 0.5) {
                ROS_WARN("[Return] Arrived Home. Switching to LAND mode.");
                Drone_mode = Land;
                Point_mode = Land_point; 
                waypoint_next = 0; // 重置航点索引到0
                precision_landing_active_ = false; 
            }
            break;
        
        // --- 5. 飞向单点模式 ---
        case Fly_to_Point:
            pub_goal(fly_to_goal_pose);
            break;

        default:
            break;
    }
}

// 接收规划器计算出的 Pose 指令
void LLController::plannercmdCallback(const geometry_msgs::PoseStamped& msg) {
    have_planner_cmd = true;            
    have_planner_pose_cmd = true;       
    planner_cmd = msg;
    planner_pose_cmd = msg;
    last_planner_pose_time = ros::Time::now();
}

// ... (其他回调函数保持不变)

// ================= 核心控制循环 (20Hz) =================
// 负责将最终指令发送给 MAVROS
void LLController::cmdCallback(const ros::TimerEvent& event) {
    // 优先级判断：如果正在精准降落，且未被手动或紧急迫降打断，则不发指令，交由 auto_land 控制
    if (precision_landing_active_ && Drone_mode != Manual_Control && Drone_mode != Emergency_Land) {
        return; 
    }
    
    // 数据完整性检查
    if(!isQuaternionNormalized(uav_pose.pose.orientation)){
        return;
    }

    switch(Drone_mode) {
        case Takeoff:  
            mavros_point_cmd.pose.position.x = takeoff_point[0];
            mavros_point_cmd.pose.position.y = takeoff_point[1];
            mavros_point_cmd.pose.position.z = takeoff_point[2];
            mavros_point_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_list[0].yaw);
            
            // ... (自动解锁逻辑省略，保持原样) ...
            // (请保留原代码中的自动解锁块)
            break;

        // 【核心修改】将 Emergency_Return 独立处理，不再混入 Run_point
        
        // 1. 正常巡航 / 飞单点
        case Run_point:
        case Fly_to_Point:  
             if(!flag_planner_px4){
                // 使用规划器
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
                // 不使用规划器 (直飞)
                 if (Drone_mode == Fly_to_Point) mavros_point_cmd = fly_to_goal_pose;
                 else mavros_point_cmd = patrol_cmd;
            }
            break;

        // 2. 紧急返航 (强制使用规划器)
        case Emergency_Return:
            {
                ros::Time now = ros::Time::now();
                // 检查规划器数据是否新鲜
                bool planner_active = have_planner_pose_cmd && (now - last_planner_pose_time).toSec() <= planner_cmd_stale_threshould;
                bool vel_active = have_planner_vel_cmd && (now - last_planner_vel_time).toSec() <= planner_cmd_stale_threshould;

                if (planner_active) {
                     mavros_point_cmd = planner_pose_cmd;
                     // ROS_INFO_THROTTLE(2, "\033[32m[Return] Following Planner Path...\033[0m");
                } 
                else if (vel_active) {
                     geometry_msgs::TwistStamped tw = planner_vel_cmd;
                     tw.header.stamp = now;
                     tw.header.frame_id = "camera_init";
                     mavros_velocity_cmd_pub.publish(tw);
                     return;
                }
                else {
                    // 【安全兜底】如果规划器挂了，不要直飞，而是原地悬停并警告
                    // 这里设置为当前位置，或者保持上一个有效位置
                    // mavros_point_cmd = uav_pose; 
                    ROS_WARN_THROTTLE(1, "\033[31m[Return] Planner NOT Ready! Hovering for safety.\033[0m");
                    // 保持 last_mavros_point_cmd 不变，或者设为当前位置
                }
            }
            break;

        case Aligning:   
            have_planner_cmd = false;
            mavros_point_cmd = patrol_cmd; 
            break;

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
            mavros_point_cmd.pose.position.z = -0.10; 
            mavros_point_cmd.pose.orientation = emergency_land_lock_pose.pose.orientation;
            
            ROS_WARN_THROTTLE(1, "\033[31m[Emergency] FORCING DESCEND (Override All).\033[0m");
            
            if (uav_pose.pose.position.z < 0.2) {
                Lock();
            }
            break;
    }

    // --- 安全插值与限幅 (Smooth Control) ---
    // (保持原有的平滑逻辑，防止指令突变)
    Eigen::Vector3d current_pos(uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z);
    Eigen::Vector3d target_pos(mavros_point_cmd.pose.position.x, mavros_point_cmd.pose.position.y, mavros_point_cmd.pose.position.z);
    
    double distance_to_target = (target_pos - current_pos).norm();
    if (distance_to_target > px4_max_distance) {
        Eigen::Vector3d direction = (target_pos - current_pos).normalized();
        Eigen::Vector3d new_pos = current_pos + direction * px4_max_distance;
        mavros_point_cmd.pose.position.x = new_pos.x();
        mavros_point_cmd.pose.position.y = new_pos.y();
        mavros_point_cmd.pose.position.z = new_pos.z();
    }

    double current_yaw = tf::getYaw(uav_pose.pose.orientation);
    double target_yaw = tf::getYaw(mavros_point_cmd.pose.orientation);
    double yaw_diff = atan2(sin(target_yaw - current_yaw), cos(target_yaw - current_yaw));

    if (fabs(yaw_diff) > max_yaw_change) {
        yaw_diff = (yaw_diff > 0) ? max_yaw_change : -max_yaw_change;
    }

    double interpolated_yaw = current_yaw + yaw_diff;
    mavros_point_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(interpolated_yaw);

    // 发布指令
    mavros_point_cmd.header.stamp = ros::Time::now();
    mavros_point_cmd.header.frame_id = "camera_init";
    mavros_point_cmd_pub.publish(mavros_point_cmd);
    last_mavros_point_cmd = mavros_point_cmd;

    // 落地锁桨
    if(Drone_mode == Land && uav_pose.pose.position.z <= 0.05){
        Lock();
        if(!planner_off_sent){
            std_msgs::Bool off; off.data = true;
            offplanner_pub_.publish(off);
            planner_off_sent = true;
        }
    }
}

// ... (Lock, CallLand, load_params, pub_goal, NextPoint 等辅助函数保持不变)
void LLController::Lock() {
    mavros_msgs::CommandLong cmd;
    cmd.request.broadcast = false;
    cmd.request.command = 400;          
    cmd.request.confirmation = 0;       
    cmd.request.param1 = 0.0;           
    cmd.request.param2 = 21196.0;       
    if(land_client.call(cmd) && cmd.response.success){ROS_INFO_THROTTLE(2, "Vehicle DisArmed!");}
}

void LLController::CallLand() {
    // 保留旧接口，目前主要使用 modeSwitch
    flag_land = true;
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
    
    land_height = nh_.param("land_height", 0.3);
    px4_max_distance = nh_.param("px4_max_distance", 1.2);
    max_yaw_change = nh_.param("max_yaw_change", 0.3);

    nh_.getParam("/debug", debug);

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
                
                // 【需求1】解析 dwell_time
                if (waypoint_list_temp[i].hasMember("dwell_time")) {
                    wp.dwell_time = static_cast<double>(waypoint_list_temp[i]["dwell_time"]);
                } else {
                    wp.dwell_time = 0.0;
                }
                // 【需求2】读取云台角度 (默认值为0)
                if (waypoint_list_temp[i].hasMember("gimbal_pitch")) {
                    wp.gimbal_pitch = static_cast<double>(waypoint_list_temp[i]["gimbal_pitch"]);
                } else {
                    wp.gimbal_pitch = 0.0;
                }

                if (waypoint_list_temp[i].hasMember("gimbal_yaw")) {
                    wp.gimbal_yaw = static_cast<double>(waypoint_list_temp[i]["gimbal_yaw"]);
                } else {
                    wp.gimbal_yaw = 0.0;
                }

                wp.pointmode = static_cast<std::string>(waypoint_list_temp[i]["pointmode"]);

                // 转换逻辑：优先判断是否为降落点，其次根据时间判断
                if (wp.pointmode == "Land_point") {
                    wp.mode_enum = Land_point;
                } else if (wp.dwell_time > 0) {
                    wp.mode_enum = Hover_detect;
                } else {
                    wp.mode_enum = Pass_through;
                }

                waypoint_list.push_back(wp);

                if(debug) ROS_INFO("Loaded WP[%d]: x=%.2f, time=%.1f, mode=%d", i, wp.x, wp.dwell_time, wp.mode_enum);
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
    
    // 更新 Point_mode
    Point_mode = waypoint_list[waypoint_next].mode_enum;
    // 【新增】发布云台控制指令
    geometry_msgs::Point gimbal_msg;
    gimbal_msg.x = waypoint_list[waypoint_next].gimbal_pitch;
    gimbal_msg.y = waypoint_list[waypoint_next].gimbal_yaw;
    gimbal_msg.z = 0.0; // 预留
    gimbal_ctrl_pub_.publish(gimbal_msg);

    // std::cout << "\033[32m >> Switching to Waypoint: " << waypoint_next 
    //           << " | Gimbal: (" << gimbal_msg.x << ", " << gimbal_msg.y << ")"
    //           << " | Mode: " << (Point_mode==Hover_detect ? "Hover" : (Point_mode==Land_point ? "Land" : "Pass"))
    //           << " | Dwell: " << waypoint_list[waypoint_next].dwell_time << "s \033[0m" << std::endl;
    ROS_INFO_STREAM("\033[32m >> Switching to Waypoint: " << waypoint_next 
                << " | Gimbal: (" << gimbal_msg.x << ", " << gimbal_msg.y << ")"
                << " | Mode: " << (Point_mode==Hover_detect ? "Hover" : (Point_mode==Land_point ? "Land" : "Pass"))
                << " | Dwell: " << waypoint_list[waypoint_next].dwell_time << "s \033[0m");

}

// 辅助检测函数已内化到 patrol 状态机中，此处保留空定义或直接删除均可，为保证编译通过暂不删除
// bool LLController::WayPointDetectDone() { ... } 
// bool LLController::LandDetectDone() { ... }
} // namespace
