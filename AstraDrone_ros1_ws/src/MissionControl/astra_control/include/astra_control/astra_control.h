#ifndef _LL_CONTROLLER_NEW_H_
#define _LL_CONTROLLER_NEW_H_

// --- ROS 标准库 ---
#include <ros/ros.h>
#include <map>
#include <cmath>
#include <string>
#include <vector>

// --- ROS 消息类型 ---
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h> 
#include <std_msgs/String.h>

// --- 数学库 ---
#include <Eigen/Core>

namespace astra_control {

/**
 * @brief 无人机飞行状态机枚举
 */
enum Dronemode {
    Takeoff,            // 起飞阶段
    Run_point,          // 巡航阶段：飞向下一个航点
    Aligning,           // 悬停/对齐阶段：到达点后执行 dwell_time 悬停
    Land,               // 降落阶段：准备降落或正在移交控制权
    Emergency_Return,   // 紧急返航：忽略规划器，直飞起飞点
    Emergency_Land,     // 紧急降落：最高优先级，原地强制迫降
    Manual_Control,     // 手动接管：响应 SDK 步进指令
    Fly_to_Point        // 飞定点模式：响应外部单点指令
};

/**
 * @brief 航点类型枚举 (由 dwell_time 和 pointmode 字符串解析而来)
 */
enum Pointmode {
    Pass_through,   // 快速通过模式 (dwell_time = 0)
    Hover_detect,   // 悬停侦测模式 (dwell_time > 0)
    Land_point      // 降落点标记
};

class LLController{
public:
    LLController(ros::NodeHandle nh);
    ~LLController();

    /**
     * @brief 初始化节点：参数加载、发布订阅初始化、定时器启动
     */
    void initializeNode();

    /**
     * @brief 从 yaml 文件加载参数 (包括 dwell_time)
     */
    void load_params();
    
    // --- 辅助工具函数 ---
    inline Eigen::Vector3f toEigen(const geometry_msgs::Point& p) {
        Eigen::Vector3f ev3(p.x, p.y, p.z);
        return ev3;
    }

    float distance3d(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2) {
        return std::sqrt(std::pow(x_2 - x_1, 2) + std::pow(y_2 - y_1, 2) + std::pow(z_2 - z_1, 2));
    }
    
private:
    /**
     * @brief 内部航点结构体
     */
    struct Waypoint {
        double x, y, z, yaw;
        float dwell_time,gimbal_pitch,cam_zoom;      // 【新增】悬停时间，单位秒
        std::string pointmode; // 原始配置字符串
        Pointmode mode_enum;   // 解析后的枚举状态
    };

    // --- 核心控制句柄 ---
    ros::NodeHandle nh_;
    ros::Timer cmd_timer;      // 核心控制循环定时器

    // --- 发布者 (Publishers) ---
    ros::Publisher setplanner_goal_pub_;    // 发送给 FastPlanner 的目标点
    ros::Publisher mavros_point_cmd_pub;    // 发送给 Mavros 的位置指令 (Setpoint)
    ros::Publisher mavros_velocity_cmd_pub; // 发送给 Mavros 的速度指令
    ros::Publisher offplanner_pub_;         // 控制规划器启停
    // 【新增】云台控制发布者
    ros::Publisher gimbal_ctrl_pub_; 
    ros::Publisher camera_zoom_pub_; 
    ros::Publisher camera_exposure_pub_;
    // --- 订阅者 (Subscribers) ---
    ros::Subscriber pose_sub_;              // 无人机当前位姿
    ros::Subscriber fastplanner_cmd_sub_;   // 规划器输出的位置指令
    ros::Subscriber fastplanner_vel_sub_;   // 规划器输出的速度指令
    ros::Subscriber detect_sub_;            // 视觉检测结果 (航点修正用)
    ros::Subscriber land_mark_sub_;         // 降落地标检测
    // 【新增】订阅规划器修正后的目标点
    ros::Subscriber new_goal_sub_;

    // --- 外部指令订阅 ---
    ros::Subscriber mode_switch_sub_;       // 模式切换指令 (返航/降落/手动/恢复)
    ros::Subscriber manual_cmd_sub_;        // 手动控制步进指令

    // --- 服务客户端 (Clients) ---
    ros::ServiceClient land_client;         // 请求降落服务 (Mavros)
    ros::ServiceClient set_mode_client;     // 切换飞行模式 (Offboard/Auto.Land)
    ros::ServiceClient land_activate_client_; // 激活精准降落节点的 Client

    // --- 状态变量 ---
    geometry_msgs::PoseStamped mavros_point_cmd; // 发送给飞控的最终指令
    geometry_msgs::PoseStamped planner_cmd;      // 规划器指令缓存
    geometry_msgs::PoseStamped patrol_cmd;       // 巡航/手动/悬停生成的目标点
    geometry_msgs::PoseStamped waypoint_mark;    // 视觉航点数据
    geometry_msgs::PoseStamped land_mark;        // 视觉降落数据
    geometry_msgs::PoseStamped emergency_land_lock_pose; // 紧急降落时的水平锁定坐标

    // --- 标志位 ---
    bool precision_landing_active_ = false; // 是否已移交控制权给精准降落节点
    bool flag_planner_px4 = 1;              // 是否使用规划器 (1:使用, 0:直连)
    bool debug = 0;
    bool flag_landing_detect = 1;
    bool auto_land = 0;

    // --- 悬停计时相关 ---
    ros::Time hover_start_time_;
    bool is_hovering_ = false;
    //云台控制相关
    bool gimbal_action_sent_= false; 
    // --- 自动解锁相关辅助 ---
    ros::Time arm_wait_start_time_;
    bool arm_wait_started_ = false;

    // --- 阈值参数 ---
    float arrive_goal_threshould = 0.3;
    float takeoff_threshould = 0.3;
    float waypoint_threshould = 0.3;
    float aligning_threshould = 0.15;
    float landing_threshould = 0.15;
    float arrive_yaw_threshould = 0.3;
    float times_detect_threshould = 30;
    float planner_min_pub_threshould = 0.01;
    float waypoint_adjust_max_second_threshould = 15;
    float land_adjust_max_second_threshould = 10;
    double planner_cmd_stale_threshould = 0.3;

    // --- 飞行限制参数 ---
    float land_height = 0.3;
    float px4_max_distance = 1.2; // 最大单次指令距离 (插值限制)
    float max_yaw_change = 0.2;   // 最大偏航角速度限制

    // --- 航点管理 ---
    int waypoint_next = 0; // 下一个目标点索引
    int waypoint_now = 0;  // 当前到达点索引
    std::vector<Waypoint> waypoint_list;

    Dronemode Drone_mode = Takeoff;
    Pointmode Point_mode = Pass_through;
    // 【新增】紧急降落判定偏移量
    double emergency_land_lock_offset = 0.2;
    double land_check_anchor_z;
    ros::Time land_check_start_time;
    // --- 内部功能函数 ---
    void Unlock(); // 解锁 (未使用，逻辑在 main loop)
    void Lock();   // 上锁
    void CallLand(); // 呼叫降落 (旧接口)
    
    /**
     * @brief 核心巡航逻辑处理
     * @details 计算距离、处理状态机跳转、管理悬停计时
     */
    void patrol(); 

    /**
     * @brief 发布目标点给规划器或控制器
     * @param goal_msg 目标位姿
     */
    void pub_goal(geometry_msgs::PoseStamped goal_msg);

    /**
     * @brief 切换到下一个航点
     * @details 更新索引、Point_mode 和日志输出
     */
    void NextPoint();

    // --- 回调函数 ---
    void positionCallback(const geometry_msgs::PoseStamped& msg);
    void plannercmdCallback(const geometry_msgs::PoseStamped& msg);
    void cmdCallback(const ros::TimerEvent& event); // 50Hz 控制循环
    void waypointMarkCallback(const geometry_msgs::PoseStamped& msg);
    void landMarkCallback(const geometry_msgs::PoseStamped& msg);

    // 新增回调
    void modeSwitchCallback(const std_msgs::Int8::ConstPtr& msg);
    void manualCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void flyToCmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    // 【新增】回调函数声明
    void newGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};
}
#endif
