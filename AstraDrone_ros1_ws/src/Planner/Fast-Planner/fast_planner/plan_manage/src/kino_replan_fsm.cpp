/*
 * This file is part of Fast-Planner.
 * Copyright ... (保留原有版权声明)
 */

/**
 * @file kino_replan_fsm.cpp
 * @brief 动力学重规划状态机实现文件
 * @details 负责调度底层的路径搜索与优化算法，处理状态流转、碰撞检测及目标点修正
 */

#include <tf/tf.h>
#include <plan_manage/kino_replan_fsm.h>

using std::cout;
using std::endl;
using std::string;

namespace fast_planner {

// ==============================================================================================
// 1. 初始化函数：读取参数，配置ROS通讯接口
// ==============================================================================================
void KinoReplanFSM::init(ros::NodeHandle& nh) {
  // 初始化成员变量
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT; // 初始状态为 INIT
  have_target_ = false;
  have_odom_   = false;
  trigger_     = false;

  /* --- 读取 FSM 参数 --- */
  nh.param("fsm/flight_type", target_type_, -1); // 飞行类型：1=手动目标，2=预设航点
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);        // (旧参数) 重规划阈值
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);  // (旧参数) 不重规划阈值
  nh.param("fsm/replan_hz", replan_hz_, 5.0);                 // 【关键参数】定时重规划的频率 (例如 2.0Hz)

  // 读取预设航点列表 (仅当 target_type_ == 2 时使用)
  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  if (waypoint_num_ > MAX_WAYPOINTS) {
    ROS_WARN("waypoint_num(%d) > MAX_WAYPOINTS(%d), will clamp.", waypoint_num_, MAX_WAYPOINTS);
    waypoint_num_ = MAX_WAYPOINTS;
  }
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + std::to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + std::to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + std::to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* --- 初始化核心规划模块 --- */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh); // 初始化地图、A*搜索器、B样条优化器等
  visualization_.reset(new PlanningVisualization(nh)); // 初始化可视化工具

  /* --- 设置定时器 --- */
  // 1. 主状态机循环：100Hz (0.01s)，负责状态跳转和逻辑分发
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  // 2. 安全检测循环：20Hz (0.05s)，负责碰撞检测和终点安全性修正
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  // safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  // 3. 定时重规划定时器：根据 param 设置的频率执行 (例如 2Hz)
  //    这保证了在动态环境中能够持续更新轨迹
  if (replan_hz_ > 0.0) {
    replan_timer_ = nh.createTimer(ros::Duration(1.0 / replan_hz_), &KinoReplanFSM::replanTimerCallback, this);
  } else {
    ROS_WARN("fsm/replan_hz <= 0, periodic replan disabled.");
  }

  /* --- 订阅者 Subscribers --- */
  // 订阅目标点 (通常来自 Rviz 的 2D Nav Goal 或 waypoint_generator)
  waypoint_sub_ = nh.subscribe("/fastplanner/goal", 1, &KinoReplanFSM::waypointCallback, this);
  // 订阅里程计 (无人机当前位姿)
  odom_sub_     = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);
  // 【新增】订阅强制停止指令 (用于 astra_control 接管控制权时暂停规划)
  off_sub_      = nh.subscribe("/planner/off", 1, &KinoReplanFSM::plannerOffCallback, this);

  /* --- 发布者 Publishers --- */
  replan_pub_   = nh.advertise<std_msgs::Empty>("/planning/replan", 10);  // (兼容旧代码)
  new_pub_      = nh.advertise<std_msgs::Empty>("/planning/new", 10);     // (兼容旧代码)
  bspline_pub_  = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10); // 发布规划好的 B-spline 轨迹
  
  // 【新增】发布修正后的目标点 (当原始目标在障碍物内时，发布移动后的安全点)
  new_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/planner/new_goal_by_planner", 1); 
}

// ==============================================================================================
// 2. 回调函数：处理外部输入信号
// ==============================================================================================

/**
 * @brief 处理 /planner/off 指令
 * @details 当收到 true 时，强制将状态机置为 WAIT_TARGET，停止规划，防止与外部控制冲突
 */
void KinoReplanFSM::plannerOffCallback(const std_msgs::BoolConstPtr& msg) {
  planner_forced_wait_ = msg->data;
  if (planner_forced_wait_) {
    changeFSMExecState(WAIT_TARGET, "OFF");
    ROS_WARN("[OFF] Planner forced to WAIT_TARGET.");
  } else {
    ROS_INFO("[OFF] Planner resumed.");
  }
}

/**
 * @brief 处理新的目标点输入
 * @details 接收目标点，设置 end_pt_ 和 end_vel_，并触发状态机进入 GEN_NEW_TRAJ
 */
void KinoReplanFSM::waypointCallback(const geometry_msgs::PoseStamped msg) {
  if (msg.pose.position.z <= -0.5) return; // 忽略异常的地下目标点

  cout << "Triggered!" << endl;
  trigger_ = true; // 标记收到触发信号

  // 根据模式设置终点
  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    // 模式1：手动目标 (Rviz点击)
    end_pt_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    // 模式2：预设航点循环
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % std::max(1, waypoint_num_);
  }

  // 设置终点期望速度
  // 模长设为 0.1 m/s，方向沿目标点的 Yaw 角方向
  // 这样规划出的轨迹末端会有特定的朝向
  double yaw = tf::getYaw(msg.pose.orientation);
  if (std::isnan(yaw)) yaw = 0.0;
  end_vel_ << 0.1 * cos(yaw), 0.1 * sin(yaw), 0.0;

  // 在 Rviz 中画出目标点
  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  have_target_ = true;

  // 状态流转逻辑：
  // 如果当前是空闲 (WAIT_TARGET) 或 正在执行 (EXEC_TRAJ)，且未被强制暂停，则立即生成新轨迹
  if ((exec_state_ == WAIT_TARGET || exec_state_ == EXEC_TRAJ) && !planner_forced_wait_)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  // 如果已经在执行中，通常由定时器负责重规划，这里不做硬性打断
  else if (exec_state_ == EXEC_TRAJ)
    ; 
}

/**
 * @brief 处理里程计数据
 * @details 更新无人机当前的全局位置、速度和姿态
 */
void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  // 提取位置
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  // 提取速度
  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  // 提取姿态 (四元数)
  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true; // 标记里程计已就绪
}

// 辅助函数：改变状态并打印日志
void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

// 辅助函数：打印当前状态 (用于调试)
void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

// ==============================================================================================
// 3. 核心状态机循环 (100Hz)
// ==============================================================================================
void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  // 1. 定时打印当前状态 (每100次调用打印一次，即 1秒1次)
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "no goal." << endl;
    fsm_num = 0;
  }

  // 2. 优先级处理：如果被强制 OFF，无条件转入 WAIT_TARGET
  if (planner_forced_wait_ && exec_state_ != WAIT_TARGET) {
    changeFSMExecState(WAIT_TARGET, "FSM");
    ROS_WARN("[exec call back]planner forced off   123");
    return;
  }

  // 3. 状态机逻辑分支
  switch (exec_state_) {
    case INIT: {
      // 初始状态：等待里程计和触发信号
      if (!have_odom_ || !trigger_) return;
      changeFSMExecState(WAIT_TARGET, "FSM");
      ROS_WARN("[switch]  wait for odom || trigger   123");
      break;
    }

    case WAIT_TARGET: {
      // 等待目标状态：如果收到目标且没被暂停，转去生成轨迹
      if (planner_forced_wait_) return;
      if (!have_target_) return;
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      ROS_WARN("[switch] from wait to gen_new_traj  123");
      break;
    }

    case GEN_NEW_TRAJ: {
      // 生成新轨迹状态
      start_pt_  = odom_pos_; // 起点为当前位置
      start_vel_ = odom_vel_; // 起始速度为当前速度
      start_acc_.setZero();   // 起始加速度设为0 (简化处理)

      // 计算起始 Yaw 角 (从四元数转换)
      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      // 调用后端规划器
      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM"); // 成功 -> 执行
        ROS_WARN("[switch] from GEN_NEW_TRAJ to EXEC_TRAJ  123");
      } else {
        // 失败 -> 保持当前状态重试 (或者可以切回 WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        ROS_WARN("[switch] from GEN_NEW_TRAJ to GEN_NEW_TRAJ  123");
      }
      break;
    }

    case EXEC_TRAJ: {
      // 执行轨迹状态：监控轨迹是否完成
      auto* info   = &planner_manager_->local_data_;
      ros::Time t0 = ros::Time::now();
      double t_cur = (t0 - info->start_time_).toSec();
      t_cur        = std::min(info->duration_, t_cur); // 防止时间越界

      // 如果当前时间超过了轨迹总时长 (减去微小余量)，认为到达
      if (t_cur > info->duration_ - 1e-2) {
        have_target_ = false; // 清除目标标志
        changeFSMExecState(WAIT_TARGET, "FSM"); // 切回等待
        ROS_WARN("[switch] from EXEC_TRAJ to WAIT_TARGET  time_out-reached");
      }
      break;
    }

    case REPLAN_TRAJ: {
      // (保留状态，暂不使用，逻辑合并在 EXEC_TRAJ + 定时器中)
      break;
    }
  }
}

// ==============================================================================================
// 4. 定时重规划回调 (例如 2Hz)
// ==============================================================================================
void KinoReplanFSM::replanTimerCallback(const ros::TimerEvent& e) {
  // 只有在 "正在执行" 且 "有目标" 且 "未暂停" 时才重规划
  if (planner_forced_wait_) return;
  if (!have_target_) return;
  if (exec_state_ != EXEC_TRAJ) return;

  // 重规划起点：当前状态
  start_pt_  = odom_pos_;
  start_acc_.setZero();

  // 速度处理：如果当前有速度，归一化到 0.1m/s
  // 目的：避免重规划时初始速度过大导致生成的轨迹过于激进或无解
  double vmag = odom_vel_.norm();
  if (vmag > 1e-6) start_vel_ = odom_vel_ * (0.1 / vmag);
  else start_vel_.setZero();

  // 姿态处理
  double roll, pitch, yaw;
  tf::Quaternion q(odom_orient_.x(), odom_orient_.y(), odom_orient_.z(), odom_orient_.w());
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  start_yaw_(0) = yaw;
  start_yaw_(1) = 0.0;
  start_yaw_(2) = 0.0;

  // 调用规划器
  bool success = callKinodynamicReplan();
  if (!success) {
    ROS_WARN_THROTTLE(1.0, "[ReplanTimer] Replan failed; will retry.");
  }
}

// ==============================================================================================
// 5. 碰撞检测与目标安全性修正 (包含修改后的线性回溯搜索)
// ==============================================================================================
void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  auto* info = &planner_manager_->local_data_;
  /* --- 第一部分：检查目标点 (End Point) 是否安全 --- */
  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    // 查询目标点距离障碍物的距离 (EDT)
    //是否处理动态障碍物
    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);
    ROS_WARN("distance_between_obstacle:%.2f",dist);    
    // double odom_dist = planner_manager_->pp_.dynamic_ ?
    //     edt_env->evaluateCoarseEDT(odom_pos_, info->duration_) :
    //     edt_env->evaluateCoarseEDT(odom_pos_, -1.0);
    // ROS_WARN("distance_between_obstacle_odom:%.2f",odom_dist);


    //安全阈值，非常需要修改
    // 如果距离小于安全阈值 (例如 0.3m)，说明目标在障碍物内或太近
    if (dist <= 0.05) {
      bool new_goal = false;
      Eigen::Vector3d goal = end_pt_; // 默认仍为原点
      // ROS_WARN("[replan]distance_between_obstacle:%.2f",dist) ;
      // -----------------------------------------------------------
      // 【修改点】线性回溯搜索 (Raycast Backwards)
      // 策略：从 不安全的目标点 向 当前无人机位置 连线回溯，
      //       寻找连线上第一个处于自由空间（安全）的点。
      // -----------------------------------------------------------

      // 1. 计算回溯方向向量 (从 终点 -> 起点)
      Eigen::Vector3d direction = (odom_pos_ - end_pt_).normalized();
      
      // 2. 计算最大搜索距离 (不能超过当前位置，否则就飞回来了)
      double max_search_dist = (odom_pos_ - end_pt_).norm();
      
      // 3. 搜索步长 (0.1m)
      double step_size = 0.1;
      
      ROS_WARN("8989000");
      // 4. 循环搜索
      for (double d = 0.0; d < max_search_dist; d += step_size) {
          // 计算连线上的候选点
          Eigen::Vector3d cand = end_pt_ + direction * d;
          ROS_WARN("[searching new goal]d:%.2f,cand:%.2f %.2f %.2f",d,cand.x(),cand.y(),cand.z());
          // 检查候选点的障碍物距离
          double cand_dist = planner_manager_->pp_.dynamic_ ?
              edt_env->evaluateCoarseEDT(cand, info->duration_) :
              edt_env->evaluateCoarseEDT(cand, -1.0);
              ROS_WARN("cand_dist:%.2f",cand_dist);

              
              

          // 如果距离大于安全阈值 (0.2m)，认为该点安全可用
          if (cand_dist >= 0.05) { 
              goal = cand;
              new_goal = true;
              ROS_WARN("[Collision]find a new goal");
              break; // 找到第一个安全点，立刻停止
          }
      }
      // -----------------------------------------------------------

      if (new_goal) {
        // cout << "Goal adjusted to free space along the line.1111111111111111111111111111111111111111111111" << endl;
        ROS_WARN("Goal adjusted to free space along the line.1111111111111111111111111111111111111111111111");

        end_pt_ = goal;     // 更新规划器的内部目标点
        end_vel_.setZero(); // 目标点速度设为0
        have_target_ = true;

        // 【新增】广播新的目标点
        // 作用：通知外部控制节点 (astra_control) 目标点已变更为安全点
        geometry_msgs::PoseStamped p;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "world"; // 注意 Frame ID 需与系统一致
        p.pose.position.x = goal.x();
        p.pose.position.y = goal.y();
        p.pose.position.z = goal.z();
        p.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        new_goal_pub_.publish(p);//发送新的目标点到astra_control

        // 可视化新的目标
        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
        
        // 此处不立即调用重规划，而是等待下一个 replanTimer 周期自动处理
        // 这样可以避免在回调中进行耗时计算
      } else {
        // 回溯整条线都没找到安全点 (极其罕见，除非整条路都被堵死)
        cout << "No valid goal found along the path, waiting..." << endl;
      }
    }
  }

  /* --- 第二部分：检查当前轨迹是否即将碰撞 --- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    // 检查当前 B-spline 轨迹的安全性
    bool safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      ROS_WARN("current traj in collision.");
      // 同样不立即触发，依赖 replanTimer 的高频重规划来避障
      //先测会不会撞11111111111111111111111111
    }
  }
}

// ==============================================================================================
// 6. 调用底层规划管理器
// ==============================================================================================
bool KinoReplanFSM::callKinodynamicReplan() {
  // 调用 FastPlannerManager 的核心接口：kinodynamicReplan
  // 输入：起点(P,V,A) 和 终点(P,V)
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success) {
    // 规划成功后，规划 Yaw 角轨迹 (平滑转向)
    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;

    /* --- 打包 B-spline 消息并发布 --- */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    // 填充控制点 (位置)
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    // 填充节点向量
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    // 填充控制点 (Yaw)
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    // 发布给 traj_server 执行
    bspline_pub_.publish(bspline);

    /* --- 可视化 --- */
    auto plan_data = &planner_manager_->plan_data_;
    // 在 Rviz 中画出规划出的几何路径 (Kinodynamic A* 的结果)
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

}  // namespace fast_planner