/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou...
* GNU LGPL v3
*/

#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;
using std::string;

namespace fast_planner {

class Test {
private:
  /* data */
  int test_;
  std::vector<int> test_vec_;
  ros::NodeHandle nh_;

public:
  Test(const int& v) { test_ = v; }
  Test(ros::NodeHandle& node) { nh_ = node; }
  ~Test() {}
  void print() { std::cout << "test: " << test_ << std::endl; }
};

class KinoReplanFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;  // legacy thresholds (kept for compatibility)

  // ===== 新增：最大路点数，与 .cpp 中的 MAX_WAYPOINTS 保持一致 =====
  static constexpr int MAX_WAYPOINTS = 200;
  double waypoints_[MAX_WAYPOINTS][3];
  int waypoint_num_;

  // ===== 新增：显式重规划频率（Hz） =====
  double replan_hz_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                              // target state
  int current_wp_;

  // ===== 新增：/planner/off 强制等待开关 =====
  bool planner_forced_wait_{false};

  /* ROS utils */
  ros::NodeHandle node_;

  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_;
  // ===== 新增：定时重规划定时器 =====
  ros::Timer replan_timer_;

  ros::Subscriber waypoint_sub_, odom_sub_;
  // ===== 修正：/planner/off 是 Subscriber，不是 Publisher =====
  ros::Subscriber off_sub_;

  ros::Publisher replan_pub_, new_pub_, bspline_pub_;
  // 由 planner 主动调整终点时的广播
  ros::Publisher new_goal_pub_;

  /* helper functions */
  bool callKinodynamicReplan();        // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent& e);
  void checkCollisionCallback(const ros::TimerEvent& e);
  void waypointCallback(const geometry_msgs::PoseStamped msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  // ===== 新增回调声明：匹配 .cpp =====
  void plannerOffCallback(const std_msgs::BoolConstPtr& msg);
  void replanTimerCallback(const ros::TimerEvent& e);

public:
  KinoReplanFSM(/* args */) {}
  ~KinoReplanFSM() {}

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
