#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>   // 一定要有
using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, RETURN };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  bool return_home_;
  Eigen::Vector3d init_pos2return;
  int callHomingPlanner();
  bool auto_home_mode = false;
  bool init_pos_recorded_ = false; 
  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  ros::Publisher finish_pub_;
  ros::Subscriber trigger_home_sub_;
  // clients/sub
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient arming_client_;
  ros::Subscriber extended_state_sub_;

  // MAVROS state (ARM / mode)
  ros::Subscriber mav_state_sub_;
  mavros_msgs::State mav_state_;
  bool have_mav_state_ = false;

  // Auto-start exploration (after OFFBOARD+ARM and reaching target height)
  bool auto_start_explore_ = false;
  double auto_start_height_ = 1.2;      // start exploration when z >= this (odom/world)
  double auto_start_z_tol_ = 0.2;      // tolerance for reaching target height
  double auto_start_hold_sec_ = 1.0;    // keep above target for this duration
  bool auto_start_triggered_ = false;   // ensure only once
  ros::Time auto_start_reached_since_;  // for debounce/hold

  // landing states
  bool land_triggered_ = false;
  bool disarm_triggered_ = false;
  uint8_t landed_state_ = mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;


  void triggerHomeCallback(const std_msgs::BoolConstPtr& msg);

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void visualize();
  void clearVisMarker();
  bool triggerLandOnce();
  void tryDisarmOnceIfLanded();
  void extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr& msg);
  void mavStateCallback(const mavros_msgs::StateConstPtr& msg);

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif