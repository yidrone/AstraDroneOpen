#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "bspline/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <poly_traj/polynomial_traj.h>
#include <active_perception/perception_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <plan_manage/backward.hpp>

namespace backward {
backward::SignalHandling sh;
}
using fast_planner::NonUniformBspline;
using fast_planner::Polynomial;
using fast_planner::PolynomialTraj;
using fast_planner::PerceptionUtils;

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;
nav_msgs::Odometry odom;
quadrotor_msgs::PositionCommand cmd;

// use position control ... 
bool use_controller = true;
ros::Publisher pose_cmd_pub; //px4_position control
geometry_msgs::PoseStamped pose_cmd;
bool init_pos_tag;
Eigen::Vector3d init_pos;
double forward_dt = 0.2;

// vector<NonUniformBspline> 一堆B样条轨迹
// Info of generated traj
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
int pub_traj_id_;

shared_ptr<PerceptionUtils> percep_utils_;

// Info of replan
bool receive_traj_ = false;
double replan_time_;

bool disable_yaw = false;

// 用于可视化&统计的轨迹数据： traj_cmd_在每次发布控制点后保存 | traj_real_在每次里程计回调时保存
// Executed traj, commanded and real ones
vector<Eigen::Vector3d> traj_cmd_, traj_real_;

// Data for benchmark comparison
ros::Time start_time, end_time, last_time;
double energy;

double yaw_goal, current_yaw, max_distance = 3.0, target_dist = 0.2;
Eigen::Vector3d odom_pos_;
Eigen::Vector3d goal_pos_;


// Loop correction
Eigen::Matrix3d R_loop;
Eigen::Vector3d T_loop;
bool isLoopCorrection = false;

double calcPathLength(const vector<Eigen::Vector3d>& path) {
  if (path.empty()) return 0;
  double len = 0.0;
  for (int i = 0; i < path.size() - 1; ++i) {
    len += (path[i + 1] - path[i]).norm();
  }
  return len;
}

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawFOV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  cmd_vis_pub.publish(mk);

  if (list1.size() == 0) return;

  // Pub new marker
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;
  cmd_vis_pub.publish(mk);
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void replanCallback(std_msgs::Empty msg) {
  // Informed of new replan, end the current traj after some time
  const double time_out = 0.3;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out + replan_time_;
  traj_duration_ = min(t_stop, traj_duration_);
}

// 用于可视化&统计对比的轨迹数据： 其对于探索过程中 轨迹的执行没有任何影响
// traj_cmd_在每次发布控制点后保存 | traj_real_在每次里程计回调时保存
void newCallback(std_msgs::Empty msg) {
  // Clear the executed traj data
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;
  odom = msg;

  odom_pos_ << msg.pose.pose.position.x,
               msg.pose.pose.position.y,
               msg.pose.pose.position.z;

  current_yaw = tf::getYaw(msg.pose.pose.orientation);

  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void pgTVioCallback(geometry_msgs::Pose msg) {
  // World to odom
  Eigen::Quaterniond q =
      Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  R_loop = q.toRotationMatrix();
  T_loop << msg.position.x, msg.position.y, msg.position.z;

  // cout << "R_loop: " << R_loop << endl;
  // cout << "T_loop: " << T_loop << endl;
}

void visCallback(const ros::TimerEvent& e) {
  // Draw the executed traj (desired state)
  // displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(1, 0, 0, 1), pub_traj_id_);
  // displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), pub_traj_id_);
  displayTrajWithColor(traj_cmd_, 0.2, Eigen::Vector4d(0, 0, 1, 1), pub_traj_id_);

  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);
}

// 接收`exploration_manager`发送的数据，使用非均匀 B 样条曲线创建一条包含位置和偏航角的轨迹
// 激活cmdCallback
void bsplineCallback(const bspline::BsplineConstPtr& msg) {
  // Received traj should have ascending traj_id  ascending-上升
  // 接收到的轨迹id需要递增
  if (msg->traj_id <= traj_id_) {
    ROS_ERROR("out of order bspline.");
    return;
  }

  // Parse the msg
  // MatrixXd-动态矩阵(行，列) VectorXd-动态向量  动态-大小不固定
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }
  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }
  // 控制点 阶数 间隔时间 points order knot_span
  // 位置轨迹 阶数-传参、时间间隔 0.1
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  // 和前面的0.1关系？ 应该是[直接替换掉前面的0.1]
  pos_traj.setKnot(knots);

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i)
    yaw_pts(i, 0) = msg->yaw_pts[i];
  // yaw轨迹 固定3阶、时间间隔-传参
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  // 0-pos 1-d_pos(vel) 2-dd_pos(acc) 3-yaw 4-d_yaw 5-ddd_pos(jerk)
  traj_.clear();
  traj_.push_back(pos_traj);
  // .getDerivative() 用于获取给定轨迹对象的一阶导数
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  // 这里 traj_[2] 对应 jerk
  traj_.push_back(traj_[2].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  // 激活cmdCallback
  receive_traj_ = true;
  init_pos_tag = false;

  // Record the start time of flight
  if (start_time.isZero()) {
    ROS_WARN("start flight");
    start_time = ros::Time::now();
  }
}

// 归一化角度到 [-π, π]
double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2 * M_PI;
    while (yaw < -M_PI) yaw += 2 * M_PI;
    return yaw;
}


double interpolateYaw(double current_yaw, double target_yaw, double distance_, double max_distance, double k = 2.0) {

    // 距离大于最大距离，直接返回当前yaw
    if (distance_ >= max_distance) {
        return current_yaw;
    }

    // 归一化当前yaw和目标yaw到 [-π, π]
    current_yaw = normalizeYaw(current_yaw);
    target_yaw = normalizeYaw(target_yaw);

    // 计算角度差，并将其限制在 [-π, π] 范围内
    double yaw_diff = target_yaw - current_yaw;
    yaw_diff = normalizeYaw(yaw_diff);

    // 平滑插值：计算插值因子，值域为 [0, 1]
    double normalized_dist = 1.0 - distance_ / max_distance;
    double factor = std::pow(normalized_dist, k);
    // std::cout << "factor = " << factor << std::endl;

    // 插值计算 yaw 角
    double interpolated_yaw = current_yaw + yaw_diff * factor;

    // 插值后的 yaw 再次归一化，防止超过 [-π, π]
    interpolated_yaw = normalizeYaw(interpolated_yaw);

    // 限制单次插值的变化量不超过 0.2
    double max_step = 0.1;
    double step = interpolated_yaw - current_yaw;
    step = normalizeYaw(step);

    if (std::fabs(step) > max_step) {
        step = (step > 0 ? max_step : -max_step);
    }

    // 计算限制后的插值结果
    double limited_yaw = current_yaw + step;
    return normalizeYaw(limited_yaw);
}

// 1）插值计算B样条轨迹上的点
// 2）发布控制指令（点）
void cmdCallback(const ros::TimerEvent& e) {
  // No publishing before receive traj data
  // if (!receive_traj_) return;

  if (!receive_traj_ && !init_pos_tag) return;

    // 未收到轨迹前, 发布基本的定点
  if(init_pos_tag && !receive_traj_){
    pose_cmd.header.stamp = ros::Time::now();
    pose_cmd.header.frame_id = "world";

    pose_cmd.pose.position.x = init_pos(0);
    pose_cmd.pose.position.y = init_pos(1);
    pose_cmd.pose.position.z = init_pos(2);

    pose_cmd.pose.orientation.x = 0.0;
    pose_cmd.pose.orientation.y = 0.0;
    pose_cmd.pose.orientation.z = 0.0;
    pose_cmd.pose.orientation.w = 1.0;

    pose_cmd_pub.publish(pose_cmd);

    ROS_WARN_THROTTLE(2.0, "Waiting for trigger, set raw point for offboard");
  }else{
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();
    Eigen::Vector3d pos, vel, acc, jer;
    double yaw, yawdot;

    // 1）插值计算 t_cur 时刻B样条轨迹上的点
    if (t_cur < traj_duration_ && t_cur >= 0.0) {
      // Current time within range of planned traj
      // evaluateDeBoorT： 递归地计算 t_cur 时刻B样条曲线上的点
      // 在计算插值点时会利用控制点、节点矢量和度数，以递归的方式逐步计算出曲线上的点
      // 插值计算 t_cur 时刻B样条轨迹上的点
      pos = traj_[0].evaluateDeBoorT(t_cur);
      vel = traj_[1].evaluateDeBoorT(t_cur);
      acc = traj_[2].evaluateDeBoorT(t_cur);
      yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
      yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
      jer = traj_[5].evaluateDeBoorT(t_cur);
    } else if (t_cur >= traj_duration_) {
      // Current time exceed range of planned traj
      // keep publishing the final position and yaw
      pos = traj_[0].evaluateDeBoorT(traj_duration_);
      vel.setZero();
      acc.setZero();
      yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
      yawdot = 0.0;

      // Report info of the whole flight
      double len = calcPathLength(traj_cmd_);
      double flight_t = (end_time - start_time).toSec();
      ROS_WARN_THROTTLE(2, "flight time: %lf, path length: %lf, mean vel: %lf, energy is: % lf ", flight_t,
                        len, len / flight_t, energy);
    } else {
      cout << "[Traj server]: invalid time." << endl;
    }

    if (isLoopCorrection) {
      // .transpose() 矩阵的转置
      pos = R_loop.transpose() * (pos - T_loop);
      vel = R_loop.transpose() * vel;
      acc = R_loop.transpose() * acc;

      Eigen::Vector3d yaw_dir(cos(yaw), sin(yaw), 0);
      yaw_dir = R_loop.transpose() * yaw_dir;
      yaw = atan2(yaw_dir[1], yaw_dir[0]);
    }

    if(use_controller){
      // 2）发布控制指令（点）
      cmd.header.stamp = time_now;
      cmd.trajectory_id = traj_id_;
      cmd.position.x = pos(0);
      cmd.position.y = pos(1);
      cmd.position.z = pos(2);
      cmd.velocity.x = vel(0);
      cmd.velocity.y = vel(1);
      cmd.velocity.z = vel(2);
      cmd.acceleration.x = acc(0);
      cmd.acceleration.y = acc(1);
      cmd.acceleration.z = acc(2);
      // cmd.yaw = yaw;
      // cmd.yaw_dot = yawdot;

      // default use yaw
      if(!disable_yaw){
        cmd.yaw = yaw;
        cmd.yaw_dot = yawdot;
      }else{
        cmd.yaw = 0.0;
        cmd.yaw_dot = 0.0;
      }

      pos_cmd_pub.publish(cmd);
    }else{

      double dt = 0.2;
      pose_cmd.header.stamp = time_now;
      pose_cmd.header.frame_id = "world";

      pose_cmd.pose.position.x = pos(0)+dt*vel(0);
      pose_cmd.pose.position.y = pos(1)+dt*vel(1);
      pose_cmd.pose.position.z = pos(2)+dt*vel(2);

      if(!disable_yaw){
        pose_cmd.pose.orientation.x = 0.0;
        pose_cmd.pose.orientation.y = 0.0;
        pose_cmd.pose.orientation.z = sin(yaw/2);
        pose_cmd.pose.orientation.w = cos(yaw/2);
      }else{
        pose_cmd.pose.orientation.x = 0.0;
        pose_cmd.pose.orientation.y = 0.0;
        pose_cmd.pose.orientation.z = 0.0;
        pose_cmd.pose.orientation.w = 1.0;
      }

      pose_cmd_pub.publish(pose_cmd);
    }

    percep_utils_->setPose(pos, yaw);
    vector<Eigen::Vector3d> l1, l2;
    percep_utils_->getFOV(l1, l2);
    drawFOV(l1, l2);

    // Record info of the executed traj
    if (traj_cmd_.size() == 0) {
      // Add the first position
      traj_cmd_.push_back(pos);
    } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
      // Add new different commanded position
      traj_cmd_.push_back(pos);
      double dt = (time_now - last_time).toSec();
      energy += jer.squaredNorm() * dt;
      end_time = ros::Time::now();
    }
    last_time = time_now;

  }

  // if (traj_cmd_.size() > 100000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

// 1）插值计算B样条轨迹上的点
// 2）发布控制指令（点）
void cmdCallbacknew(const ros::TimerEvent& e) {
  // No publishing before receive traj data
  // if (!receive_traj_) return;

  if (!receive_traj_ && !init_pos_tag) return;

    // 未收到轨迹前, 发布基本的定点
  if(init_pos_tag && !receive_traj_){
    pose_cmd.header.stamp = ros::Time::now();
    pose_cmd.header.frame_id = "world";

    pose_cmd.pose.position.x = init_pos(0);
    pose_cmd.pose.position.y = init_pos(1);
    pose_cmd.pose.position.z = init_pos(2);

    pose_cmd.pose.orientation.x = 0.0;
    pose_cmd.pose.orientation.y = 0.0;
    pose_cmd.pose.orientation.z = 0.0;
    pose_cmd.pose.orientation.w = 1.0;

    pose_cmd_pub.publish(pose_cmd);

    ROS_WARN_THROTTLE(2.0, "Waiting for trigger, set raw point for offboard");
  }else{
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();
    Eigen::Vector3d pos, vel, acc, jer;
    double yaw, yawdot;

    double step = 0.02;
    double t_min = 0.0;
    double t_max = traj_duration_;

    double best_t = t_max;  // 默认选最后一个时间点
    Eigen::Vector3d goal_pos = traj_[0].evaluateDeBoorT(t_max);  // 默认目标点
    double max_valid_t = -1.0;

    for (double t = t_min + step; t <= t_max; t += step) {
      Eigen::Vector3d cur_pos = traj_[0].evaluateDeBoorT(t);
      double dist = (cur_pos - odom_pos_).norm();
      
      if (dist - target_dist < 0.01) {
        // 更新为时间更大的点
        if (t > max_valid_t) {
          max_valid_t = t;
          best_t = t;
          goal_pos = cur_pos;
        }
      }
    }

    // fallback：若全段都未满足距离，则仍使用终点
    if (max_valid_t < 0.0) {
      best_t = traj_duration_;
      goal_pos = traj_[0].evaluateDeBoorT(best_t);
    }

    pos = traj_[0].evaluateDeBoorT(best_t);
    vel = traj_[1].evaluateDeBoorT(best_t);
    acc = traj_[2].evaluateDeBoorT(best_t);
    yaw = traj_[3].evaluateDeBoorT(best_t)[0];
    yawdot = traj_[4].evaluateDeBoorT(best_t)[0];

    yaw_goal = yaw;
    double interpolated_yaw = interpolateYaw(current_yaw, yaw_goal, (goal_pos_ - odom_pos_).norm(), max_distance);

    // 1）插值计算 t_cur 时刻B样条轨迹上的点
    if (t_cur < traj_duration_ && t_cur >= 0.0) {
      // Current time within range of planned traj
      // evaluateDeBoorT： 递归地计算 t_cur 时刻B样条曲线上的点
      // 在计算插值点时会利用控制点、节点矢量和度数，以递归的方式逐步计算出曲线上的点
      // 插值计算 t_cur 时刻B样条轨迹上的点
      // pos = traj_[0].evaluateDeBoorT(t_cur);
      // vel = traj_[1].evaluateDeBoorT(t_cur);
      // acc = traj_[2].evaluateDeBoorT(t_cur);
      // yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
      // yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
      jer = traj_[5].evaluateDeBoorT(t_cur);
    } else if (t_cur >= traj_duration_) {
      // Current time exceed range of planned traj
      // keep publishing the final position and yaw
      // pos = traj_[0].evaluateDeBoorT(traj_duration_);

      vel.setZero();
      acc.setZero();
      // yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
      yawdot = 0.0;

      // Report info of the whole flight
      double len = calcPathLength(traj_cmd_);
      double flight_t = (end_time - start_time).toSec();
      ROS_WARN_THROTTLE(2, "flight time: %lf, path length: %lf, mean vel: %lf, energy is: % lf ", flight_t,
                        len, len / flight_t, energy);
    } else {
      cout << "[Traj server]: invalid time." << endl;
    }

    if(use_controller){
      // 2）发布控制指令（点）
      cmd.header.stamp = time_now;
      cmd.trajectory_id = traj_id_;
      cmd.position.x = pos(0);
      cmd.position.y = pos(1);
      cmd.position.z = pos(2);
      cmd.velocity.x = vel(0);
      cmd.velocity.y = vel(1);
      cmd.velocity.z = vel(2);
      cmd.acceleration.x = acc(0);
      cmd.acceleration.y = acc(1);
      cmd.acceleration.z = acc(2);
      // cmd.yaw = yaw;
      // cmd.yaw_dot = yawdot;

      // default use yaw
      if(!disable_yaw){
        cmd.yaw = yaw;
        cmd.yaw_dot = yawdot;
      }else{
        cmd.yaw = 0.0;
        cmd.yaw_dot = 0.0;
      }

      pos_cmd_pub.publish(cmd);
    }else{

      double dt = 0.2;
      pose_cmd.header.stamp = time_now;
      pose_cmd.header.frame_id = "world";

      pose_cmd.pose.position.x = pos(0);
      pose_cmd.pose.position.y = pos(1);
      pose_cmd.pose.position.z = pos(2);

      if(!disable_yaw){
        pose_cmd.pose.orientation.x = 0.0;
        pose_cmd.pose.orientation.y = 0.0;
        pose_cmd.pose.orientation.z = sin(interpolated_yaw/2);
        pose_cmd.pose.orientation.w = cos(interpolated_yaw/2);
      }else{
        pose_cmd.pose.orientation.x = 0.0;
        pose_cmd.pose.orientation.y = 0.0;
        pose_cmd.pose.orientation.z = 0.0;
        pose_cmd.pose.orientation.w = 1.0;
      }

      pose_cmd_pub.publish(pose_cmd);
    }

    percep_utils_->setPose(pos, yaw);
    vector<Eigen::Vector3d> l1, l2;
    percep_utils_->getFOV(l1, l2);
    drawFOV(l1, l2);

    // Record info of the executed traj
    if (traj_cmd_.size() == 0) {
      // Add the first position
      traj_cmd_.push_back(pos);
    } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
      // Add new different commanded position
      traj_cmd_.push_back(pos);
      double dt = (time_now - last_time).toSec();
      energy += jer.squaredNorm() * dt;
      end_time = ros::Time::now();
    }
    last_time = time_now;

  }

  // if (traj_cmd_.size() > 100000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  // ****************** 订阅 *********************
  // 接收B样条轨迹 点+控制点 bspline::Bspline
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  // trigger： std_msgs::Empty  replan-flag  一段时间后结束当前轨迹
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  // 清除执行的轨迹数据-trigger：std_msgs::Empty -- 没用
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
  // uav odom  traj_real_.push_back
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);
  // geometry_msgs::Pose  T_loop R_loop
  ros::Subscriber pg_T_vio_sub = node.subscribe("/loop_fusion/pg_T_vio", 10, pgTVioCallback);

  // ****************** 发布 *********************
  // 可视化
  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  // Uav control cmd
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  // 可视化
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  // position ... 
  pose_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 50); //px4 直接接收
  // 是否使用控制器
  nh.param("traj_server/use_controller", use_controller, true);
  // 不使用控制器需要程序发送初始位置控制指令
  if(!use_controller)
    init_pos_tag = 1;
  else
    init_pos_tag = 0;
  
  // 向前多少时间间隔
  nh.param("traj_server/forward_dt", forward_dt, 0.2);

  nh.param("traj_server/target_dist", target_dist, -1.0);
  nh.param("traj_server/adjust_distance_yaw", max_distance, -1.0);


  // task handle callback 
  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  // ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallbacknew);
  
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  nh.param("traj_server/pub_traj_id", pub_traj_id_, -1);
  nh.param("fsm/replan_time", replan_time_, 0.1);
  nh.param("loop_correction/isLoopCorrection", isLoopCorrection, false);

  nh.param("traj_server/init_x", init_pos[0], 0.0);
  nh.param("traj_server/init_y", init_pos[1], 0.0);
  nh.param("traj_server/init_z", init_pos[2], 0.0);

  // only for lidar
  nh.param("traj_server/disable_yaw", disable_yaw, false);

  ROS_WARN("[Traj server]: init...");
  ros::Duration(1.0).sleep();

  // Control parameter
  cmd.kx = { 5.7, 5.7, 6.2 };
  cmd.kv = { 3.4, 3.4, 4.0 };

  std::cout << start_time.toSec() << std::endl;
  std::cout << end_time.toSec() << std::endl;

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = init_pos[0];
  cmd.position.y = init_pos[1];
  cmd.position.z = init_pos[2];
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  cmd.yaw = 0.0;
  cmd.yaw_dot = 0.0;

  percep_utils_.reset(new PerceptionUtils(nh));

  // test();
  // Initialization for exploration, move upward and downward
  for (int i = 0; i < 100; ++i) {
    cmd.position.z += 0.01;
    pos_cmd_pub.publish(cmd);
    ros::Duration(0.01).sleep();
  }
  for (int i = 0; i < 100; ++i) {
    cmd.position.z -= 0.01;
    pos_cmd_pub.publish(cmd);
    ros::Duration(0.01).sleep();
  }
  // ros::Duration(1.0).sleep();
  // for (int i = 0; i < 100; ++i)
  // {
  //   cmd.position.x -= 0.01;
  //   pos_cmd_pub.publish(cmd);
  //   ros::Duration(0.01).sleep();
  // }

  R_loop = Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();
  T_loop = Eigen::Vector3d(0, 0, 0);

  ROS_WARN("[Traj server]: ready.");
  ros::spin();

  return 0;
}
