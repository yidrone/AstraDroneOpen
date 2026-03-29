/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/
/**
 *  @file traj_server.cpp
 *  @author luli (luli.gptt@gmail.com)
 *  @brief 优化了原有planner的z轴策略以及yaw角规划策略等等，详见readme
 *  @version 0.2
 *  @date 5-16-2025
 */
#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "plan_manage/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <algorithm>
#include <tf/tf.h>

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;
double yaw_goal, current_yaw, max_distance = 3.0, target_dist = 0.2;
ros::Publisher pose_cmd_pub;
nav_msgs::Odometry odom;
Eigen::Vector3d odom_pos_;
Eigen::Vector3d goal_pos_;
quadrotor_msgs::PositionCommand cmd;
geometry_msgs::PoseStamped pose_cmd;//px4

// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = { 5.7, 5.7, 6.2 };
double vel_gain[3] = { 3.4, 3.4, 4.0 };

using fast_planner::NonUniformBspline;

bool receive_traj_ = false;
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd_, traj_real_;

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "camera_init";
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


void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "camera_init";
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

void bsplineCallback(plan_manage::BsplineConstPtr msg) {
  // parse pos traj
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

  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }

  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

void replanCallback(std_msgs::Empty msg) {
  /* reset duration */
  const double time_out = 0.01;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(std_msgs::Empty msg) {
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

void visCallback(const ros::TimerEvent& e) {
  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);
  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
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

void goalCallback(const geometry_msgs::PoseStamped msg)
{
  goal_pos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  // 计算终端速度，模长为 0.1，方向为收到的 yaw 角方向
  yaw_goal = tf::getYaw(msg.pose.orientation);
  if (std::isnan(yaw_goal)) yaw_goal = 0.0;
}

void cmdCallback(const ros::TimerEvent& e) {
  if (!receive_traj_) return;
  std::pair<double, double> yaw_yawdot(0, 0);

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

  Eigen::Vector3d pos = traj_[0].evaluateDeBoorT(best_t);
  Eigen::Vector3d vel = traj_[1].evaluateDeBoorT(best_t);
  Eigen::Vector3d acc = traj_[2].evaluateDeBoorT(best_t);

  double interpolated_yaw = interpolateYaw(current_yaw, yaw_goal, (goal_pos_ - odom_pos_).norm(), max_distance);
  // std::cout << "Interpolated Yaw: " << interpolated_yaw << std::endl;
  // yaw_yawdot = calculate_yaw(best_t, pos, time_now, time_last_);
  // std::cout<<"Start yaw = "<<traj_[3].evaluateDeBoorT(0)[0]<<std::endl;
  // std::cout<<"End yaw = "<<traj_[3].evaluateDeBoorT(traj_[3].getTimeSum())[0]<<std::endl;

  double yaw = traj_[3].evaluateDeBoorT(best_t)[0];
  double yawdot = traj_[4].evaluateDeBoorT(best_t)[0];

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "camera_init";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
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

  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);
  ros::Time time_now = ros::Time::now();
  pose_cmd.header.stamp = time_now;
  pose_cmd.header.frame_id = "camera_init";

  pose_cmd.pose.position.x = pos(0);
  pose_cmd.pose.position.y = pos(1);
  pose_cmd.pose.position.z = pos(2);

  pose_cmd.pose.orientation.x = 0.0;
  pose_cmd.pose.orientation.y = 0.0;
  pose_cmd.pose.orientation.z = sin(interpolated_yaw/2);
  pose_cmd.pose.orientation.w = cos(interpolated_yaw/2);
  // pose_cmd.pose.orientation.z = 0.0;
  // pose_cmd.pose.orientation.w = 1.0;
  pose_cmd_pub.publish(pose_cmd);

  // 可视化方向箭头
  Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // 存轨迹
  traj_cmd_.push_back(pos);
  if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);
  ros::Subscriber goal_sub = node.subscribe("/fastplanner/goal", 1, goalCallback);

  pose_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/fastplanner/setpoint_position/local", 50);//发给主控程序

  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/adjust_distance_yaw", max_distance, -1.0);
  nh.param("traj_server/target_dist", target_dist, -1.0);
  last_yaw_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
