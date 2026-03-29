/*
 * Fast-Planner traj_server.cpp — LQR/LQI 速度跟踪器版
 *
 * 主要改动：
 * 1) 用离散 LQR/LQI（带积分抗偏）的加速度调节替换原 MPC（维持接口不变）。
 * 2) DARE 只在启动时求解一次，运行期仅做矩阵乘法，计算量极小。
 * 3) 仍发布两路：/position_cmd (可视化/兼容) 与 /fastplanner/setpoint_velocity/cmd_vel（世界系速度）。
 * 4) 通过参数开关：traj_server/mpc_positon == 1 → 位置模式；0 → LQR 速度模式（保持与原逻辑兼容）。
 */

#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "plan_manage/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <algorithm>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sstream>
#include <iomanip>

using fast_planner::NonUniformBspline;
using Eigen::Matrix; using Eigen::Matrix3d; using Eigen::Vector3d; using Eigen::MatrixXd;

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;
ros::Publisher pose_cmd_pub; // position setpoint (px4)
ros::Publisher vel_cmd_pub;  // velocity setpoint (LQR 模式)

double yaw_goal, current_yaw, max_distance = 3.0, target_dist = 0.2;
int mpc_position_mode = 1;  // 1=位置模式；0=LQR 速度模式（兼容原参名）

nav_msgs::Odometry odom;
Eigen::Vector3d odom_pos_;
Eigen::Vector3d odom_vel_ = Eigen::Vector3d::Zero();
Eigen::Vector3d goal_pos_;
quadrotor_msgs::PositionCommand cmd;
geometry_msgs::PoseStamped pose_cmd; // px4

double pos_gain[3] = {5.7, 5.7, 6.2};
double vel_gain[3] = {3.4, 3.4, 4.0};

bool receive_traj_ = false;
std::vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

double last_yaw_;
double time_forward_;

std::vector<Eigen::Vector3d> traj_cmd_, traj_real_;

// --- debug helper ---
static inline std::string vecToStr(const Eigen::Vector3d& v, int p=3){
  std::ostringstream oss; oss.setf(std::ios::fixed); oss<<std::setprecision(p)
    <<"["<<v.x()<<","<<v.y()<<","<<v.z()<<"]"; return oss.str();
}


//===================== LQR / LQI 参数与实现 =====================
struct LQRParams {
  double dt;               // 控制步长（用于离散化与速度积分）
  double q_pos;            // 位置误差权重
  double q_vel;            // 速度误差权重
  double q_int;            // 积分项权重（=0 则退化为纯 LQR）
  double r_acc;            // 加速度代价
  double max_speed;        // 速度限幅 [m/s]
  double max_acc;          // 加速度限幅 [m/s^2]
  double max_yaw_rate;     // 航向角速度限幅 [rad/s]
  bool   use_integral;     // 是否启用积分抗偏
  double i_limit;          // 积分限幅（每轴）
} lqr_;

// LQI 状态：积分误差
Eigen::Vector3d i_err_ = Eigen::Vector3d::Zero();
ros::Time last_ctrl_time_;

// 离散系统矩阵（带积分增广）
// x = [p; v] (6x1), i = ∫(p - p_ref)dt (3x1)
// e = [ep; ev; i]，控制律 u = a_ref - K * e
Matrix<double, 9, 9> A_aug_;
Matrix<double, 9, 3> B_aug_;
Matrix<double, 3, 9> K_aug_;
bool lqr_ready_ = false;

Matrix<double, 9, 9> makeQaug(double qpos, double qvel, double qint){
  Matrix<double, 9, 9> Q = Matrix<double, 9, 9>::Zero();
  Q.block<3,3>(0,0) = qpos * Matrix3d::Identity();
  Q.block<3,3>(3,3) = qvel * Matrix3d::Identity();
  Q.block<3,3>(6,6) = qint * Matrix3d::Identity();
  return Q;
}

void buildAugmentedSystem(double dt){
  // 原始离散双积分器（x=[p;v], u=a）
  Matrix<double,6,6> A = Matrix<double,6,6>::Zero();
  A.block<3,3>(0,0) = Matrix3d::Identity();
  A.block<3,3>(0,3) = dt * Matrix3d::Identity();
  A.block<3,3>(3,3) = Matrix3d::Identity();

  Matrix<double,6,3> B = Matrix<double,6,3>::Zero();
  B.block<3,3>(0,0) = 0.5 * dt * dt * Matrix3d::Identity();
  B.block<3,3>(3,0) = dt * Matrix3d::Identity();

  // 增广误差动态：
  // e_p+ = e_p + dt e_v + 0.5 dt^2 u_tilde
  // e_v+ = e_v + dt u_tilde
  //  i+  = i  + dt e_p
  A_aug_.setZero(); B_aug_.setZero();
  // A_aug_ 对应 e 到 e 的线性项
  A_aug_.block<3,3>(0,0) = Matrix3d::Identity();
  A_aug_.block<3,3>(0,3) = dt * Matrix3d::Identity();
  A_aug_.block<3,3>(3,3) = Matrix3d::Identity();
  A_aug_.block<3,3>(6,6) = Matrix3d::Identity();
  A_aug_.block<3,3>(6,0) = dt * Matrix3d::Identity();
  // B_aug_ 对应 u_tilde（等价于控制输入）
  B_aug_.block<3,3>(0,0) = 0.5 * dt * dt * Matrix3d::Identity();
  B_aug_.block<3,3>(3,0) = dt * Matrix3d::Identity();
}

// 简单的离散 Riccati 迭代（DARE），只在启动时求一次
Matrix<double, 3, 9> dlqr(const Matrix<double,9,9>& A, const Matrix<double,9,3>& B,
                          const Matrix<double,9,9>& Q, const Matrix3d& R){
  Matrix<double,9,9> P = Q;
  for(int i=0;i<500;++i){
    Matrix3d BtPB = B.transpose()*P*B;
    Matrix3d S = R + BtPB;
    Matrix<double,3,9> Ktmp = S.ldlt().solve(B.transpose()*P*A);
    Matrix<double,9,9> Pn = A.transpose()*P*A - A.transpose()*P*B*Ktmp + Q;
    if ((Pn-P).norm() < 1e-7) { P = Pn; break; }
    P = Pn;
  }
  Matrix3d S = R + B.transpose()*P*B;
  Matrix<double,3,9> K = S.ldlt().solve(B.transpose()*P*A);
  return K;
}

inline double clamp(double x, double lo, double hi){ return std::max(lo, std::min(hi, x)); }

// 角度归一化
double normalizeYaw(double yaw){
  while (yaw > M_PI) yaw -= 2*M_PI;
  while (yaw < -M_PI) yaw += 2*M_PI;
  return yaw;
}

double interpolateYaw(double current_yaw, double target_yaw, double distance_, double max_distance, double k=2.0){
  if(distance_ >= max_distance) return current_yaw;
  current_yaw = normalizeYaw(current_yaw);
  target_yaw  = normalizeYaw(target_yaw);
  double yaw_diff = normalizeYaw(target_yaw - current_yaw);
  double normalized_dist = 1.0 - distance_ / max_distance;
  double factor = std::pow(normalized_dist, k);
  double interpolated_yaw = normalizeYaw(current_yaw + yaw_diff * factor);
  double max_step = 0.1;
  double step = normalizeYaw(interpolated_yaw - current_yaw);
  if (std::fabs(step) > max_step) step = (step>0?max_step:-max_step);
  return normalizeYaw(current_yaw + step);
}

//========================== 可视化辅助 ==========================
void displayTrajWithColor(std::vector<Eigen::Vector3d> path, double resolution,
                          Eigen::Vector4d color, int id){
  visualization_msgs::Marker mk;
  mk.header.frame_id = "camera_init";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0); mk.color.g = color(1); mk.color.b = color(2); mk.color.a = color(3);
  mk.scale.x = resolution; mk.scale.y = resolution; mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for(size_t i=0;i<path.size();++i){ pt.x=path[i](0); pt.y=path[i](1); pt.z=path[i](2); mk.points.push_back(pt);} 
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color){
  visualization_msgs::Marker mk;
  mk.header.frame_id = "camera_init";
  mk.header.stamp = ros::Time::now();
  mk.id = id; mk.type = visualization_msgs::Marker::ARROW; mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0; mk.scale.x = 0.1; mk.scale.y = 0.2; mk.scale.z = 0.3;
  geometry_msgs::Point p; p.x=pos(0); p.y=pos(1); p.z=pos(2); mk.points.push_back(p);
  p.x=pos(0)+vec(0); p.y=pos(1)+vec(1); p.z=pos(2)+vec(2); mk.points.push_back(p);
  mk.color.r=color(0); mk.color.g=color(1); mk.color.b=color(2); mk.color.a=color(3);
  cmd_vis_pub.publish(mk);
}

//========================== 回调逻辑 ============================
void bsplineCallback(plan_manage::BsplineConstPtr msg){
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(),3);
  Eigen::VectorXd knots(msg->knots.size());
  for(int i=0;i<(int)msg->knots.size();++i) knots(i)=msg->knots[i];
  for(int i=0;i<(int)msg->pos_pts.size();++i){
    pos_pts(i,0)=msg->pos_pts[i].x; pos_pts(i,1)=msg->pos_pts[i].y; pos_pts(i,2)=msg->pos_pts[i].z;
  }
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1); pos_traj.setKnot(knots);

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(),1);
  for(int i=0;i<(int)msg->yaw_pts.size();++i) yaw_pts(i,0)=msg->yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time; traj_id_ = msg->traj_id;
  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_duration_ = traj_[0].getTimeSum();
  receive_traj_ = true;
}

void replanCallback(std_msgs::Empty){
  const double time_out = 0.01; ros::Time now = ros::Time::now();
  double t_stop = (now - start_time_).toSec() + time_out; traj_duration_ = std::min(t_stop, traj_duration_);
}

void newCallback(std_msgs::Empty){ traj_cmd_.clear(); traj_real_.clear(); }

void odomCallbck(const nav_msgs::Odometry& msg){
  if (msg.child_frame_id=="X" || msg.child_frame_id=="O") return;
  odom = msg;
  odom_pos_ << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
  odom_vel_ << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
  current_yaw = tf::getYaw(msg.pose.pose.orientation);
  traj_real_.push_back(Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin()+1000);
}

void visCallback(const ros::TimerEvent&){ displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0,1,0,1), 2); }

void goalCallback(const geometry_msgs::PoseStamped msg){
  goal_pos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  yaw_goal = tf::getYaw(msg.pose.orientation); if (std::isnan(yaw_goal)) yaw_goal = 0.0;
}

// 计算一次 LQR 控制加速度（仅矩阵运算）
Eigen::Vector3d solveLQRAcel(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                             const Eigen::Vector3d& pos_ref, const Eigen::Vector3d& vel_ref,
                             const Eigen::Vector3d& acc_ref){
  Eigen::Vector3d ep = pos - pos_ref;
  Eigen::Vector3d ev = vel - vel_ref;
  // 积分误差在 cmdCallback 中按真实周期累计，这里直接读 i_err_
  Eigen::Matrix<double,9,1> e; e << ep, ev, i_err_;
  Eigen::Vector3d u_tilde = -(K_aug_ * e);
  Eigen::Vector3d u = acc_ref + u_tilde;
  if (u.norm() > lqr_.max_acc) u = u.normalized()*lqr_.max_acc;
  return u;
}

void cmdCallback(const ros::TimerEvent&){
  if(!receive_traj_ || !lqr_ready_) return;

  double step = 0.02; // 搜索最近目标点用
  double t_min = 0.0, t_max = traj_duration_;
  double best_t = t_max, max_valid_t = -1.0;
  Eigen::Vector3d goal_pos = traj_[0].evaluateDeBoorT(t_max);
  for(double t=t_min+step; t<=t_max; t+=step){
    Eigen::Vector3d cur_pos = traj_[0].evaluateDeBoorT(t);
    double dist = (cur_pos - odom_pos_).norm();
    if (dist - target_dist < 0.01){ if (t > max_valid_t){ max_valid_t = t; best_t = t; goal_pos = cur_pos; } }
  }
  if (max_valid_t < 0.0){ best_t = traj_duration_; goal_pos = traj_[0].evaluateDeBoorT(best_t); }

  Eigen::Vector3d pos = traj_[0].evaluateDeBoorT(best_t);
  Eigen::Vector3d vel = traj_[1].evaluateDeBoorT(best_t);
  Eigen::Vector3d acc = traj_[2].evaluateDeBoorT(best_t);

  double interpolated_yaw = interpolateYaw(current_yaw, yaw_goal, (goal_pos_ - odom_pos_).norm(), max_distance);
  double yaw   = traj_[3].evaluateDeBoorT(best_t)[0];
  double yawdot= traj_[4].evaluateDeBoorT(best_t)[0];

  // 1) 仍发布 PositionCommand（兼容/可视化）
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "camera_init";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.position.x=pos(0); cmd.position.y=pos(1); cmd.position.z=pos(2);
  cmd.velocity.x=vel(0); cmd.velocity.y=vel(1); cmd.velocity.z=vel(2);
  cmd.acceleration.x=acc(0); cmd.acceleration.y=acc(1); cmd.acceleration.z=acc(2);
  cmd.yaw = yaw; cmd.yaw_dot = yawdot; last_yaw_ = cmd.yaw; pos_cmd_pub.publish(cmd);

  ros::Time now = ros::Time::now();
  if (last_ctrl_time_.isZero()) last_ctrl_time_ = now;
  double dt_real = (now - last_ctrl_time_).toSec();
  if (dt_real <= 0.0) dt_real = lqr_.dt; // 兜底

  if (mpc_position_mode == 1){
    // 位置模式：交给主控
    pose_cmd.header.stamp = now; pose_cmd.header.frame_id = "camera_init";
    pose_cmd.pose.position.x = pos(0); pose_cmd.pose.position.y = pos(1); pose_cmd.pose.position.z = pos(2);
    pose_cmd.pose.orientation.x = 0.0; pose_cmd.pose.orientation.y = 0.0;
    pose_cmd.pose.orientation.z = sin(interpolated_yaw/2); pose_cmd.pose.orientation.w = cos(interpolated_yaw/2);
    pose_cmd_pub.publish(pose_cmd);
  }else{
    // LQR/LQI 速度模式
    // 积分误差更新（抗偏/抗外扰），使用真实周期，限幅防风up
    if (lqr_.use_integral){
      Eigen::Vector3d ep = odom_pos_ - pos; // e = current - ref
      i_err_ += dt_real * ep;
      for(int i=0;i<3;++i) i_err_[i] = clamp(i_err_[i], -lqr_.i_limit, lqr_.i_limit);
    }

    Eigen::Vector3d u_acc = solveLQRAcel(odom_pos_, odom_vel_, pos, vel, acc);
    Eigen::Vector3d v_cmd = odom_vel_ + u_acc * lqr_.dt; // 使用设计步长积分
    if (v_cmd.norm() > lqr_.max_speed) v_cmd = v_cmd.normalized()*lqr_.max_speed;

    geometry_msgs::TwistStamped tw;
    tw.header.stamp = now;

    // 只发布机体系(FLU)线速度：v_body = R_bw * v_world
    Eigen::Quaterniond q_wb(
      odom.pose.pose.orientation.w,
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z
    );
    Eigen::Matrix3d R_wb = q_wb.toRotationMatrix();
    Eigen::Vector3d v_body = R_wb.transpose() * v_cmd; // world->body

    tw.header.frame_id = "base_link"; // 机体系(FLU)
    tw.twist.linear.x = v_body.x();
    tw.twist.linear.y = v_body.y();
    tw.twist.linear.z = v_body.z();
    // 输出机体系(FLU)的偏航角速度 r_body
    // 使用 ZYX 欧拉角关系：r ≈ cos(roll)*cos(pitch)*yawdot （忽略 rolldot/thetadot）
    double roll, pitch, yaw_now;
    tf::Matrix3x3(tf::Quaternion(
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w)).getRPY(roll, pitch, yaw_now);

    double r_body = std::cos(roll) * std::cos(pitch) * yawdot;
    r_body = clamp(r_body, -lqr_.max_yaw_rate, lqr_.max_yaw_rate);

    tw.twist.angular.x = 0.0;
    tw.twist.angular.y = 0.0;
    tw.twist.angular.z = r_body;

    // ---- 调试信息：周期性打印（限速 5Hz） ----
    Eigen::Vector3d ep_dbg = odom_pos_ - pos;
    Eigen::Vector3d ev_dbg = odom_vel_ - vel;
    Eigen::Matrix<double,9,1> e_aug_dbg; e_aug_dbg << ep_dbg, ev_dbg, i_err_;
    Eigen::Vector3d u_tilde_dbg = -(K_aug_ * e_aug_dbg);
    Eigen::Vector3d u_pre_dbg = acc + u_tilde_dbg; // 限幅前
    std::ostringstream dbg; dbg.setf(std::ios::fixed);
    dbg << "[LQR]"
        << "t=" << std::setprecision(2) << best_t
        << "  dt=" << std::setprecision(3) << dt_real << "\n"
        << "ref  p=" << vecToStr(pos,3)
        << " current  p=" << vecToStr(odom_pos_,3)<< "\n"
        << "  v="   << vecToStr(vel,3)
        << " current  v="   << vecToStr(odom_vel_,3)
        << "  a="   << vecToStr(acc,3)
        << "  |a|=" << acc.norm() << "\n"
        << "err  ep=" << vecToStr(ep_dbg,3) << "  |ep|=" << ep_dbg.norm()
        << "  ev="   << vecToStr(ev_dbg,3) << "  |ev|=" << ev_dbg.norm()
        << "  i="    << vecToStr(i_err_,3)  << "  |i|="  << i_err_.norm() << "\n"
        << "cmd  u_pre=" << vecToStr(u_pre_dbg,3) << "  |u_pre|=" << u_pre_dbg.norm()
        << "  u="       << vecToStr(u_acc,3)     << "  |u|="     << u_acc.norm() << "\n"
        << "vel  world=" << vecToStr(v_cmd,3) << "  |v|=" << v_cmd.norm()
        << "  body="     << vecToStr(v_body,3) << "\n"
        << "omega r_body=" << std::setprecision(3) << r_body
        << "  yawdot_ref=" << std::setprecision(3) << yawdot;
    ROS_INFO_STREAM_THROTTLE(0.2, dbg.str());

    vel_cmd_pub.publish(tw);
  }
  // 可视化方向箭头（基于轨迹 yaw）
  Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  drawCmd(pos, 2*dir, 2, Eigen::Vector4d(1,1,0,0.7));
  traj_cmd_.push_back(pos); if (traj_cmd_.size()>10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin()+1000);
  last_ctrl_time_ = now;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node; ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub  = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub     = node.subscribe("planning/new", 10, newCallback);
  ros::Subscriber odom_sub    = node.subscribe("/odom_world", 50, odomCallbck);
  ros::Subscriber goal_sub    = node.subscribe("/fastplanner/goal", 1, goalCallback);

  pose_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/fastplanner/setpoint_position/local", 50);
  vel_cmd_pub  = nh.advertise<geometry_msgs::TwistStamped>("/fastplanner/setpoint_velocity/cmd_vel", 50);
  cmd_vis_pub  = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub  = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_pub     = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  // 兼容 PositionCommand 的增益
  cmd.kx[0]=pos_gain[0]; cmd.kx[1]=pos_gain[1]; cmd.kx[2]=pos_gain[2];
  cmd.kv[0]=vel_gain[0]; cmd.kv[1]=vel_gain[1]; cmd.kv[2]=vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/adjust_distance_yaw", max_distance, -1.0);
  nh.param("traj_server/target_dist", target_dist, -1.0);
  nh.param("traj_server/mpc_positon", mpc_position_mode, 1); // 1: 位置；0: LQR

  //=========== LQR/LQI 参数 ===========
  nh.param("traj_server/lqr/dt",            lqr_.dt,          0.05);
  nh.param("traj_server/lqr/q_pos",         lqr_.q_pos,       10.0);
  nh.param("traj_server/lqr/q_vel",         lqr_.q_vel,        2.0);
  nh.param("traj_server/lqr/q_int",         lqr_.q_int,        3.0);
  nh.param("traj_server/lqr/r_acc",         lqr_.r_acc,        0.25);
  nh.param("traj_server/lqr/max_speed",     lqr_.max_speed,    2.0);
  nh.param("traj_server/lqr/max_acc",       lqr_.max_acc,      3.0);
  nh.param("traj_server/lqr/max_yaw_rate",  lqr_.max_yaw_rate, 1.0);
  nh.param("traj_server/lqr/use_integral",  lqr_.use_integral, true);
  nh.param("traj_server/lqr/i_limit",       lqr_.i_limit,      1.0);

  // 预构建增广系统并求一次 DLQR 增益
  buildAugmentedSystem(lqr_.dt);
  Matrix<double,9,9> Qaug = makeQaug(lqr_.q_pos, lqr_.q_vel, (lqr_.use_integral? lqr_.q_int : 0.0));
  Matrix3d R = lqr_.r_acc * Matrix3d::Identity();
  K_aug_ = dlqr(A_aug_, B_aug_, Qaug, R);
// 打印一次增益与权重，方便对比调参
Matrix3d Kp = K_aug_.block<3,3>(0,0);
Matrix3d Kv = K_aug_.block<3,3>(0,3);
Matrix3d Ki = K_aug_.block<3,3>(0,6);
ROS_WARN_STREAM("[LQR] dt="<<lqr_.dt<<"  Qp="<<lqr_.q_pos<<"  Qv="<<lqr_.q_vel
                <<"  Qi="<<(lqr_.use_integral? lqr_.q_int : 0.0)
                <<"  R="<<lqr_.r_acc);
ROS_WARN_STREAM("[LQR] ||Kp||="<<Kp.norm()<<"  ||Kv||="<<Kv.norm()<<"  ||Ki||="<<Ki.norm());
lqr_ready_ = true;

  last_yaw_ = 0.0; ros::Duration(0.5).sleep();
  ROS_WARN("[Traj server]: ready. mode=%s (LQR vel when 0)", (mpc_position_mode==1?"position":"LQR-velocity"));
  ros::spin();
  return 0;
}
