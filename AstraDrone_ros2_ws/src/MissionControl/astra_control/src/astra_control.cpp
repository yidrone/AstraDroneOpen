/**
 * @file astra_control.cpp
 * @author luli (luli.gptt@gmail.com)
 * @brief 本程序为无人机巡检流程控制程序，可以修改指定yaml，修改路径点
 * @version 0.2
 * @date 5-16-2025
 */
#include "astra_control/astra_control.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <memory>
#include <sstream>
#include <algorithm> // 添加 algorithm 头文件

using namespace std::chrono_literals;

// 全局变量
int times_detect = 0;
bool flag_takeoff_done = false;
bool first_call = true;
bool have_planner_cmd = false, flag_land = false, have_waypoint_mark = false, have_land_mark = false;
rclcpp::Time start_time;
geometry_msgs::msg::PoseStamped uav_pose;
geometry_msgs::msg::PoseStamped last_mavros_point_cmd;
Eigen::Vector3f takeoff_point;
Eigen::Vector3f last_pub_point = {0, 0, 0};
Eigen::Vector3f now_pub_point;
Eigen::Vector3f uav_newest_position;
Eigen::Vector4f adjust_target_position;

namespace astra_control {

// 实现缺失的函数
Eigen::Vector3f LLController::toEigen(const geometry_msgs::msg::Point& point) {
  return Eigen::Vector3f(point.x, point.y, point.z);
}

double LLController::distance3d(float x1, float y1, float z1, float x2, float y2, float z2) {
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

PointMode LLController::stringToPointmode(const std::string& str) {
  if (str == "Detect_point") return Detect_point;
  if (str == "Land_point") return Land_point;
  return Nothing_point; // 默认值
}

LLController::LLController() : Node("astra_control_node") {
  initializeNode();
  RCLCPP_INFO(this->get_logger(), "\033[47;30m ---------------------------------- Start mission ---------------------------------- \033[0m");
}

LLController::~LLController() {}

void LLController::initializeNode() {
  load_params();

  waypoint_now = -1;
  waypoint_next = 0;

  // 创建QoS配置对象
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);  // 设置为可靠传输

  // 订阅无人机当前位置 - 使用统一的QoS配置
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose_flu", qos,
    std::bind(&LLController::positionCallback, this, std::placeholders::_1));
  
  // 订阅fastplanner命令 - 使用相同的QoS
  fastplanner_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/fastplanner/setpoint_position/local", qos,
    std::bind(&LLController::plannercmdCallback, this, std::placeholders::_1));
  
  // 订阅检测到的航路点标记
  detect_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/detect/waypoint_mark", qos,
    std::bind(&LLController::waypointMarkCallback, this, std::placeholders::_1));
  
  // 订阅降落标记
  land_mark_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/detect/land_mark", qos,
    std::bind(&LLController::landMarkCallback, this, std::placeholders::_1));
  
  // 发布目标点 - 使用相同的可靠策略，但保持队列深度50
  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(50));
  pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  mavros_point_cmd_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/setpoint_position/local_flu", pub_qos);
  
  // 发布目标点给规划器 - 保持队列深度10
  auto planner_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  planner_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  setplanner_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/fastplanner/goal", planner_qos);
  
  
  // 创建服务客户端
  land_client = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
  set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
  
  // 创建定时器
  cmd_timer = this->create_wall_timer(
    50ms, std::bind(&LLController::cmdCallback, this));
}

void LLController::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  uav_pose = *msg;
  uav_newest_position = toEigen(msg->pose.position);
  
  if (!flag_takeoff_done) {
    if (debug) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "\033[34mDistance with Next Waypoint = %3lf m\033[0m", 
                          (uav_newest_position - takeoff_point).norm());
    }
    
    if ((uav_newest_position - takeoff_point).norm() < takeoff_threshould) {
      flag_takeoff_done = true;
      NextPoint();
      Drone_mode = Run_point;
    }
  } else {
    patrol();
  }
}

void LLController::patrol() {
  geometry_msgs::msg::PoseStamped next_position_msg;
  float dis_to_next_position = 0;
  double yaw;
  
  switch (Drone_mode) {
    case Run_point:
      dis_to_next_position = distance3d(
        uav_newest_position[0], uav_newest_position[1], uav_newest_position[2],
        waypoint_list[waypoint_next].x, waypoint_list[waypoint_next].y, waypoint_list[waypoint_next].z);
      
      next_position_msg.pose.position.x = waypoint_list[waypoint_next].x;
      next_position_msg.pose.position.y = waypoint_list[waypoint_next].y;
      next_position_msg.pose.position.z = waypoint_list[waypoint_next].z;
      yaw = waypoint_list[waypoint_next].yaw;
      next_position_msg.pose.orientation = createQuaternionMsgFromYaw(yaw);
      
      arrive_goal_threshould = waypoint_threshould;
      pub_goal(next_position_msg);
      break;
      
    case Aligning:
      dis_to_next_position = distance3d(
        uav_newest_position[0], uav_newest_position[1], uav_newest_position[2],
        adjust_target_position[0], adjust_target_position[1], adjust_target_position[2]);
      
      patrol_cmd.pose.position.x = adjust_target_position[0];
      patrol_cmd.pose.position.y = adjust_target_position[1];
      patrol_cmd.pose.position.z = adjust_target_position[2];
      
      if (std::isnan(adjust_target_position[3])) 
        yaw = waypoint_list[waypoint_next].yaw;
      else 
        yaw = adjust_target_position[3];
      
      patrol_cmd.pose.orientation = createQuaternionMsgFromYaw(yaw);
      arrive_goal_threshould = aligning_threshould;
      break;
      
    case Land:
      dis_to_next_position = distance3d(
        uav_newest_position[0], uav_newest_position[1], 0,
        adjust_target_position[0], adjust_target_position[1], 0);
      
      patrol_cmd.pose.position.x = adjust_target_position[0];
      patrol_cmd.pose.position.y = adjust_target_position[1];
      patrol_cmd.pose.position.z = land_height;
      
      if (std::isnan(adjust_target_position[3])) 
        yaw = waypoint_list[waypoint_next].yaw;
      else 
        yaw = adjust_target_position[3];
      
      patrol_cmd.pose.orientation = createQuaternionMsgFromYaw(yaw);
      
      if (flag_landing_detect) 
        arrive_goal_threshould = landing_threshould;
      else 
        arrive_goal_threshould = waypoint_threshould;
      break;
      
    default:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "\033[31m[Error]: unknown Drone_mode\033[0m");
      break;
  }
  
  if (debug && !flag_land) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "\033[34mDistance with Next Waypoint = %3lf m\033[0m", 
                        dis_to_next_position);
  }
  
  if (debug && flag_land) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "\033[34mHave Land!\033[0m");
  }
  
  double current_yaw = getYawFromQuaternion(uav_pose.pose.orientation);
  
  if (dis_to_next_position <= arrive_goal_threshould && 
      std::abs(current_yaw - yaw) <= arrive_yaw_threshould) {
    switch (Point_mode) {
      case Detect_point:
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "\033[34mArrive waypoint, and detect...\033[0m");
        Drone_mode = Aligning;
        
        if (WayPointDetectDone()) {
          NextPoint();
          times_detect = 0;
          have_waypoint_mark = false;
          Drone_mode = Run_point;
        }
        break;
        
      case Nothing_point:
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                            "\033[34mArrive goal position, and Pure run point.\033[0m");
        NextPoint();
        have_waypoint_mark = false;
        Drone_mode = Run_point;
        break;
        
      case Land_point:
        if (debug && !flag_land) {
          if (flag_landing_detect) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "\033[34mArrive landing position, start detect land mark...\033[0m");
          } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "\033[34mArrive landing position, and not detect.\033[0m");
          }
        }
        Drone_mode = Land;
        
        if (!flag_landing_detect || LandDetectDone()) {
          CallLand();
          have_land_mark = false;
          times_detect = 0;
        }
        break;
        
      default:
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "\033[31m[Error]: unknown Point_mode\033[0m");
        break;
    }
  }
}

void LLController::plannercmdCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  have_planner_cmd = true;
  planner_cmd = *msg;
}

void LLController::waypointMarkCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  have_waypoint_mark = true;
  waypoint_mark = *msg;
}

void LLController::landMarkCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  have_land_mark = true;
  land_mark = *msg;
}

bool LLController::isQuaternionNormalized(const geometry_msgs::msg::Quaternion& q, double tolerance) {
  double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::abs(norm - 1.0) < tolerance;
}

void LLController::cmdCallback() {
  if (!isQuaternionNormalized(uav_pose.pose.orientation)) {
    RCLCPP_WARN(this->get_logger(), 
               "\033[33m[WARN]: The quaternion of the drone position has not been unitized. "
               "Please check whether the position information is correct!\033[0m");
    return;
  }
  
  switch (Drone_mode) {
    case Takeoff:
      have_planner_cmd = false;
      mavros_point_cmd.pose.position.x = takeoff_point[0];
      mavros_point_cmd.pose.position.y = takeoff_point[1];
      mavros_point_cmd.pose.position.z = takeoff_point[2];
      mavros_point_cmd.pose.orientation = createQuaternionMsgFromYaw(waypoint_list[0].yaw);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                          "Send point to take off");
      break;
      
    case Run_point:
      if (!flag_planner_px4) {
        if (have_planner_cmd) 
          mavros_point_cmd = planner_cmd;
        else 
          mavros_point_cmd = last_mavros_point_cmd;
      } else {
        mavros_point_cmd = patrol_cmd;
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                          "Send point to Run_point");
      break;
      
    case Aligning:
      have_planner_cmd = false;
      mavros_point_cmd = patrol_cmd;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                          "Send point to Aligning");
      break;
      
    case Land:
      have_planner_cmd = false;
      mavros_point_cmd = patrol_cmd;
      if (!flag_land) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "Send point to Land");
      }
      break;
  }
  
  // 计算当前位置到目标点的距离
  Eigen::Vector3d current_pos(
    uav_pose.pose.position.x, 
    uav_pose.pose.position.y, 
    uav_pose.pose.position.z);
  
  Eigen::Vector3d target_pos(
    mavros_point_cmd.pose.position.x, 
    mavros_point_cmd.pose.position.y, 
    mavros_point_cmd.pose.position.z);
  
  double distance_to_target = (target_pos - current_pos).norm();
  
  // 如果距离超过px4_max_distance，则进行插值
  if (distance_to_target > px4_max_distance) {
    Eigen::Vector3d direction = (target_pos - current_pos).normalized();
    Eigen::Vector3d new_pos = current_pos + direction * px4_max_distance;
    
    mavros_point_cmd.pose.position.x = new_pos.x();
    mavros_point_cmd.pose.position.y = new_pos.y();
    mavros_point_cmd.pose.position.z = new_pos.z();
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Target point adjusted to max distance limit.");
  }
  
  // 提取当前yaw和目标yaw
  double current_yaw = getYawFromQuaternion(uav_pose.pose.orientation);
  double target_yaw = getYawFromQuaternion(mavros_point_cmd.pose.orientation);
  
  // 计算yaw差值
  double yaw_diff = target_yaw - current_yaw;
  yaw_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));
  
  // 限制yaw改变量
  if (std::abs(yaw_diff) > max_yaw_change) {
    yaw_diff = (yaw_diff > 0) ? max_yaw_change : -max_yaw_change;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Target point adjusted to max yaw limit.");
  }
  
  // 插值后的yaw
  double interpolated_yaw = current_yaw + yaw_diff;
  interpolated_yaw = std::atan2(std::sin(interpolated_yaw), std::cos(interpolated_yaw));
  
  // 创建四元数
  mavros_point_cmd.pose.orientation = createQuaternionMsgFromYaw(interpolated_yaw);
  
  // 发布命令
  mavros_point_cmd.header.stamp = this->now();
  mavros_point_cmd.header.frame_id = "camera_init";
  mavros_point_cmd_pub->publish(mavros_point_cmd);
  last_mavros_point_cmd = mavros_point_cmd;
  
  // 判断是否已经降落，降落成功就锁桨
  if (Drone_mode == Land && uav_pose.pose.position.z <= 0.08) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "OVER!");
    // Lock();
  }
}

void LLController::Lock() {
  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->broadcast = false;
  request->command = 400;     // 命令ID
  request->confirmation = 0;  // 确认
  request->param1 = 0.0;      // 参数1
  request->param2 = 21196.0;  // 参数2
  request->param3 = 0.0;      // 参数3
  request->param4 = 0.0;      // 参数4
  request->param5 = 0.0;      // 参数5
  request->param6 = 0.0;      // 参数6
  request->param7 = 0.0;      // 参数7
  
  auto result_future = land_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                          "Vehicle DisArmed!");
    }
  }
}

void LLController::CallLand() {
  if (auto_land) {
    if (!flag_landing_detect) {
      adjust_target_position[0] = waypoint_list[waypoint_next].x;
      adjust_target_position[1] = waypoint_list[waypoint_next].y;
    }
    
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "AUTO.LAND";
    
    auto result_future = set_mode_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
      if (result->mode_sent) {
        RCLCPP_INFO(this->get_logger(), "\033[32m Auto land mode done \033[0m");
      }
    }
  } else {
    if (!flag_landing_detect) {
      adjust_target_position[0] = waypoint_list[waypoint_next].x;
      adjust_target_position[1] = waypoint_list[waypoint_next].y;
      adjust_target_position[3] = waypoint_list[waypoint_next].yaw;
    }
    land_height = -1.0;
  }
  flag_land = true;
}

void LLController::load_params() {
  // 1. 声明所有参数（提供默认值）
  // 开关参数
  this->declare_parameter<int>("switch.flag_planner_px4", 1);
  this->declare_parameter<int>("switch.flag_landing_detect", 1);
  this->declare_parameter<int>("switch.auto_land", 0);
  
  // 阈值参数
  this->declare_parameter<double>("threshould.takeoff_threshould", 0.3);
  this->declare_parameter<double>("threshould.planner_min_pub_threshould", 0.02);
  this->declare_parameter<double>("threshould.waypoint_threshould", 0.3);
  this->declare_parameter<double>("threshould.aligning_threshould", 0.15);
  this->declare_parameter<double>("threshould.landing_threshould", 0.15);
  this->declare_parameter<double>("threshould.arrive_yaw_threshould", 0.3);
  this->declare_parameter<int>("threshould.times_detect_threshould", 30);
  this->declare_parameter<double>("threshould.waypoint_adjust_max_second_threshould", 10.0);
  this->declare_parameter<double>("threshould.land_adjust_max_second_threshould", 10.0);
  
  // 其他参数
  this->declare_parameter<double>("land_height", 0.3);
  this->declare_parameter<double>("px4_max_distance", 1.2);
  this->declare_parameter<double>("max_yaw_change", 0.3);
  this->declare_parameter<bool>("debug", false);
  this->declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>());

  // 2. 获取参数值
  flag_planner_px4 = this->get_parameter("switch.flag_planner_px4").as_int();
  flag_landing_detect = this->get_parameter("switch.flag_landing_detect").as_int();
  auto_land = this->get_parameter("switch.auto_land").as_int();
  
  takeoff_threshould = this->get_parameter("threshould.takeoff_threshould").as_double();
  planner_min_pub_threshould = this->get_parameter("threshould.planner_min_pub_threshould").as_double();
  waypoint_threshould = this->get_parameter("threshould.waypoint_threshould").as_double();
  aligning_threshould = this->get_parameter("threshould.aligning_threshould").as_double();
  landing_threshould = this->get_parameter("threshould.landing_threshould").as_double();
  arrive_yaw_threshould = this->get_parameter("threshould.arrive_yaw_threshould").as_double();
  times_detect_threshould = this->get_parameter("threshould.times_detect_threshould").as_int();
  waypoint_adjust_max_second_threshould = this->get_parameter("threshould.waypoint_adjust_max_second_threshould").as_double();
  land_adjust_max_second_threshould = this->get_parameter("threshould.land_adjust_max_second_threshould").as_double();
  
  land_height = this->get_parameter("land_height").as_double();
  px4_max_distance = this->get_parameter("px4_max_distance").as_double();
  max_yaw_change = this->get_parameter("max_yaw_change").as_double();
  debug = this->get_parameter("debug").as_bool();
  
  auto waypoint_params = this->get_parameter("waypoints").as_string_array();

  // 3. 输出所有参数值
  RCLCPP_INFO(this->get_logger(), "\n========== 加载参数 ==========");
  
  // 开关参数
  RCLCPP_INFO(this->get_logger(), "开关参数:");
  RCLCPP_INFO(this->get_logger(), "  flag_planner_px4: %d", flag_planner_px4);
  RCLCPP_INFO(this->get_logger(), "  flag_landing_detect: %d", flag_landing_detect);
  RCLCPP_INFO(this->get_logger(), "  auto_land: %d", auto_land);
  
  // 阈值参数
  RCLCPP_INFO(this->get_logger(), "\n阈值参数:");
  RCLCPP_INFO(this->get_logger(), "  takeoff_threshould: %.2f", takeoff_threshould);
  RCLCPP_INFO(this->get_logger(), "  planner_min_pub_threshould: %.3f", planner_min_pub_threshould);
  RCLCPP_INFO(this->get_logger(), "  waypoint_threshould: %.2f", waypoint_threshould);
  RCLCPP_INFO(this->get_logger(), "  aligning_threshould: %.2f", aligning_threshould);
  RCLCPP_INFO(this->get_logger(), "  landing_threshould: %.2f", landing_threshould);
  RCLCPP_INFO(this->get_logger(), "  arrive_yaw_threshould: %.2f", arrive_yaw_threshould);
  RCLCPP_INFO(this->get_logger(), "  times_detect_threshould: %d", times_detect_threshould);
  RCLCPP_INFO(this->get_logger(), "  waypoint_adjust_max_second_threshould: %.1f", waypoint_adjust_max_second_threshould);
  RCLCPP_INFO(this->get_logger(), "  land_adjust_max_second_threshould: %.1f", land_adjust_max_second_threshould);
  
  // 其他参数
  RCLCPP_INFO(this->get_logger(), "\n其他参数:");
  RCLCPP_INFO(this->get_logger(), "  land_height: %.2f", land_height);
  RCLCPP_INFO(this->get_logger(), "  px4_max_distance: %.2f", px4_max_distance);
  RCLCPP_INFO(this->get_logger(), "  max_yaw_change: %.2f", max_yaw_change);
  RCLCPP_INFO(this->get_logger(), "  debug: %s", debug ? "true" : "false");
  
  // 航路点参数
  RCLCPP_INFO(this->get_logger(), "\n航路点参数 (数量: %zu):", waypoint_params.size());
  for (size_t i = 0; i < waypoint_params.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "  waypoint[%zu]: %s", i, waypoint_params[i].c_str());
  }
  
  // 解析航路点字符串
  RCLCPP_INFO(this->get_logger(), "\n结构化航路点:");
  for (const auto& wp_str : waypoint_params) {
    Waypoint wp;
    std::istringstream iss(wp_str);
    std::string token;
    
    while (std::getline(iss, token, ',')) {
      auto pos = token.find(':');
      if (pos == std::string::npos) continue;
      
      std::string key = token.substr(0, pos);
      std::string value = token.substr(pos + 1);
      
      if (key == "x") wp.x = std::stod(value);
      else if (key == "y") wp.y = std::stod(value);
      else if (key == "z") wp.z = std::stod(value);
      else if (key == "yaw") wp.yaw = std::stod(value) * M_PI / 180.0;
      else if (key == "pointmode") wp.pointmode = value;
    }
    
    waypoint_list.push_back(wp);
    
    RCLCPP_INFO(this->get_logger(), 
               "  x=%.2f, y=%.2f, z=%.2f, yaw=%.2f rad (%.1f°), mode=%s",
               wp.x, wp.y, wp.z, wp.yaw, wp.yaw * 180.0 / M_PI, wp.pointmode.c_str());
  }
  
  // 起飞点
  if (!waypoint_list.empty()) {
    takeoff_point[0] = waypoint_list[0].x;
    takeoff_point[1] = waypoint_list[0].y;
    takeoff_point[2] = waypoint_list[0].z;
    RCLCPP_INFO(this->get_logger(), "\n起飞点: (%.2f, %.2f, %.2f)", 
               takeoff_point[0], takeoff_point[1], takeoff_point[2]);
  } else {
    RCLCPP_WARN(this->get_logger(), "未加载航路点，使用默认起飞点");
    takeoff_point[0] = 0.0;
    takeoff_point[1] = 0.0;
    takeoff_point[2] = 0.0;
  }
  
  RCLCPP_INFO(this->get_logger(), "=================================\n");
}



void LLController::pub_goal(geometry_msgs::msg::PoseStamped goal_msg) {
  goal_msg.header.frame_id = "camera_init";
  goal_msg.header.stamp = this->now();
  
  if (!flag_planner_px4) {
    now_pub_point[0] = goal_msg.pose.position.x;
    now_pub_point[1] = goal_msg.pose.position.y;
    now_pub_point[2] = goal_msg.pose.position.z;
    
    if ((last_pub_point - now_pub_point).norm() >= planner_min_pub_threshould) {
      if (debug) {
        RCLCPP_INFO(this->get_logger(), 
                   "Planner goal_msg: [%f, %f, %f]",
                   goal_msg.pose.position.x,
                   goal_msg.pose.position.y,
                   goal_msg.pose.position.z);
      }
      
      goal_msg.header.stamp = this->now();
      setplanner_goal_pub_->publish(goal_msg);
      last_pub_point = now_pub_point;
    }
  } else {
    goal_msg.header.stamp = this->now();
    patrol_cmd = goal_msg;
  }
}

void LLController::NextPoint() {
  waypoint_now = waypoint_next;
  waypoint_next = waypoint_now + 1;
  
  if (waypoint_next >= static_cast<int>(waypoint_list.size())) {
    waypoint_next = waypoint_list.size() - 1;
  }
  
  Point_mode = stringToPointmode(waypoint_list[waypoint_next].pointmode);
  
  RCLCPP_INFO(this->get_logger(), 
             "\033[32mHave arrive Point number: %d\nDrone pose now [x, y, z]: %f, %f, %f\033[0m",
             waypoint_now,
             uav_pose.pose.position.x,
             uav_pose.pose.position.y,
             uav_pose.pose.position.z);
  
  RCLCPP_INFO(this->get_logger(), 
             "\033[42;37mNext Point number: %d\nNext Point Pose: x = %f, y = %f, z = %f\033[0m",
             waypoint_next,
             waypoint_list[waypoint_next].x,
             waypoint_list[waypoint_next].y,
             waypoint_list[waypoint_next].z);
}

bool LLController::WayPointDetectDone() {
  static auto last_output_time = this->now();
  double output_interval = 0.2;
  
  if (first_call) {
    start_time = this->now();
    first_call = false;
    times_detect = 0;
  }
  
  if (have_waypoint_mark) {
    adjust_target_position[0] = waypoint_mark.pose.position.x;
    adjust_target_position[1] = waypoint_mark.pose.position.y;
    adjust_target_position[2] = waypoint_mark.pose.position.z;
    adjust_target_position[3] = getYawFromQuaternion(waypoint_mark.pose.orientation);
    times_detect++;
  } else {
    adjust_target_position[0] = waypoint_list[waypoint_next].x;
    adjust_target_position[1] = waypoint_list[waypoint_next].y;
    adjust_target_position[2] = waypoint_list[waypoint_next].z;
    adjust_target_position[3] = waypoint_list[waypoint_next].yaw;
  }
  
  auto current_time = this->now();
  double elapsed_time = (current_time - start_time).seconds();
  
  int clamped_detect_threshould = std::max(1, static_cast<int>(times_detect_threshould));
  double clamped_time_threshould = std::max(0.1, waypoint_adjust_max_second_threshould);
  
  double detect_progress = std::min(1.0, static_cast<double>(times_detect) / clamped_detect_threshould);
  double time_progress = std::min(1.0, elapsed_time / clamped_time_threshould);
  
  if ((current_time - last_output_time).seconds() >= output_interval) {
    last_output_time = current_time;
    
    int bar_width = 20;
    auto buildProgressBar = [&](double progress) {
      std::string bar;
      int pos = bar_width * progress;
      for (int i = 0; i < bar_width; ++i) {
        bar += (i < pos) ? "#" : "-";
      }
      return bar;
    };
    
    std::string detect_bar = buildProgressBar(detect_progress);
    std::string time_bar = buildProgressBar(time_progress);
    
    RCLCPP_INFO(this->get_logger(), 
               "Alignment Progress: [%s] %d/%d | Time Progress: [%s] %.1f/%.1f sec",
               detect_bar.c_str(), times_detect, clamped_detect_threshould,
               time_bar.c_str(), elapsed_time, clamped_time_threshould);
  }
  
  if (times_detect >= clamped_detect_threshould || 
      elapsed_time >= clamped_time_threshould) {
    first_call = true;
    times_detect = 0;
    return true;
  }
  
  return false;
}

bool LLController::LandDetectDone() {
  static auto last_output_time = this->now();
  double output_interval = 0.2;
  
  if (first_call) {
    start_time = this->now();
    first_call = false;
    times_detect = 0;
  }
  
  if (have_land_mark) {
    adjust_target_position[0] = land_mark.pose.position.x;
    adjust_target_position[1] = land_mark.pose.position.y;
    adjust_target_position[2] = land_height;
    adjust_target_position[3] = getYawFromQuaternion(land_mark.pose.orientation);
    times_detect++;
    have_land_mark = false;
  } else {
    adjust_target_position[0] = waypoint_list[waypoint_next].x;
    adjust_target_position[1] = waypoint_list[waypoint_next].y;
    adjust_target_position[2] = land_height;
    adjust_target_position[3] = waypoint_list[waypoint_next].yaw;
  }
  
  auto current_time = this->now();
  double elapsed_time = (current_time - start_time).seconds();
  
  int clamped_detect_threshould = std::max(1, static_cast<int>(times_detect_threshould));
  double clamped_time_threshould = std::max(0.1, land_adjust_max_second_threshould);
  
  double detect_progress = std::min(1.0, static_cast<double>(times_detect) / clamped_detect_threshould);
  double time_progress = std::min(1.0, elapsed_time / clamped_time_threshould);
  
  if ((current_time - last_output_time).seconds() >= output_interval && !flag_land) {
    last_output_time = current_time;
    
    int bar_width = 20;
    auto buildProgressBar = [&](double progress) {
      std::string bar;
      int pos = bar_width * progress;
      for (int i = 0; i < bar_width; ++i) {
        bar += (i < pos) ? "#" : "-";
      }
      return bar;
    };
    
    std::string detect_bar = buildProgressBar(detect_progress);
    std::string time_bar = buildProgressBar(time_progress);
    
    RCLCPP_INFO(this->get_logger(), 
               "Landing Progress: [%s] %d/%d | Time Progress: [%s] %.1f/%.1f sec",
               detect_bar.c_str(), times_detect, clamped_detect_threshould,
               time_bar.c_str(), elapsed_time, clamped_time_threshould);
  }
  
  if (times_detect >= clamped_detect_threshould || 
      elapsed_time >= clamped_time_threshould) {
    first_call = true;
    times_detect = 0;
    return true;
  }
  
  return false;
}

geometry_msgs::msg::Quaternion LLController::createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

double LLController::getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace astra_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<astra_control::LLController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}