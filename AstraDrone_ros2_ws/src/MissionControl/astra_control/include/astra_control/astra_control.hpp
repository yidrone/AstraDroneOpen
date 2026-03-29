#ifndef ASTRA_CONTROL_HPP
#define ASTRA_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <memory>
#include <sstream>

namespace astra_control {

// 航路点结构体
struct Waypoint {
  double x;
  double y;
  double z;
  double yaw;
  std::string pointmode;
};

// 无人机状态枚举
enum DroneState { Takeoff, Run_point, Aligning, Land };
enum PointMode { Detect_point, Nothing_point, Land_point };

class LLController : public rclcpp::Node {
 public:
  LLController();
  ~LLController();

 private:
  void initializeNode();
  void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void plannercmdCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void waypointMarkCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void landMarkCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cmdCallback();
  void patrol();
  void Lock();
  void CallLand();
  void load_params();
  void pub_goal(geometry_msgs::msg::PoseStamped goal_msg);
  void NextPoint();
  bool WayPointDetectDone();
  bool LandDetectDone();
  bool isQuaternionNormalized(const geometry_msgs::msg::Quaternion& q, double tolerance = 1e-6);
  
  rclcpp::QoS reliable_qos_ = rclcpp::QoS(10).reliable();
  // 添加函数实现
  Eigen::Vector3f toEigen(const geometry_msgs::msg::Point& point);
  double distance3d(float x1, float y1, float z1, float x2, float y2, float z2);
  PointMode stringToPointmode(const std::string& str);
  
  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

  // ROS2 订阅器和发布器
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fastplanner_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detect_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr land_mark_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_point_cmd_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setplanner_goal_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr land_client;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
  rclcpp::TimerBase::SharedPtr cmd_timer;

  // 状态变量
  DroneState Drone_mode = Takeoff;
  PointMode Point_mode = Nothing_point;
  int waypoint_now;
  int waypoint_next;
  std::vector<Waypoint> waypoint_list;

  // 参数
  bool flag_planner_px4;
  bool flag_landing_detect;
  bool auto_land;
  double takeoff_threshould;
  double planner_min_pub_threshould;
  double waypoint_threshould;
  double aligning_threshould;
  double landing_threshould;
  double arrive_yaw_threshould;
  int times_detect_threshould;
  double waypoint_adjust_max_second_threshould;
  double land_adjust_max_second_threshould;
  double land_height;
  double px4_max_distance;
  double max_yaw_change;
  bool debug;

  // 临时变量
  geometry_msgs::msg::PoseStamped planner_cmd;
  geometry_msgs::msg::PoseStamped waypoint_mark;
  geometry_msgs::msg::PoseStamped land_mark;
  geometry_msgs::msg::PoseStamped patrol_cmd;
  geometry_msgs::msg::PoseStamped mavros_point_cmd;
  double arrive_goal_threshould;
};

}  // namespace astra_control

#endif  // ASTRA_CONTROL_HPP