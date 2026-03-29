#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/RCIn.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <algorithm>

// RcObstacleAvoidance
//
// - 将遥控输入映射为当前位置附近的目标点（含偏航角），持续发送给 Fast-Planner
// - 订阅 Fast-Planner 输出的避障轨迹（位置/速度），实时转发给 PX4 让无人机沿障碍物绕行
// - 提供死区、步长、超时等保护，确保遥控缺失或规划停更时自动回到当前位置悬停
class RcObstacleAvoidance
{
public:
  RcObstacleAvoidance()
    : nh_(),
      pnh_("~"),
      has_pose_(false),
      has_target_(false),
      has_planner_pose_(false),
      has_planner_vel_(false),
      initial_takeoff_done_(false)
  {
    // ---------------- 参数读取 ----------------
    // 话题配置
    pnh_.param<std::string>("rc_topic", rc_topic_, std::string("/mavros/rc/in"));
    pnh_.param<std::string>("pose_topic", pose_topic_, std::string("/mavros/local_position/pose"));
    pnh_.param<std::string>("goal_topic", goal_topic_, std::string("/fastplanner/goal"));
    pnh_.param<std::string>("planner_pose_topic", planner_pose_topic_,
                            std::string("/fastplanner/setpoint_position/local"));
    pnh_.param<std::string>("planner_vel_topic", planner_vel_topic_,
                            std::string("/fastplanner/setpoint_velocity/cmd_vel"));
    pnh_.param<std::string>("mavros_position_topic", mavros_position_topic_,
                            std::string("/mavros/setpoint_position/local"));
    pnh_.param<std::string>("mavros_velocity_topic", mavros_velocity_topic_,
                            std::string("/mavros/setpoint_velocity/cmd_vel"));

    // 摇杆输入对应的移动配置
    pnh_.param<double>("deadband", deadband_, 50.0);
    pnh_.param<double>("max_xy_step", max_xy_step_, 1.0);
    pnh_.param<double>("max_z_step", max_z_step_, 0.6);
    pnh_.param<double>("max_yaw_change", max_yaw_change_, 0.35);

    pnh_.param<int>("roll_channel", roll_channel_, 1);
    pnh_.param<int>("pitch_channel", pitch_channel_, 2);
    pnh_.param<int>("throttle_channel", throttle_channel_, 3);
    pnh_.param<int>("yaw_channel", yaw_channel_, 4);

    // 发布控制选项
    pnh_.param<bool>("use_fastplanner", use_fastplanner_, true);
    pnh_.param<bool>("enable_planner_follow", enable_planner_follow_, true);
    pnh_.param<double>("publish_rate", publish_rate_, 20.0);
    pnh_.param<double>("command_timeout", command_timeout_, 0.5);
    pnh_.param<double>("planner_cmd_timeout", planner_cmd_timeout_, 0.3);
    pnh_.param<double>("initial_takeoff_height", initial_takeoff_height_, 1.0);
    pnh_.param<double>("initial_takeoff_tolerance", initial_takeoff_tolerance_, 0.1);

    rc_sub_ = nh_.subscribe(rc_topic_, 10, &RcObstacleAvoidance::rcCallback, this);
    pose_sub_ = nh_.subscribe(pose_topic_, 10, &RcObstacleAvoidance::poseCallback, this);
    if (use_fastplanner_)
    {
      goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 10);
    }
    if (enable_planner_follow_)
    {
      // 订阅规划器的避障轨迹（位姿 & 速度），并转发给 PX4
      planner_pose_sub_ = nh_.subscribe(planner_pose_topic_, 10, &RcObstacleAvoidance::plannerPoseCallback, this);
      planner_vel_sub_ = nh_.subscribe(planner_vel_topic_, 10, &RcObstacleAvoidance::plannerVelCallback, this);
      mavros_position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(mavros_position_topic_, 20);
      mavros_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(mavros_velocity_topic_, 20);
    }

    // 定时任务：
    // 1) 将最新 RC 目标发布给 Fast-Planner
    // 2) 将最新规划轨迹转发给 PX4
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &RcObstacleAvoidance::publishTimer, this);

    ROS_INFO_STREAM("RC obstacle avoidance node started. RC topic: " << rc_topic_
                    << ", pose topic: " << pose_topic_ << ", goal topic: " << goal_topic_
                    << ", planner pose: " << planner_pose_topic_ << ", planner vel: " << planner_vel_topic_
                    << ", mavros position: " << mavros_position_topic_ << ", mavros velocity: " << mavros_velocity_topic_);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber rc_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber planner_pose_sub_;
  ros::Subscriber planner_vel_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher mavros_position_pub_;
  ros::Publisher mavros_velocity_pub_;
  ros::Timer publish_timer_;

  std::string rc_topic_;
  std::string pose_topic_;
  std::string goal_topic_;
  std::string planner_pose_topic_;
  std::string planner_vel_topic_;
  std::string mavros_position_topic_;
  std::string mavros_velocity_topic_;

  double deadband_;
  double max_xy_step_;
  double max_z_step_;
  double max_yaw_change_;
  double publish_rate_;
  double command_timeout_;
  double planner_cmd_timeout_;
  double initial_takeoff_height_;
  double initial_takeoff_tolerance_;

  int roll_channel_;
  int pitch_channel_;
  int throttle_channel_;
  int yaw_channel_;

  geometry_msgs::PoseStamped latest_pose_;      // 当前无人机位姿
  bool has_pose_;
  geometry_msgs::PoseStamped target_pose_;      // RC 转换后的目标位姿
  bool has_target_;
  ros::Time last_cmd_time_;                     // 最近一次接收到 RC 指令的时间

  geometry_msgs::PoseStamped planner_pose_cmd_; // Fast-Planner 输出的位姿轨迹点
  geometry_msgs::TwistStamped planner_vel_cmd_; // Fast-Planner 输出的速度指令
  bool has_planner_pose_;
  bool has_planner_vel_;
  ros::Time last_planner_pose_time_;
  ros::Time last_planner_vel_time_;

  bool initial_takeoff_done_;

  bool use_fastplanner_;
  bool enable_planner_follow_;

  static double normalizeChannel(uint16_t value, double deadband)
  {
    // 将PWM值映射到 [-1, 1]，并且为中心设置死区
    const double center = 1500.0;
    double delta = static_cast<double>(value) - center;
    if (std::fabs(delta) < deadband)
    {
      return 0.0;
    }
    delta = std::max(std::min(delta, 500.0), -500.0);
    return delta / 500.0;
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    latest_pose_ = *msg;
    has_pose_ = true;

    // 初始化起飞：将目标设置到指定高度，并持续发布直到达到高度
    if (!initial_takeoff_done_)
    {
      geometry_msgs::PoseStamped takeoff_target = latest_pose_;
      takeoff_target.pose.position.z = initial_takeoff_height_;
      takeoff_target.header.stamp = ros::Time::now();
      target_pose_ = takeoff_target;
      has_target_ = true;

      const double current_z = latest_pose_.pose.position.z;
      if (std::fabs(current_z - initial_takeoff_height_) <= initial_takeoff_tolerance_)
      {
        initial_takeoff_done_ = true;
        ROS_INFO("Initial takeoff complete, RC control enabled.");
      }
    }
  }

  void plannerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    // 缓存规划器的避障轨迹（位置指令）
    planner_pose_cmd_ = *msg;
    has_planner_pose_ = true;
    last_planner_pose_time_ = ros::Time::now();
  }

  void plannerVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
  {
    // 缓存规划器的避障轨迹（速度指令），当位置缺失时作为备选
    planner_vel_cmd_ = *msg;
    has_planner_vel_ = true;
    last_planner_vel_time_ = ros::Time::now();
  }

  void rcCallback(const mavros_msgs::RCIn::ConstPtr &msg)
  {
    if (!has_pose_)
    {
      ROS_WARN_THROTTLE(1.0, "Waiting for pose before processing RC commands.");
      return;  // 没有位姿前不生成目标，避免无效输出
    }

    if (!initial_takeoff_done_)
    {
      ROS_WARN_THROTTLE(1.0, "Initial takeoff in progress, RC input ignored until 1 m height reached.");
      return;
    }

    // 将通道索引转换为数组下标，缺失时回退到 1500（中位），保证健壮性
    const auto channel_value = [&](int channel_index) -> uint16_t {
      const size_t idx = static_cast<size_t>(channel_index - 1);
      if (idx >= msg->channels.size())
      {
        ROS_WARN_THROTTLE(1.0, "RC message missing channel %d", channel_index);
        return static_cast<uint16_t>(1500);
      }
      return msg->channels[idx];
    };

    // 归一化到 [-1, 1]，并根据通道方向调整符号
    const double roll_input = -normalizeChannel(channel_value(roll_channel_), deadband_);
    const double pitch_input = -normalizeChannel(channel_value(pitch_channel_), deadband_); // 前推为正向前
    const double throttle_input = normalizeChannel(channel_value(throttle_channel_), deadband_);
    const double yaw_input = -normalizeChannel(channel_value(yaw_channel_), deadband_);

    // 将归一化量映射为空间偏移/角度偏移，控制规划目标在当前位姿附近移动
    const double lateral_step = roll_input * max_xy_step_;
    const double forward_step = pitch_input * max_xy_step_;
    const double vertical_step = throttle_input * max_z_step_;
    const double yaw_step = yaw_input * max_yaw_change_;

    // ---------------- 生成目标位姿 ----------------
    geometry_msgs::PoseStamped goal = latest_pose_;
    goal.header.stamp = ros::Time::now();

    // 在机体系下的偏移量转换到世界系，使前后推杆始终沿机头方向运动
    const double current_yaw = tf::getYaw(latest_pose_.pose.orientation);
    const double cos_yaw = std::cos(current_yaw);
    const double sin_yaw = std::sin(current_yaw);
    const double delta_x = forward_step * cos_yaw - lateral_step * sin_yaw;
    const double delta_y = forward_step * sin_yaw + lateral_step * cos_yaw;

    goal.pose.position.x += delta_x;                      // 前后平移（沿机头）
    goal.pose.position.y += delta_y;                      // 左右平移（机头左/右）
    goal.pose.position.z = std::max(0.0, goal.pose.position.z + vertical_step); // 底高限制，防止给出负高度

    const double target_yaw = current_yaw + yaw_step;     // 偏航累加实现持续转向
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw);

    // 缓存目标，供定时器发布到规划器
    target_pose_ = goal;
    has_target_ = true;
    last_cmd_time_ = ros::Time::now();
  }

  void publishTarget()
  {
    // 初始起飞阶段：持续发布起飞目标
    if (!initial_takeoff_done_)
    {
      if (has_target_ && use_fastplanner_ && goal_pub_)
      {
        goal_pub_.publish(target_pose_);
      }
      return;
    }

    // 若超时未收到遥控量，则保持当前位置避免累积偏移
    if (has_pose_ && (ros::Time::now() - last_cmd_time_).toSec() > command_timeout_)
    {
      target_pose_ = latest_pose_;
      target_pose_.header.stamp = ros::Time::now();
      has_target_ = true;
    }

    if (!has_target_ || !has_pose_)
    {
      return;
    }

    if (use_fastplanner_ && goal_pub_)
    {
      goal_pub_.publish(target_pose_);
    }

  }

  void forwardPlannerCommand()
  {
    if (!enable_planner_follow_)
    {
      return;
    }

    const ros::Time now = ros::Time::now();
    // 检查规划指令的新鲜度，超过超时时间则不再使用，防止 PX4 跟随陈旧轨迹
    const bool pose_fresh = has_planner_pose_ && (now - last_planner_pose_time_).toSec() <= planner_cmd_timeout_;
    const bool vel_fresh = has_planner_vel_ && (now - last_planner_vel_time_).toSec() <= planner_cmd_timeout_;

    if (pose_fresh && mavros_position_pub_)
    {
      geometry_msgs::PoseStamped cmd = planner_pose_cmd_;
      cmd.header.stamp = now;  // 使用当前时间戳保持 PX4 接收
      mavros_position_pub_.publish(cmd);
      return;
    }

    if (vel_fresh && mavros_velocity_pub_)
    {
      geometry_msgs::TwistStamped cmd = planner_vel_cmd_;
      cmd.header.stamp = now;
      mavros_velocity_pub_.publish(cmd);
      return;
    }

    ROS_WARN_THROTTLE(1.0, "Planner command timeout, nothing forwarded to MAVROS.");
  }

  void publishTimer(const ros::TimerEvent &)
  {
    // 定时发布 RC 转换后的目标给 Fast-Planner，并跟随规划器轨迹到 PX4
    publishTarget();
    forwardPlannerCommand();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rc_obstacle_avoidance_node");
  RcObstacleAvoidance node;
  ros::spin();
  return 0;
}
