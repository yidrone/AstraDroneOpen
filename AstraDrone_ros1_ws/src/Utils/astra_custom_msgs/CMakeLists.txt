cmake_minimum_required(VERSION 3.0.2)
project(astra_custom_msgs)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib_msgs
)

# 消息声明
add_message_files(
  DIRECTORY msg
  FILES
  # 添加你的自定义消息文件
  DroneStatus.msg
  States.msg
  MarkerMeasurement.msg
  MarkerMeasurementArray.msg
)

# 服务声明
add_service_files(
  DIRECTORY srv
  FILES
  # 添加你的自定义服务文件
  MissionCommand.srv
)

# 生成消息和服务
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib_msgs
)

# 包配置
catkin_package(
  CATKIN_DEPENDS 
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    actionlib_msgs
)