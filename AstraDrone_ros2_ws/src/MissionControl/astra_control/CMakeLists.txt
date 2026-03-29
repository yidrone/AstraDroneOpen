cmake_minimum_required(VERSION 3.5)
project(astra_control)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)  # 确保这一行存在
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(astra_control_node
  src/astra_control.cpp
)

# 使用 ament_target_dependencies 管理 ROS2 依赖
ament_target_dependencies(astra_control_node
  rclcpp
  geometry_msgs
  mavros_msgs  # 关键：添加 mavros_msgs 依赖
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 单独链接 yaml-cpp
target_link_libraries(astra_control_node
  yaml-cpp
)

# 安装目标
install(TARGETS astra_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()