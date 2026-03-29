cmake_minimum_required(VERSION 3.0.2)
project(lidar_cam_fusion)

# 添加消息生成依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  message_filters
  pcl_conversions
  message_generation  # 确保包含
  std_msgs
)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

# 添加服务定义
add_service_files(
  FILES
  EstimateDepth.srv
)

# 生成消息和服务
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  sensor_msgs 
  cv_bridge 
  tf2_ros 
  tf2_geometry_msgs 
  pcl_ros 
  message_filters 
  message_runtime  # 添加运行时依赖
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/devel/include  # 添加生成的头文件路径
)

add_executable(lidar_cam_fusion src/lidar_cam_fusion.cpp)
target_link_libraries(lidar_cam_fusion
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)