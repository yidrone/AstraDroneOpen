cmake_minimum_required(VERSION 2.8.3)
project(target_prediction)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf        # 旧的tf库，用于kalman_filter
  tf2       # 新的tf2库，用于EKF_prediction
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  astra_custom_msgs
)

find_package(Eigen3 REQUIRED)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES target_prediction
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    visualization_msgs
    tf
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    astra_custom_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 现有的库和目标
add_library(target_prediction src/kalman_filter_dynamic.cpp)

## 现有的可执行文件 - 使用旧的tf库
add_executable(kalman_filter_dynamic src/kalman_filter_dynamic.cpp)
target_link_libraries(kalman_filter_dynamic 
  target_prediction
  ${catkin_LIBRARIES}
  ${tf_LIBRARIES}  # 显式链接tf库
)
add_dependencies(kalman_filter_dynamic 
  ${catkin_EXPORTED_TARGETS}
  target_prediction_generate_messages_cpp
)

add_executable(kalman_filter src/kalman_filter.cpp)
target_link_libraries(kalman_filter 
  target_prediction
  ${catkin_LIBRARIES}
  ${tf_LIBRARIES}  # 显式链接tf库
)
add_dependencies(kalman_filter 
  ${catkin_EXPORTED_TARGETS}
  target_prediction_generate_messages_cpp
)

## 新增的EKF预测节点 - 使用新的tf2库
add_executable(EKF_prediction src/EKF_prediction.cpp)
target_link_libraries(EKF_prediction 
  target_prediction
  ${catkin_LIBRARIES}
  # tf2库已包含在catkin_LIBRARIES中
)
add_dependencies(EKF_prediction 
  ${catkin_EXPORTED_TARGETS}
  target_prediction_generate_messages_cpp
)