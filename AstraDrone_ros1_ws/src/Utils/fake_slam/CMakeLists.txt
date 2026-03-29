cmake_minimum_required(VERSION 3.0.2)
project(fake_slam)

## 查找catkin构建系统和所需的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf
  pcl_ros
  pcl_conversions
)

## 声明catkin的构建信息
catkin_package(
  CATKIN_DEPENDS roscpp
)

## 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 包含头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 添加可执行文件
add_executable(map_generator_node src/map_generator_node.cpp)

## 链接目标与catkin和其他依赖项的库
target_link_libraries(map_generator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
