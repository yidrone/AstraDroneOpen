cmake_minimum_required(VERSION 3.0.2)
project(eva_metric)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  std_msgs
  visualization_msgs
  geometry_msgs
  tf
  tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS segmentation io)
find_package(OpenCV REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES eva_metric
 CATKIN_DEPENDS message_filters roscpp std_msgs visualization_msgs geometry_msgs tf tf2_ros
#  DEPENDS system_lib
)

# 模块方式寻找 yaml-cpp（不使用 yaml-cppConfig.cmake）：  find_package(yaml-cpp REQUIRED)
find_library(YAML_CPP_LIB yaml-cpp REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})




# 独立的程序
add_executable(globalclouds 
    src/utils/clouds2ikdtree.cpp
    src/utils/ikd_Tree.cpp  # 确保包含 ikd_Tree.cpp
)

target_link_libraries(globalclouds   ${catkin_LIBRARIES} )

target_link_libraries(globalclouds
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIB}  # 👈 使用 find_library 结果
  # OpenMP::OpenMP_CXX
)

target_include_directories(globalclouds
  PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
