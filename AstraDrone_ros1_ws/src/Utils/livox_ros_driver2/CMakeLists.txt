# Copyright(c) 2019 livoxtech limited.

cmake_minimum_required(VERSION 3.0)

#---------------------------------------------------------------------------------------
# Start livox_ros_driver2 project
#---------------------------------------------------------------------------------------
include(cmake/version.cmake)
project(livox_ros_driver2 VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
message(STATUS "livox_ros_driver2 version: ${LIVOX_ROS_DRIVER2_VERSION}")         

#---------------------------------------------------------------------------------------
# Add ROS Version MACRO
#---------------------------------------------------------------------------------------
add_definitions(-DBUILDING_ROS1)

#---------------------------------------------------------------------------------------
# find package and the dependency
#---------------------------------------------------------------------------------------
find_package(Boost 1.54 REQUIRED COMPONENTS
  system
  thread
  chrono
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  rosbag
  pcl_ros
)

find_package(PCL REQUIRED)
find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
  message(${APR_INCLUDE_DIRS})
  message(${APR_LIBRARIES})
endif ()

#---------------------------------------------------------------------------------------
# Generate messages
#---------------------------------------------------------------------------------------
add_message_files(FILES
  CustomPoint.msg
  CustomMsg.msg
)
generate_messages(DEPENDENCIES std_msgs)

#---------------------------------------------------------------------------------------
# Catkin Package
#---------------------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime pcl_ros
)

#---------------------------------------------------------------------------------------
# Build Configuration
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_static.a /usr/local/lib)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#---------------------------------------------------------------------------------------
# Executable
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp
  src/lddc.cpp
  src/livox_ros_driver2.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

target_compile_options(${PROJECT_NAME}_node PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  3rdparty
  src
)

target_link_libraries(${PROJECT_NAME}_node
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${Boost_LIBRARY}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
)

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch_ROS1/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch_ROS1
)

#---------------------------------------------------------------------------------------
# End of CMakeLists.txt
#---------------------------------------------------------------------------------------