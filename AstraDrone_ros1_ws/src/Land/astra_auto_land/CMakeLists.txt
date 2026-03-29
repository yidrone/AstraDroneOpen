cmake_minimum_required(VERSION 3.0.2)
project(astra_auto_land)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
  FILES
  PrecisionLandingActivate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS include
  ##LIBRARIES precision_landing
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    nav_msgs 
    mavros_msgs 
    tf2 
    tf2_geometry_msgs 
    tf2_ros
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(astra_static_land src/astra_static_land.cpp)

## Add cmake target dependencies of the executable
add_dependencies(astra_static_land ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(astra_static_land
  ${catkin_LIBRARIES}
)

