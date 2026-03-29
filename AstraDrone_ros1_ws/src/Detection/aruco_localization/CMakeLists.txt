cmake_minimum_required(VERSION 2.8.3)
project(aruco_localization)

# set opencv version
# set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV 3.4.8 REQUIRED)
message("find ${OpenCV_VERSION}")

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Debug")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(cv_bridge_INCLUDE_DIRS /home/orangepi/vision_opencv-noetic/cv_bridge/build/)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    cv_bridge
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    tf
    tf2_ros
    image_geometry
    astra_custom_msgs
)


##读取yaml
find_package(yaml-cpp REQUIRED)  

# aruco installs `Findaruco.cmake` to /usr/local/lib/cmake
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake/")
find_package(aruco REQUIRED)
message("find ${aruco_VERSION}")

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   # MarkerMeasurement.msg
#   # MarkerMeasurementArray.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/Controller.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES desktopquad
#  CATKIN_DEPENDS roscpp rospy nav_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${aruco_INCLUDE_DIRS})

add_executable(aruco_localization src/aruco_localization_node.cpp src/aruco_localization/ArucoLocalizer.cpp)
target_link_libraries(aruco_localization ${catkin_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBS} stdc++fs)
target_link_libraries(aruco_localization ${YAML_CPP_LIBRARIES})


# tf_manage
add_executable(tf_manage src/tf_manage.cpp)

target_link_libraries(tf_manage
  ${catkin_LIBRARIES}
)