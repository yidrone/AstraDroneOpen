cmake_minimum_required(VERSION 2.8.3)
project(camera_sdk)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
  dynamic_reconfigure
)

find_package(OpenCV)  # 仅用于可选功能

# 寻找TurboJPEG
find_package(PkgConfig REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)

generate_dynamic_reconfigure_options(
  cfg/CameraConfig.cfg
)

catkin_package(
  CATKIN_DEPENDS  roscpp sensor_msgs image_transport cv_bridge camera_info_manager dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TURBOJPEG_INCLUDE_DIRS}
)

add_executable(camera_sdk_node src/camera_sdk_node.cpp)
target_link_libraries(camera_sdk_node
  ${catkin_LIBRARIES}
  ${TURBOJPEG_LIBRARIES}
)

install(TARGETS camera_sdk_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)