cmake_minimum_required(VERSION 3.0.2)
project(csi_camera_driver)

## 1) catkin 依赖（不要包含 OpenCV / cv_bridge）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  camera_info_manager
)

## 2) GStreamer（通过 pkg-config）
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0>=1.14
  gstreamer-app-1.0
  gstreamer-video-1.0
  gstreamer-rtsp-server-1.0
)

## 3) 导出 catkin 包
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs image_transport camera_info_manager
)

## 4) 头文件与库路径
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

link_directories(
  ${GST_LIBRARY_DIRS}
)

## 5) 可执行文件（你的源码文件名保持和你工程一致）
# 若你的源文件名是 src/csi_camera_node.cpp（包含纯 GStreamer 版本代码），保持不变
add_executable(csi_camera_node src/csi_camera_node.cpp)

## 6) 链接（关键：必须链接 ${catkin_LIBRARIES} 才会把 camera_info_manager 链进来）
target_link_libraries(csi_camera_node
  ${catkin_LIBRARIES}
  ${GST_LIBRARIES}
)

## 7) 安装
install(TARGETS csi_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#########################
# 可选：dynamic_reconfigure（如果你暂时不用，就保持注释）
#########################
# find_package(dynamic_reconfigure QUIET)
# if(dynamic_reconfigure_FOUND)
#   generate_dynamic_reconfigure_options(
#     cfg/Camera.cfg
#   )
#   add_dependencies(csi_camera_node ${${PROJECT_NAME}_gencfg})
# endif()
