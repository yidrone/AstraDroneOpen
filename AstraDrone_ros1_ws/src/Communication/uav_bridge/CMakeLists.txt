cmake_minimum_required(VERSION 3.0.2)
project(uav_bridge)

## 启用 C++11
add_compile_options(-std=c++11)

## 1. 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation  # <--- 必须有
  # 其他依赖...
  nav_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

## 2. 添加消息文件 (重点在这里！！！)
add_message_files(
  FILES
  CustomPoint.msg  # <--- 先写被依赖的（其实顺序不严格，但必须都有）
  CustomMsg.msg    # <--- CustomMsg 里面用了 CustomPoint
)

## 3. 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs  # 因为用到了 std_msgs/Header
)

## 4. 导出依赖
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime # <--- 必须有 message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## --- 可执行文件生成 (保持你不变的部分) ---
# ... (你的节点编译代码) ...
#add_executable(tcp_link_node src/tcp_link_node.cpp)
#add_executable(state_packer_node src/state_packer_node.cpp)
#add_executable(command_parser_node src/command_parser_node.cpp)
#add_executable(px4_unlocker_node src/px4_unlocker_node.cpp)
#target_link_libraries(tcp_link_node ${catkin_LIBRARIES})
#target_link_libraries(state_packer_node ${catkin_LIBRARIES})
#target_link_libraries(command_parser_node ${catkin_LIBRARIES})
#arget_link_libraries(px4_unlocker_node ${catkin_LIBRARIES})
