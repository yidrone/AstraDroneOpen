cmake_minimum_required(VERSION 3.0.2)
project(rc_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  mavros_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs mavros_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rc_obstacle_avoidance_node src/rc_obstacle_avoidance_node.cpp)
target_link_libraries(rc_obstacle_avoidance_node ${catkin_LIBRARIES})

install(TARGETS rc_obstacle_avoidance_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
