cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacle_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  gazebo_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  src/obstacle_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
