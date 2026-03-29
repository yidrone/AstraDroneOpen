cmake_minimum_required(VERSION 3.0.2)
project(topic_tf_tran)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topic_tf_tran
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(topic2tf src/topic2tf.cpp)
target_link_libraries(topic2tf
  ${catkin_LIBRARIES}
)
add_executable(tf2topic src/tf2topic.cpp)
target_link_libraries(tf2topic
  ${catkin_LIBRARIES}
)
