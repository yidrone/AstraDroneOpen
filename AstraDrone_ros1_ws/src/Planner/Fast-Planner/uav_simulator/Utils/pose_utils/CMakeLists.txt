cmake_minimum_required(VERSION 2.8.3)
project(pose_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Armadillo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_utils
  CATKIN_DEPENDS roscpp
  DEPENDS Armadillo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(pose_utils
  src/pose_utils.cpp
)

target_link_libraries(pose_utils
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)

install(TARGETS pose_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
