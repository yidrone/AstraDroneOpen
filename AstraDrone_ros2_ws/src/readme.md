## 安装mavros2
sudo apt install ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs

ros2 launch mavros px4.launch

## 创建包
ros2 pkg create rc_transform_node --build-type ament_cmake --dependencies rclcpp geometry_msgs tf2_ros tf2_geometry_msgs mavros_msgs nav_msgs

## 在功能包根目录
colcon build 
source install/setup.bash

## 启动节点
ros2 launch rc_transform_node rc_transform.launch.py