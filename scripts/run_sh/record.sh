#!/bin/bash 
tmux kill-session -t record
tmux new-session -d -s record

tmux select-pane -t 0
tmux send-keys "cd ~/bag " C-m 
tmux send-keys "rosbag	record   /camera/color/image_raw/compressed /csi_camera/image_raw/compressed  /cloud_registered_body /mavros/setpoint_position/local /mavros/setpoint_raw/local /mavros/setpoint_raw/local /mavros/state /tf /mavros/local_position/pose /mavros/local_position/odom /livox/lidar /livox/imu /camera/image_raw/compressed /freedom/static_pointcloud /mavros/battery /mavros/global_position/local /mavros/imu/data" C-m 
tmux -2 attach-session -t record
