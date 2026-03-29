#!/bin/bash 
tmux kill-session -t onboard_example
tmux new-session -d -s onboard_example

# split
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 4
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "roscore &" C-m 

tmux select-pane -t 1
tmux send-keys "sleep 3s && cde" C-m 
tmux send-keys "roslaunch mavros px4.launch" C-m 

tmux select-pane -t 2
tmux send-keys "sleep 6s && cde " C-m 
tmux send-keys "astra" C-m
tmux send-keys "roslaunch livox_ros_driver2 msg_MID360.launch" C-m 

tmux select-pane -t 3
tmux send-keys "sleep 9s && cde" C-m 
tmux send-keys "astra" C-m 
tmux send-keys "roslaunch fast_lio mapping_mid360.launch" C-m 

tmux select-pane -t 4
tmux send-keys "sleep 13s && cde" C-m 
tmux send-keys "astra" C-m 
tmux send-keys "roslaunch csi_camera_driver csi_camera.launch" C-m 

tmux select-pane -t 5
tmux send-keys "sleep 15s && cde" C-m 
tmux send-keys "astra" C-m 
tmux send-keys "roslaunch realsense2_camera rs_camera.launch" C-m 

tmux -2 attach-session -t onboard_example
