#!/bin/bash 
tmux kill-session -t test
tmux new-session -d -s test

# split
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "roscore &" C-m 
tmux send-keys "sleep 30s && cde" C-m 
tmux send-keys "rostopic echo /mavros/local_position/pose" C-m

tmux select-pane -t 1
tmux send-keys "sleep 8s && cde" C-m 
tmux send-keys "roslaunch mavros px4.launch" C-m 

tmux select-pane -t 2
tmux send-keys "sleep 10s && cde " C-m 
tmux send-keys "astra" C-m
tmux send-keys "roslaunch livox_ros_driver2 msg_MID360.launch" C-m 

tmux select-pane -t 3
tmux send-keys "sleep 14s && cde" C-m 
tmux send-keys "astra" C-m 
tmux send-keys "roslaunch fast_lio mapping_mid360.launch" C-m 

tmux select-pane -t 4
tmux send-keys "sleep 19s && cde" C-m 
tmux send-keys "astra" C-m 
tmux send-keys "roslaunch astra_control astra_control_real.launch" C-m 

tmux select-pane -t 0
tmux split-window -h
tmux select-pane -t 1
tmux send-keys "sleep 50s && cde" C-m 
tmux send-keys "rosrun rviz rviz -d /home/uav/Desktop/astra_test.rviz" C-m

tmux -2 attach-session -t test
