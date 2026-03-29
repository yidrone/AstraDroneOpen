#!/bin/bash 
tmux kill-session -t demonstration
tmux new-session -d -s demonstration

# split
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 3
tmux split-window -v
#tmux select-pane -t 2
#tmux split-window -v
#tmux select-pane -t 6
#tmux split-window -v

tmux select-pane -t 0
tmux send-keys "roscore &" C-m 
tmux send-keys "sleep 30s && cde" C-m 
tmux send-keys "rostopic echo /mavros/local_position/pose" C-m

tmux select-pane -t 1
tmux send-keys "sleep 5s && cde" C-m 
tmux send-keys "roslaunch mavros px4.launch" C-m 

tmux select-pane -t 2
tmux send-keys "sleep 11s && cde " C-m 
tmux send-keys "astra" C-m
tmux send-keys "roslaunch livox_ros_driver2 msg_MID360.launch" C-m 

tmux select-pane -t 3
tmux send-keys "sleep 17s && cde" C-m 
tmux send-keys "astra" C-m 
tmux send-keys "roslaunch fast_lio mapping_mid360.launch" C-m 

tmux select-pane -t 4
tmux send-keys "sleep 25s && cde" C-m 
tmux send-keys "astra" C-m 
tmux send-keys "roslaunch exploration_manager lidar_exploration.launch" C-m 

tmux select-pane -t 5
tmux send-keys "sleep 35s && cde" C-m 
tmux send-keys "astra" C-m 
tmux send-keys "roslaunch exploration_manager rviz.launch" C-m 

tmux -2 attach-session -t demonstration
