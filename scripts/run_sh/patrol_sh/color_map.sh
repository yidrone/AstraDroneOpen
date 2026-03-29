#!/bin/bash 
tmux kill-session -t color_map
tmux new-session -d -s color_map

# split
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "roslaunch mavros px4.launch" C-m

tmux select-pane -t 1
tmux send-keys "sleep 6s && astra" C-m 
tmux send-keys "roslaunch livox_ros_driver2 msg_MID360.launch" C-m 

tmux select-pane -t 2
tmux send-keys "sleep 10s && astra" C-m 
tmux send-keys "roslaunch orbbec_camera gemini_330_series.launch" C-m 

tmux select-pane -t 3
tmux send-keys "sleep 17s && astra" C-m 
tmux send-keys "roslaunch fast_lio mapping_mid360_color.launch" C-m 


tmux -2 attach-session -t color_map
