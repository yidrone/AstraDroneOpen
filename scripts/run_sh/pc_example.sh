#!/bin/bash 
tmux kill-session -t pc_example
tmux new-session -d -s pc_example

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

tmux select-pane -t 1
tmux send-keys "sleep 3s" C-m 
tmux send-keys "roslaunch px4 astra_example.launch" C-m 

tmux select-pane -t 2
tmux send-keys "sleep 6s" C-m 
tmux send-keys "astra" C-m
tmux send-keys "roslaunch fast_lio mapping_mid360.launch rviz:=false" C-m 

tmux select-pane -t 3
tmux send-keys "sleep 10s" C-m 
tmux send-keys "astra" C-m
tmux send-keys "roslaunch offboard autoarming_control.launch" C-m 

tmux select-pane -t 4
tmux send-keys "qgc" C-m 

tmux -2 attach-session -t pc_example
