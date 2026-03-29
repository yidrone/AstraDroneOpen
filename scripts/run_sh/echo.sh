#!/bin/bash 
tmux kill-session -t echo
tmux new-session -d -s echo

# split
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "rostopic echo /mavros/local_position/pose" C-m 

tmux select-pane -t 1
tmux send-keys "rostopic echo /mavros/state" C-m 

tmux select-pane -t 2
tmux send-keys "rostopic echo /mavros/setpoint_position/local" C-m 

tmux select-pane -t 3
tmux send-keys "rostopic echo /mavros/setpoint_raw/local" C-m 

tmux -2 attach-session -t echo
