#!/bin/bash
SESSION="base_lio"

# 1. 重置会话
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -n "Base_Layer"

# 2. 分割窗口 (左1右3布局)
# [0: Roscore] | [1: PX4]
#              | [2: LIO]
#              | [3: Tag]
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux split-window -v
tmux select-pane -t 1
tmux split-window -v
tmux split-window -v

# 重置布局为 4 格
tmux kill-pane -a -t 0 # 清理一下
tmux split-window -h -t 0
tmux split-window -v -t 0
tmux split-window -v -t 2

# Pane 0: Position
tmux select-pane -t 0
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "sleep 22s" C-m
tmux send-keys "rostopic echo /mavros/local_position/pose" C-m

# Pane 1: PX4
tmux select-pane -t 1
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "roslaunch mavros px4.launch" C-m

# Pane 2: Mid-360
tmux select-pane -t 2
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "astra" C-m
tmux send-keys "sleep 5s" C-m
tmux send-keys "roslaunch livox_ros_driver2 msg_MID360.launch" C-m

# Pane 3: A8mini
tmux select-pane -t 3
tmux send-keys "astra" C-m
tmux send-keys "sleep 12s" C-m
tmux send-keys "roslaunch fast_lio mapping_mid360.launch rviz:=false" C-m

# 进入会话
tmux -2 attach-session -t $SESSION
