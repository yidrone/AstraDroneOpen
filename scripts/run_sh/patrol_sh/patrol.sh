#!/bin/bash
SESSION="patrol"

# 1. 重置会话
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -n "patrol"

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

# Pane 0: Auto Land
tmux select-pane -t 0
tmux send-keys "source ~/.bashrc && astra" C-m
tmux send-keys "sleep 2s" C-m
tmux send-keys "roslaunch astra_auto_land astra_static_land.launch" C-m

# Pane 1: Astra Control (核心)
tmux select-pane -t 1
tmux send-keys "source ~/.bashrc && astra" C-m
tmux send-keys "sleep 30s" C-m
tmux send-keys "roslaunch astra_control astra_control_real.launch" C-m

# Pane 4: apriltag_realvideo
tmux select-pane -t 2
tmux send-keys "source ~/.bashrc && astra" C-m
tmux send-keys "sleep 8s" C-m
tmux send-keys "roslaunch apriltag_ros apriltag_realvideo.launch" C-m

# Pane 3: camera
tmux select-pane -t 3
tmux send-keys "source ~/.bashrc && astra" C-m
tmux send-keys "sleep 15s" C-m
tmux send-keys "sh ~/AstraDroneOpen/scripts/start_camera_with_retry.sh" C-m

# 进入会话
tmux -2 attach-session -t $SESSION
