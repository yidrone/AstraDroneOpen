#!/bin/bash
SESSION="base"

# 1. 重置会话
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -n "Base_Layer"

# 重置布局为 4 格
tmux kill-pane -a -t 0 # 清理一下
tmux split-window -h -t 0
tmux split-window -v -t 0
tmux split-window -v -t 2

tmux split-window -v -t 3

# Pane 0: Position
tmux select-pane -t 0
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "sleep 25s" C-m
tmux send-keys "rostopic echo /mavros/local_position/pose" C-m

# Pane 1: PX4
tmux select-pane -t 1
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "roslaunch mavros px4.launch" C-m

# Pane 2: Mid-360
tmux select-pane -t 2
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "astra" C-m
tmux send-keys "sleep 16s" C-m
tmux send-keys "roslaunch livox_ros_driver2 msg_MID360.launch" C-m

# Pane 3: A8mini
tmux select-pane -t 3
tmux send-keys "sleep 5s" C-m
tmux send-keys "python /home/uav/AstraDroneOpen/AstraDrone_ros1_ws/src/Utils/siyi_A8mini_sdk/control.py" C-m

# Pane 4: A8mini
tmux select-pane -t 4
tmux send-keys "sleep 10s" C-m
tmux send-keys "python /home/uav/AstraDroneOpen/AstraDrone_ros1_ws/src/Utils/siyi_A8mini_sdk/siyi_camera_a8_mini/zoom_controller.py" C-m

# 进入会话
tmux -2 attach-session -t $SESSION
