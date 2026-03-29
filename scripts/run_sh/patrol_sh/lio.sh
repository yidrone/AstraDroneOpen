#!/bin/bash 
# 强制杀掉旧会话
tmux kill-session -t lio 2>/dev/null

# 创建新会话
tmux new-session -d -s lio -n "LIO_Layer"

# 【关键修改】显式指定 -t lio:0 (发送给 lio 会话的第0个窗口)
tmux send-keys -t lio:0 "astra" C-m
tmux send-keys -t lio:0 "sleep 15" C-m  # <--- 在这里增加延时，让环境准备好
tmux send-keys -t lio:0 "roslaunch fast_lio mapping_mid360.launch rviz:=false" C-m

# 进入会话
tmux -2 attach-session -t lio
