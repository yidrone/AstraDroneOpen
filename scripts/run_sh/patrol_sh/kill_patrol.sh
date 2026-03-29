#!/bin/bash

# =================================================================
# 脚本名称: kill_nav_session.sh
# 功    能: 检查并终止名为 "nav" 的 tmux 会话及其所有内部进程。
#           通常用于停止导航相关的ROS节点。
# =================================================================

# 定义要操作的tmux会话的名称
SESSION_NAME="patrol"

# 检查指定的tmux会话是否存在
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    # 如果会话存在...
    echo "找到名为 '$SESSION_NAME' 的 tmux 会话，正在终止..."
    
    # 发送 kill-session 命令来终止整个会话
    tmux kill-session -t "$SESSION_NAME"
    
    echo "会话 '$SESSION_NAME' 及其所有内部进程已成功终止。"
else
    # 如果会话不存在...
    echo "未找到名为 '$SESSION_NAME' 的 tmux 会话。可能已经关闭或未曾启动。"
fi

exit 0
