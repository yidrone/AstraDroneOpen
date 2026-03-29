#!/bin/bash

# =================配置区域=================
# Launch文件绝对路径
LAUNCH_FILE="/home/yidrone/AstraDrone/AstraDrone_ros1_ws/src/SDK/src/uav_bridge/launch/start_sdk.launch"
# 工作空间 setup.bash 路径
WS_SETUP="/home/yidrone/AstraDrone/AstraDrone_ros1_ws/devel/setup.bash"
# ROS 基础环境 (根据你的系统调整，通常是 noetic 或 melodic)
ROS_SETUP="/opt/ros/noetic/setup.bash"
# =========================================

# 1. 加载环境变量
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    echo "[Error] ROS setup not found at $ROS_SETUP"
fi

if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
else
    echo "[Error] Workspace setup not found at $WS_SETUP"
    exit 1
fi

# 2. 定义启动命令
# 使用绝对路径启动 launch 文件
CMD="roslaunch $LAUNCH_FILE"

# 3. 获取运行模式参数 (默认值为 0)
# $1 是脚本的第一个参数
MODE=${1:-0}

echo "========================================"
echo "   Auto Start Script for UAV Bridge"
echo "   Launch File: start_sdk.launch"
echo "========================================"

sleep 20

if [ "$MODE" == "1" ]; then
    # --- 模式 0: 当前终端启动 ---
    echo "[INFO] Mode 0: Launching in CURRENT terminal..."
    # 直接执行命令
    $CMD

else
    # --- 模式 1: 新终端启动 ---
    echo "[INFO] Mode 1: Launching in a NEW terminal..."
    
    # gnome-terminal 命令解释:
    # --window: 打开新窗口
    # --title: 设置窗口标题
    # -- bash -c "...": 在新终端中执行的命令
    # 我们需要在新终端里再次 source 环境，因为新终端不会继承当前脚本的 source 效果
    gnome-terminal --window --title="UAV Bridge SDK" -- bash -c "source $WS_SETUP; $CMD; exec bash"
fi
