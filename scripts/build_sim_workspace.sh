#!/bin/bash
# ==========================================
# build_AstraDrone_ros1.sh  (shc 友好版 + 依赖自动安装)
# ==========================================

set -Eeuo pipefail

# ---------- 稳健解析脚本所在目录（兼容 shc / set -u） ----------
_src="${BASH_SOURCE[0]:-$0}"
if command -v readlink >/dev/null 2>&1; then
  _src="$(readlink -f "$_src" 2>/dev/null || echo "$_src")"
elif command -v realpath >/dev/null 2>&1; then
  _src="$(realpath "$_src" 2>/dev/null || echo "$_src")"
fi
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$_src")" >/dev/null 2>&1 && pwd -P)"

# ---------- ROS 环境安全 source（兼容 set -u） ----------
ROS_SETUP="/opt/ros/noetic/setup.bash"
safe_source_ros() {
  if [[ -f "$ROS_SETUP" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "$ROS_SETUP"
    source ~/.bashrc   
    set -u
  else
    echo "❌ 未找到 ROS Noetic：$ROS_SETUP"
    exit 1
  fi
}

# 仿真工作空间与源码
WORKSPACE_DIR="$SCRIPT_DIR/../simulation/sim_workspace"
SRC_DIR="$WORKSPACE_DIR/src"

ASTRA_MODELS_PATH="$(realpath "$SCRIPT_DIR/../simulation/astra_gazebo_models")"
PX4_SIM_DIR="$(realpath "$SCRIPT_DIR/../simulation/px4_sim_files")"
PX4_DIR="$(realpath "$SCRIPT_DIR/../../PX4-Autopilot")"

AIRFRAMES_DIR="$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/airframes"
GAZEBO_MODELS_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
PX4_LAUNCH_DIR="$PX4_DIR/launch"

PX4_IRIS_PARAMS_DIR="$PX4_SIM_DIR/px4_iris_params"
PX4_IRIS_SDF_DIR="$PX4_SIM_DIR/px4_iris_sdf"
PX4_LAUNCH_SRC_DIR="$PX4_SIM_DIR/px4_launch"

export https_proxy=http://127.0.0.1:7890
export http_proxy=http://127.0.0.1:7890
export all_proxy=socks5://127.0.0.1:7890

# ---------- 打印路径 ----------
echo -e "\n🔎 路径探测："
echo "SCRIPT_DIR        = $SCRIPT_DIR"
echo "WORKSPACE_DIR     = $WORKSPACE_DIR"
echo "PX4_SIM_DIR       = $PX4_SIM_DIR"
echo "PX4_DIR           = $PX4_DIR"

[[ -d "$PX4_DIR" ]] || { echo "❌ PX4-Autopilot 不存在"; exit 1; }

mkdir -p "$AIRFRAMES_DIR" "$GAZEBO_MODELS_DIR" "$PX4_LAUNCH_DIR"

# ---------- 立即刷新 ROS 环境 ----------
safe_source_ros

# =====================================================================
# 依赖安装
# =====================================================================
echo -e "\n🧰 依赖检查与安装..."
export DEBIAN_FRONTEND=noninteractive

retry() {
  local n=$1; shift
  for ((i=0;i<n;i++)); do "$@" && return 0 || sleep 2; done
  return 1
}

need_cmd() { command -v "$1" >/dev/null 2>&1; }

apt_install() {
  retry 2 sudo apt-get update
  retry 2 sudo apt-get install -y "$@"
}

GENERAL_PKGS=(build-essential cmake git curl unzip pkg-config)
PY_PKGS=(python3 python3-pip python3-venv)
SYS_PKGS=(lsb-release ca-certificates gnupg)

MISSING=()
for c in gcc g++ cmake git python3 pip3; do need_cmd "$c" || MISSING+=("$c"); done
((${#MISSING[@]})) && apt_install "${GENERAL_PKGS[@]}" "${PY_PKGS[@]}" "${SYS_PKGS[@]}"

echo "   ✅ 检测到 ROS Noetic，安装仿真控制依赖..."
apt_install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-ackermann-msgs
apt_install libeigen3-dev libyaml-cpp-dev || true
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
# ---------- rosdep ----------
# if need_cmd rosdep; then
#   safe_source_ros
#   sudo rosdep init 2>/dev/null || true
#   rosdep update
#   [[ -d "$SRC_DIR" ]] && (cd "$WORKSPACE_DIR" && rosdep install --from-paths src --ignore-src -r -y || true)
# fi

# =====================================================================
# 构建
# =====================================================================
echo -e "\n🚧 开始构建..."
cd "$WORKSPACE_DIR"
safe_source_ros

rm -rf build devel
mkdir -p devel/include

if [[ -d src/sensors/lidar_simulator/devels/livox_laser_simulation ]]; then
  cp -r src/sensors/lidar_simulator/devels/livox_laser_simulation devel/include
fi

catkin_make

# =====================================================================
# bashrc & Gazebo
# =====================================================================
ALIAS_CMD="source $WORKSPACE_DIR/devel/setup.bash"
PX4_KEYWORD="PX4 Auto-Config"

BASHRC="$HOME/.bashrc"

# 如果已经存在，什么都不做
if grep -Fxq "$ALIAS_CMD" "$BASHRC"; then
  echo "✅ astrasim 已存在于 ~/.bashrc"
else
  if grep -q "$PX4_KEYWORD" "$BASHRC"; then
    # 找到包含 PX4 Auto-Config 的第一行行号
    line_num=$(grep -n "$PX4_KEYWORD" "$BASHRC" | head -n 1 | cut -d: -f1)

    # 在该行“上一行”插入
    sed -i "${line_num}i $ALIAS_CMD" "$BASHRC"

    echo "✅ 已将 astrasim 插入到 PX4 Auto-Config 之前"
  else
    # 兜底：如果没找到 PX4 Auto-Config，就追加到末尾
    echo "$ALIAS_CMD" >> "$BASHRC"
    echo "ℹ️ 未找到 PX4 Auto-Config, 已追加到 ~/.bashrc 末尾"
  fi
fi

safe_source_ros
# =============== 添加快捷指令 astra ===============
echo -e "\n🚀 添加pc_example快捷指令..."
ALIAS_CMD="alias pc_example='$SCRIPT_DIR/run_sh/pc_example.sh '"
if grep -Fxq "$ALIAS_CMD" ~/.bashrc; then
  echo "✅ 快捷指令 pc_example 已存在于 ~/.bashrc"
else
  echo "$ALIAS_CMD" >> ~/.bashrc
  echo "✅ 已添加 pc_example 快捷指令到 ~/.bashrc"
fi
safe_source_ros

grep -Fq -- "$ASTRA_MODELS_PATH" ~/.bashrc || {
  echo "# AstraDroneOpen Gazebo Models" >> ~/.bashrc
  echo "export GAZEBO_MODEL_PATH=\"\${GAZEBO_MODEL_PATH:+\${GAZEBO_MODEL_PATH}:}$ASTRA_MODELS_PATH\"" >> ~/.bashrc
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+${GAZEBO_MODEL_PATH}:}$ASTRA_MODELS_PATH"
}

# =====================================================================
# PX4 文件同步
# =====================================================================
cp -v "$PX4_IRIS_PARAMS_DIR"/* "$AIRFRAMES_DIR"/ 2>/dev/null || true
cp -rv "$PX4_IRIS_SDF_DIR"/* "$GAZEBO_MODELS_DIR"/ 2>/dev/null || true
cp -rv "$PX4_LAUNCH_SRC_DIR"/* "$PX4_LAUNCH_DIR"/ 2>/dev/null || true

echo -e "\n🎉 完成：仿真环境已构建并立即生效"
