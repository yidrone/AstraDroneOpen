#!/bin/bash
# ==========================================
# build_AstraDrone_ros1.sh  (shc 友好版)
# - 更稳的脚本目录解析（兼容 shc）
# - 基础健壮性检查
# - 原有参数/功能保持不变
# - 新增：--small_build 最小化编译（通过集中配置区批量 include/exclude）
# - 新增：clean_build[=true|false] 控制是否清理 build/ devel 后再编译（可与其他参数同时存在）
# - 规则更新：默认(无参)就编译；--state/list/help 单独出现不编译；与修改/构建类参数并存时继续编译
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

WORKSPACE_DIR="$SCRIPT_DIR/../AstraDrone_ros1_ws"
SRC_DIR="$WORKSPACE_DIR/src"
ORIG_ARGS=("$@")

# ---------- 基础检查 ----------
err()  { echo -e "❌ $*"; exit 1; }
warn() { echo -e "⚠️  $*"; }
info() { echo -e "ℹ️  $*"; }
safe_source_ros() {
  [[ -f /opt/ros/noetic/setup.bash ]] || return 1
  set +u
  set --
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  set -- "${ORIG_ARGS[@]}"
  set -u
}

[[ -d "$WORKSPACE_DIR" ]] || err "未找到工作空间目录：$WORKSPACE_DIR"
[[ -d "$SRC_DIR"       ]] || err "未找到 src 目录：$SRC_DIR"

# 先加载 ROS 环境，避免全新系统第一次运行时 PATH 中还没有 catkin_make。
if safe_source_ros; then
  :
else
  warn "/opt/ros/noetic/setup.bash 不存在，稍后 source 可能失败"
fi

command -v find        >/dev/null 2>&1 || err "未找到 find，请安装：sudo apt-get install -y findutils"
command -v catkin_make >/dev/null 2>&1 || err "未找到 catkin_make，请先安装 ROS 的 catkin 工具"

# =============== 工具函数 ===============
sorted_keys() { local __arr_name="$1"; eval "printf '%s\n' \"\${!$__arr_name[@]}\" | sort"; }

# 集合是否被忽略（递归检测集合内是否有屏蔽标记）
is_set_ignored() {
  local set_dir="$1"
  local cat_dir; cat_dir="$(dirname "$set_dir")"

  [[ -f "$SRC_DIR/CATKIN_IGNORE" ]] && return 0
  [[ -f "$cat_dir/CATKIN_IGNORE" ]] && return 0
  [[ -f "$set_dir/CATKIN_IGNORE" ]] && return 0

  local found=""
  if read -r found < <(find "$set_dir" -type f -name "CATKIN_IGNORE" -print -quit 2>/dev/null); then
    [[ -n "$found" ]] && return 0
  fi
  return 1
}

# =============== 扫描集合（src 的二级目录：<类别>/<集合>） ===============
declare -A PKG_SETS=()     # key: "Cat/Set" -> 绝对路径
declare -A SET_ALIAS=()    # key: "Set" -> 路径或 "__AMBIGUOUS__"

echo "扫描包集合目录..."
while IFS= read -r set_dir; do
  cat_name="$(basename "$(dirname "$set_dir")")"
  set_name="$(basename "$set_dir")"
  key="${cat_name}/${set_name}"
  PKG_SETS["$key"]="$set_dir"
  if [[ -z "${SET_ALIAS[$set_name]:-}" ]]; then
    SET_ALIAS["$set_name"]="$set_dir"
  elif [[ "${SET_ALIAS[$set_name]}" != "$set_dir" ]]; then
    SET_ALIAS["$set_name"]="__AMBIGUOUS__"
  fi
done < <(find "$SRC_DIR" -mindepth 2 -maxdepth 2 -type d | sort)

[[ ${#PKG_SETS[@]} -gt 0 ]] || err "未找到任何集合（应位于 src/<类别>/<集合>/）!"

# =============== 解析集合名（支持 Set 或 Cat/Set） ===============
resolve_set() {
  local input="$1"  # "Set" 或 "Cat/Set"
  if [[ "$input" == */* ]]; then
    [[ -n "${PKG_SETS[$input]:-}" ]] && { printf '%s\n' "${PKG_SETS[$input]}"; return 0; }
    echo "ERR:NOTFOUND"; return 1
  else
    local v="${SET_ALIAS[$input]:-}"
    if [[ -z "$v" ]]; then
      echo "ERR:NOTFOUND"; return 1
    elif [[ "$v" == "__AMBIGUOUS__" ]]; then
      echo "ERR:AMBIG"; return 2
    else
      printf '%s\n' "$v"; return 0
    fi
  fi
}

suggest_sets() {
  local name="$1"
  echo "可用集合（类别/集合）："
  for k in $(sorted_keys PKG_SETS); do
    if [[ "$k" == */"$name" || "$k" == *"$name"* ]]; then
      echo "  $k"
    fi
  done
}

# ======= Small Build（最小化编译）配置 =======
SMALL_BUILD_EXCLUDE_SETS=(
  "Communication/serial_tool"
  "Detection/aruco_localization"
  "Detection/target_prediction"
  "Utils/fake_slam"
  "Utils/imu_2_euler"
  "Utils/lidar_cam_fusion"
  "Utils/pixel2map"
  "Utils/pointcloudcut"
  "Utils/pub_cam_info"
  "Utils/rc_topic"
  "Utils/rtk2local"
  "Utils/serial"
  "Utils/C10Pro_sdk"
  "Utils/OrbbecSDK_ROS1"
  "Utils/camera_sdk"
  "Planner/ego-planner"
  "Detection/apriltag_ros"
  "Track/pix_tracker"
)

SMALL_BUILD_INCLUDE_SETS=(
  "MissionControl/astra_uavoffbard_frame"
  "SLAM/FAST_LIO"
)

run_small_build() {
  echo "============== Small Build（最小化编译）模式 =============="
  local changed=0

  if [[ ${#SMALL_BUILD_EXCLUDE_SETS[@]} -gt 0 ]]; then
    echo "将排除以下集合："
    for name in "${SMALL_BUILD_EXCLUDE_SETS[@]}"; do
      echo "  - $name"
      process_pkg_set "add" "$name" || true
      changed=1
    done
  else
    echo "（未配置 SMALL_BUILD_EXCLUDE_SETS，当前不会额外排除任何集合）"
  fi

  if [[ ${#SMALL_BUILD_INCLUDE_SETS[@]} -gt 0 ]]; then
    echo "将强制包含以下集合："
    for name in "${SMALL_BUILD_INCLUDE_SETS[@]}"; do
      echo "  - $name"
      process_pkg_set "remove" "$name" || true
      changed=1
    done
  fi

  [[ $changed -eq 0 ]] && warn "small_build 未对编译清单做任何改动，你可能需要在配置区填写集合名。"
  echo "========================================================="
}

# =============== 帮助、列表、状态 ===============
show_help() {
  echo "用法: $0 [选项] [clean_build[=true|false]]"
  echo "选项:"
  echo "  --include <集合>          启用该集合"
  echo "  --exclude <集合>          禁用该集合"
  echo "  --list                    列出所有 可用集合（类别/集合），然后退出"
  echo "  --state                   显示当前编译状态（会/不会编译）"
  echo "  --small_build             项目初始最小编译，仅包含演示例程相关包"
  echo "  --help                    显示此帮助信息"
  echo "  clean_build               加上此参数，会删除工作空间历史编译文件后再编译（可与上面参数共存）"
  echo
  echo "示例："
  echo "  $0                                 # 默认直接编译（增量）"
  echo "  $0 clean_build                     # 清理后全量编译"
  echo "  $0 --include Planner/ego-planner --state"
  echo "  $0 --exclude Planner/ego-planner clean_build"
  echo "  $0 --small_build clean_build=false # 最小化编译（增量）"
  echo "  $0 --state                         # 仅查看状态，不编译"
  echo
}

show_list() {
  echo "可用集合（两级：类别/集合）："
  declare -A cats=()
  for k in "${!PKG_SETS[@]}"; do cats["${k%/*}"]=1; done
  for cat in $(printf '%s\n' "${!cats[@]}" | sort); do
    echo "  ${cat}/"
    for k in $(sorted_keys PKG_SETS); do
      [[ "${k%/*}" == "$cat" ]] || continue
      echo "    $k"
    done
  done
  echo
}

collect_state_sets() {
  echo "====== 当前编译状态（集合级, 类别/集合） ======"
  declare -A cats=()
  for k in "${!PKG_SETS[@]}"; do cats["${k%/*}"]=1; done

  echo "将会编译的功能包集合："
  local any_build=0
  for cat in $(printf '%s\n' "${!cats[@]}" | sort); do
    local printed=0
    for k in $(sorted_keys PKG_SETS); do
      [[ "${k%/*}" == "$cat" ]] || continue
      set_path="${PKG_SETS[$k]}"
      if ! is_set_ignored "$set_path"; then
        if [[ $printed -eq 0 ]]; then echo "  ${cat}/"; printed=1; fi
        echo "    $k"
        any_build=1
      fi
    done
  done
  [[ $any_build -eq 0 ]] && echo "  (无)"

  echo
  echo "不会编译的功能包集合："
  local any_skip=0
  for cat in $(printf '%s\n' "${!cats[@]}" | sort); do
    local printed=0
    for k in $(sorted_keys PKG_SETS); do
      [[ "${k%/*}" == "$cat" ]] || continue
      set_path="${PKG_SETS[$k]}"
      if is_set_ignored "$set_path"; then
        if [[ $printed -eq 0 ]]; then echo "  ${cat}/"; printed=1; fi
        echo "    $k"
        any_skip=1
      fi
    done
  done
  [[ $any_skip -eq 0 ]] && echo "  (无)"
  echo "==========================================="
}

# =============== include/exclude 处理 ===============
process_pkg_set() {
  local action="$1"   # add / remove
  local name="$2"     # Set 或 Cat/Set

  local set_path
  if ! set_path="$(resolve_set "$name")"; then
    rc=$?
    if [[ $rc -eq 2 ]]; then
      echo "错误: 集合名 '$name' 在多个类别下存在："
      suggest_sets "$name"
    else
      echo "错误: 未找到集合 '$name'。你可以尝试以下匹配："
      suggest_sets "$name"
    fi
    exit 1
  fi

  [[ -x "$SCRIPT_DIR/env_sh/ignore_packages.sh" ]] || [[ -f "$SCRIPT_DIR/env_sh/ignore_packages.sh" ]] || \
    err "缺少工具脚本：$SCRIPT_DIR/env_sh/ignore_packages.sh"

  echo "处理集合: $name ($set_path)"
  "$SCRIPT_DIR/env_sh/ignore_packages.sh" \
      "$WORKSPACE_DIR" \
      "$set_path" \
      "$action" \
      "CATKIN_IGNORE"
}

# =============== 默认操作（无参数时就编译，不改集合启停） ===============
default_operation() {
  info "未指定参数，将按当前工作空间启停状态直接编译。"
}

# =============== 参数解析 ===============
DO_SMALL_BUILD=0
DO_STATE=0
DO_BUILD=0          # 是否执行编译
CLEAN_BUILD=false   # 默认不清理，增量编译

if [[ $# -eq 0 ]]; then
  default_operation
  DO_BUILD=1                 # 无参默认编译
else
  while [[ $# -gt 0 ]]; do
    case $1 in
      --include)
        [[ -z ${2:-} ]] && { echo "错误: --include 需要参数"; exit 1; }
        process_pkg_set "remove" "$2"
        DO_BUILD=1
        shift 2
        ;;
      --exclude)
        [[ -z ${2:-} ]] && { echo "错误: --exclude 需要参数"; exit 1; }
        process_pkg_set "add" "$2"
        DO_BUILD=1
        shift 2
        ;;
      --list)
        show_list
        exit 0
        ;;
      --state)
        DO_STATE=1
        shift 1
        ;;
      --small_build)
        DO_SMALL_BUILD=1
        DO_BUILD=1
        shift 1
        ;;
      clean_build)
        if [[ -n ${2:-} && "${2,,}" =~ ^(true|false|1|0)$ ]]; then
          [[ "${2,,}" =~ ^(true|1)$ ]] && CLEAN_BUILD=true || CLEAN_BUILD=false
          shift 2
        else
          CLEAN_BUILD=true
          shift 1
        fi
        DO_BUILD=1   # clean_build 单独出现也应编译
        ;;
      clean_build=*)
        val="${1#clean_build=}"; val="${val,,}"
        if   [[ "$val" =~ ^(true|1)$ ]]; then CLEAN_BUILD=true
        elif [[ "$val" =~ ^(false|0)$ ]]; then CLEAN_BUILD=false
        else warn "clean_build 的值无效：'$val'，已按 true 处理"; CLEAN_BUILD=true
        fi
        DO_BUILD=1
        shift 1
        ;;
      --help|-h)
        show_help
        exit 0
        ;;
      *)
        echo "未知选项: $1"
        show_help
        exit 1
        ;;
    esac
  done
fi

# small_build：先批量调整集合
if [[ $DO_SMALL_BUILD -eq 1 ]]; then
  run_small_build
fi

# --state：显示状态；若本次没有要编译的动作，则退出；否则继续编译
if [[ $DO_STATE -eq 1 ]]; then
  collect_state_sets
  if [[ $DO_BUILD -eq 0 ]]; then
    exit 0
  fi
fi

# 没有需要编译则退出（理论上此时仅会出现在 --list/--help 情况）
if [[ $DO_BUILD -eq 0 ]]; then
  exit 0
fi

# =============== 构建操作（DO_BUILD=1） ===============
echo -e "\n开始构建..."
cd "$WORKSPACE_DIR"

# ROS 环境
if safe_source_ros; then
  :
else
  warn "/opt/ros/noetic/setup.bash 不存在，可能导致 catkin 环境未初始化。"
fi

# 清理旧构建（只在 clean_build=true 时）
if [[ "$CLEAN_BUILD" == true ]]; then
  echo "🧹 清理旧的 build/ devel ..."
  rm -rf build devel
else
  echo "⚡ 增量编译模式（保留 build/ devel）"
fi

# 复制开发文件（仅在需要时执行，避免触发不必要的重编译）
if [[ "$CLEAN_BUILD" == true || ! -d "$WORKSPACE_DIR/devel/include" ]]; then
  mkdir -p "$WORKSPACE_DIR/devel/"
  if command -v rsync >/dev/null 2>&1; then
    rsync -a --delete --checksum \
      "$WORKSPACE_DIR/src/Utils/devels/include/" \
      "$WORKSPACE_DIR/devel/include/" 2>/dev/null || true
  else
    cp -aru "$WORKSPACE_DIR/src/Utils/devels/include" "$WORKSPACE_DIR/devel/" 2>/dev/null || true
  fi
fi

# 🚀 第一步：只编译指定包
if catkin_make --pkg astra_custom_msgs cv_bridge; then
  echo "✅ 消息包 astra_custom_msgs 构建成功！"
else
  err "消息包构建失败, 请检查错误。"
fi

# 🚀 第二步：构建全部包
echo -e "\n🚀 构建全部 ROS 包..."
if catkin_make; then
  echo -e "\n✅ AstraDrone_ros1_ws 构建成功！"
else
  err "AstraDrone_ros1_ws 构建失败！请检查错误。"
fi

# =============== 第三步：添加快捷指令 astra ===============
echo -e "\n🚀 添加快捷指令..."
ALIAS_CMD="alias astra='source $WORKSPACE_DIR/devel/setup.bash'"
if grep -Fxq "$ALIAS_CMD" ~/.bashrc; then
  echo "✅ 快捷指令 astra 已存在于 ~/.bashrc"
else
  echo "$ALIAS_CMD" >> ~/.bashrc
  echo "✅ 已添加 astra 快捷指令到 ~/.bashrc, 你可以使用 'astra' 一键配置ROS环境"
fi
