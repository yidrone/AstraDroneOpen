#!/bin/bash
set -Eeuo pipefail
# -------- 稳健的脚本目录解析（避免 BASH_SOURCE 未绑定）--------
_src="${BASH_SOURCE[0]:-$0}"
if command -v readlink >/dev/null 2>&1; then
  _src="$(readlink -f "$_src" 2>/dev/null || echo "$_src")"
elif command -v realpath >/dev/null 2>&1; then
  _src="$(realpath "$_src" 2>/dev/null || echo "$_src")"
fi
SCRIPT_DIR="$(cd "$(dirname "$_src")" && pwd)"
cd "$SCRIPT_DIR"

SCRIPTS=(
  "./env_sh/00_env_ubuntu_init.sh"
  "./env_sh/01_env_px4_init_onboard.sh"
  "./env_sh/02_env_third_party_init.sh"
  "build_AstraDrone_ros1.sh"
)

# 颜色/符号
GREEN="\033[32m"; YELLOW="\033[33m"; RED="\033[31m"; BLUE="\033[34m"; NC="\033[0m"
OK="✅"; WARN="⚠️"; ERR="❌"; RUN="🚀"; INFO="ℹ️"

# ============ 输出横幅 ============
print_banner() {
  local title="AstraDrone"
  local divider="===================================================================="

  echo -e "${RUN} ${BLUE}${divider}${NC}"

  if command -v figlet >/dev/null 2>&1; then
    echo -e "${BLUE}"
    figlet -w 120 -f standard "$title"
    echo -e "${NC}"
  else
    echo -e "${BLUE}"
    echo "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓"
    printf "┃ %-54s ┃\n" "$title"
    echo "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛"
    echo -e "${NC}"
  fi

  echo -e "${OK} ${GREEN}开源项目：${NC}${YELLOW}${title}${NC}"
  echo -e "${INFO} ${GREEN}交流QQ群：${NC}${YELLOW}1059975794${NC}"
  echo -e "${INFO} ${GREEN}商业合作（微信）：${NC}${YELLOW}Luli2225893438${NC}"
  echo -e "${RUN} ${BLUE}${divider}${NC}"
}

# ============ 依赖检查与安装 ============
# 定义颜色输出函数
echo_err() { echo -e "${ERR} ${RED}$1${NC}"; }
echo_warn() { echo -e "${WARN} ${YELLOW}$1${NC}"; }
echo_info() { echo -e "${INFO} ${GREEN}$1${NC}"; }
echo_run() { echo -e "${RUN} ${BLUE}$1${NC}"; }

# 依赖安装函数
install_deps() {
  local missing=()
  for cmd in "$@"; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
      missing+=("$cmd")
    fi
  done
  
  if [ ${#missing[@]} -gt 0 ]; then
    echo_warn "安装缺失依赖: ${missing[*]}"
    sudo apt update && sudo apt install -y "${missing[@]}"
    if [ $? -ne 0 ]; then
      echo_err "依赖安装失败"
      exit 1
    fi
  fi
}

# 必需命令列表
required_cmds=(bash grep sed awk tr git)

# 安装必需依赖
install_deps "${required_cmds[@]}"

# 可选依赖 - realpath
if ! command -v realpath >/dev/null 2>&1; then
  echo_warn "安装可选依赖: realpath"
  sudo apt install -y coreutils
fi

# 新增：安装 figlet（艺术字工具）
if ! command -v figlet >/dev/null 2>&1; then
  echo_warn "安装可选依赖: figlet (用于打印 AstraDrone 艺术字)"
  sudo apt install -y figlet
fi

# 启动时展示横幅
print_banner
sleep 2

# ============ 网络检查提示 ============
echo_warn "本脚本后续操作需要正常连接到 GitHub, 请仔细检查网络设置!"
echo_warn "本脚本耗时较长(> 1h30min)，请注意中途的输入密码提示!"
sleep 3  # 停留3秒让用户看到提示

# ============ 执行指定脚本 ============
run_script() {
  local f="$1"
  local path="$SCRIPT_DIR/$f"

  # 若写的是 .sh 且存在同名 .bin，则优先执行 .bin
  if [[ "$path" == *.sh && -f "${path%.sh}.bin" ]]; then
    path="${path%.sh}.bin"
  fi

  if [[ ! -e "$path" ]]; then
    echo_err "找不到脚本：$f"
    exit 1
  fi

  echo_run "执行：$path"

  # 判断是否为 ELF 二进制（前4字节 = 0x7F 'E' 'L' 'F'）
  if [[ -x "$path" ]] && head -c4 "$path" 2>/dev/null | grep -q $'^\x7fELF'; then
    "$path"            # 直接执行二进制
  else
    bash "$path"       # 当作 shell 脚本执行
  fi

  echo_info "完成：$path"
}

# ============ 主流程 ============
for f in "${SCRIPTS[@]}"; do
  run_script "$f"
done

echo_info "全部脚本执行完成！可以新开终端输入以下命令，验证环境："
echo_info "~/AstraDroneOpen/scripts/run_sh/pc_example.sh"
# 启动时展示横幅
print_banner
sleep 2
