#!/bin/bash

set -Eeuo pipefail

APT_UPDATED=0
SYSTEM_UPGRADED=0
ARCH=""
OS_ID=""
VERSION_CODENAME=""
PRETTY_NAME=""
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/astradrone"
MIRROR_CACHE_DIR="$CACHE_DIR/mirror_latency"
MIRROR_CACHE_TTL=21600

if [[ -t 1 ]]; then
    COLOR_RESET=$'\033[0m'
    COLOR_BOLD=$'\033[1m'
    COLOR_BLUE=$'\033[34m'
    COLOR_GREEN=$'\033[32m'
    COLOR_YELLOW=$'\033[33m'
    COLOR_RED=$'\033[31m'
    COLOR_CYAN=$'\033[36m'
    COLOR_MAGENTA=$'\033[35m'
else
    COLOR_RESET=""
    COLOR_BOLD=""
    COLOR_BLUE=""
    COLOR_GREEN=""
    COLOR_YELLOW=""
    COLOR_RED=""
    COLOR_CYAN=""
    COLOR_MAGENTA=""
fi

log_step() {
    printf '%s[步骤]%s %s\n' "${COLOR_BOLD}${COLOR_BLUE}" "$COLOR_RESET" "$*"
}

log_info() {
    printf '%s[信息]%s %s\n' "$COLOR_CYAN" "$COLOR_RESET" "$*"
}

log_success() {
    printf '%s[完成]%s %s\n' "$COLOR_GREEN" "$COLOR_RESET" "$*"
}

log_warn() {
    printf '%s[注意]%s %s\n' "$COLOR_YELLOW" "$COLOR_RESET" "$*"
}

log_error() {
    printf '%s[错误]%s %s\n' "$COLOR_RED" "$COLOR_RESET" "$*" >&2
}

log_probe() {
    printf '%s[测速]%s %s\n' "$COLOR_MAGENTA" "$COLOR_RESET" "$*" >&2
}

retry() {
    local attempts="$1"
    shift
    local i
    for ((i = 1; i <= attempts; i++)); do
        if "$@"; then
            return 0
        fi
        sleep 2
    done
    return 1
}

invalidate_apt_cache() {
    APT_UPDATED=0
}

ensure_sudo() {
    if sudo -n true 2>/dev/null; then
        log_success "sudo 权限已就绪，继续执行"
    else
        log_warn "接下来需要 sudo 权限，请输入密码"
        if ! sudo -v; then
            log_error "sudo 校验失败"
            exit 1
        fi
    fi
}

apt_update() {
    if [[ "$APT_UPDATED" -eq 0 ]]; then
        retry 2 sudo apt-get update
        APT_UPDATED=1
    fi
}

apt_install() {
    apt_update
    retry 2 sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y "$@"
}

system_upgrade_once() {
    if [[ "$SYSTEM_UPGRADED" -eq 0 ]]; then
        apt_update
        retry 2 sudo env DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
        retry 2 sudo env DEBIAN_FRONTEND=noninteractive apt-get dist-upgrade -y
        SYSTEM_UPGRADED=1
    fi
}

safe_source_file() {
    local target="$1"
    [[ -f "$target" ]] || return 0
    set +u
    # shellcheck disable=SC1090
    source "$target"
    set -u
}

load_os_info() {
    # shellcheck disable=SC1091
    source /etc/os-release
    OS_ID="${ID:-}"
    VERSION_CODENAME="${VERSION_CODENAME:-${UBUNTU_CODENAME:-}}"
    PRETTY_NAME="${PRETTY_NAME:-$ID}"
    ARCH="$(dpkg --print-architecture)"

    log_step "识别系统与架构信息"
    log_info "当前系统：$PRETTY_NAME"
    log_info "检测到架构：$ARCH"
    case "$ARCH" in
        amd64)
            log_info "镜像策略：x86_64 标准 Ubuntu 软件源"
            ;;
        arm64)
            log_info "镜像策略：ARM64 使用 ubuntu-ports 软件源"
            ;;
        armhf)
            log_info "镜像策略：ARMHF 使用 ubuntu-ports 软件源"
            ;;
        *)
            log_warn "镜像策略：未知非 amd64 架构，将尝试 ubuntu-ports 通用方案"
            ;;
    esac
}

cache_key_for_ranking() {
    local endpoint="$1"
    shift
    if command -v sha256sum >/dev/null 2>&1; then
        {
            printf '%s\n' "$ARCH"
            printf '%s\n' "$OS_ID"
            printf '%s\n' "$VERSION_CODENAME"
            printf '%s\n' "$endpoint"
            printf '%s\n' "$@"
        } | sha256sum | awk '{print $1}'
    else
        {
            printf '%s\n' "$ARCH"
            printf '%s\n' "$OS_ID"
            printf '%s\n' "$VERSION_CODENAME"
            printf '%s\n' "$endpoint"
            printf '%s\n' "$@"
        } | cksum | awk '{print $1}'
    fi
}

cache_age_seconds() {
    local cache_file="$1"
    [[ -f "$cache_file" ]] || return 1

    local now=""
    local modified=""
    now="$(date +%s)"
    modified="$(stat -c %Y "$cache_file" 2>/dev/null || true)"
    [[ -n "$modified" ]] || return 1
    echo $((now - modified))
}

read_cached_ranking() {
    local cache_file="$1"
    local age=""
    age="$(cache_age_seconds "$cache_file")" || return 1
    if (( age > MIRROR_CACHE_TTL )); then
        return 1
    fi

    log_probe "命中测速缓存，缓存年龄 ${age}s：$cache_file"
    awk -F '\t' 'NF >= 2 {print $2}' "$cache_file"
}

write_cached_ranking() {
    local cache_file="$1"
    local measured="$2"
    mkdir -p "$MIRROR_CACHE_DIR"
    printf '%s' "$measured" | sort -n -k1,1 > "$cache_file"
}

measure_url_latency() {
    local test_url="$1"
    local timeout="${2:-1.5}"

    python3 - "$test_url" "$timeout" <<'PY'
import http.client
import ssl
import sys
import time
from urllib.parse import urlparse

test_url = sys.argv[1]
timeout = float(sys.argv[2])
parsed = urlparse(test_url)
path = parsed.path or "/"
if parsed.query:
    path = f"{path}?{parsed.query}"

start = time.time()
try:
    if parsed.scheme == "https":
        conn = http.client.HTTPSConnection(
            parsed.netloc,
            timeout=timeout,
            context=ssl.create_default_context(),
        )
    else:
        conn = http.client.HTTPConnection(parsed.netloc, timeout=timeout)
    conn.request(
        "GET",
        path,
        headers={"User-Agent": "AstraDroneMirrorProbe/1.0", "Cache-Control": "no-cache"},
    )
    response = conn.getresponse()
    if response.status >= 400:
        raise RuntimeError(response.status)
    response.read(1)
    conn.close()
    print(f"{time.time() - start:.6f}")
except Exception:
    sys.exit(1)
PY
}

rank_mirrors_by_latency() {
    local endpoint="$1"
    shift
    local candidates=("$@")
    local cache_key=""
    local cache_file=""

    if ! command -v python3 >/dev/null 2>&1; then
        printf '%s\n' "${candidates[@]}"
        return 0
    fi

    mkdir -p "$MIRROR_CACHE_DIR"
    cache_key="$(cache_key_for_ranking "$endpoint" "${candidates[@]}")"
    cache_file="$MIRROR_CACHE_DIR/${cache_key}.tsv"
    if read_cached_ranking "$cache_file"; then
        return 0
    fi

    local measured=""
    local mirror=""
    local latency=""
    local probe_url=""

    log_probe "开始探测镜像延迟"
    for mirror in "${candidates[@]}"; do
        probe_url="${mirror%/}${endpoint}"
        printf '%s[测速]%s %-55s' "$COLOR_MAGENTA" "$COLOR_RESET" "$mirror" >&2
        if latency="$(measure_url_latency "$probe_url" 1.5)"; then
            printf ' %ss\n' "$latency" >&2
            measured+="${latency}"$'\t'"${mirror}"$'\n'
        else
            printf ' 超时\n' >&2
        fi
    done

    if [[ -z "$measured" ]]; then
        log_warn "镜像测速失败，保持原始顺序"
        printf '%s\n' "${candidates[@]}"
        return 0
    fi

    write_cached_ranking "$cache_file" "$measured"
    printf '%s' "$measured" | sort -n -k1,1 | cut -f2-
}

get_ubuntu_mirror_candidates() {
    if [[ "$ARCH" == "amd64" ]]; then
        cat <<'EOF'
https://mirrors.tuna.tsinghua.edu.cn/ubuntu
https://mirror.sysu.edu.cn/ubuntu
https://mirrors.ustc.edu.cn/ubuntu
https://archive.ubuntu.com/ubuntu
http://mirrors.tuna.tsinghua.edu.cn/ubuntu
http://mirror.sysu.edu.cn/ubuntu
http://mirrors.ustc.edu.cn/ubuntu
http://archive.ubuntu.com/ubuntu
EOF
    else
        cat <<'EOF'
https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
https://mirror.sysu.edu.cn/ubuntu-ports
https://ports.ubuntu.com/ubuntu-ports
http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
http://mirror.sysu.edu.cn/ubuntu-ports
http://ports.ubuntu.com/ubuntu-ports
EOF
    fi
}

get_ros_mirror_candidates() {
    cat <<'EOF'
http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu
https://mirrors.ustc.edu.cn/ros/ubuntu
https://repo.huaweicloud.com/ros/ubuntu
http://mirrors.cernet.edu.cn/ros/ubuntu
http://packages.ros.org/ros/ubuntu
EOF
}

check_ros_installed() {
    if [[ -f "/opt/ros/noetic/setup.bash" ]]; then
        log_success "检测到 ROS Noetic 已安装，跳过安装"
        return 0
    fi
    log_info "未检测到 ROS Noetic，准备安装"
    return 1
}

check_vscode_installed() {
    if command -v code >/dev/null 2>&1; then
        log_success "检测到 VSCode 已安装，跳过安装"
        return 0
    fi
    log_info "未检测到 VSCode，准备安装"
    return 1
}

require_supported_ros_os() {
    if [[ "$OS_ID" != "ubuntu" || "$VERSION_CODENAME" != "focal" ]]; then
        log_error "当前 PC 安装脚本仅面向 Ubuntu 20.04 (focal) + ROS Noetic"
        log_error "当前系统：$PRETTY_NAME"
        exit 1
    fi
}

backup_third_party_sources() {
    local backup_dir="/etc/apt/sources.list.d/astradrone-disabled"
    sudo mkdir -p "$backup_dir"

    local source_file
    shopt -s nullglob
    for source_file in /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do
        case "$source_file" in
            "$backup_dir"/*)
                continue
                ;;
        esac
        sudo mv "$source_file" "$backup_dir"/
    done
    shopt -u nullglob
}

write_ubuntu_sources() {
    local mirror="$1"
    local suites=(
        "$VERSION_CODENAME"
        "${VERSION_CODENAME}-updates"
        "${VERSION_CODENAME}-backports"
        "${VERSION_CODENAME}-security"
    )
    local components="main restricted universe multiverse"

    sudo tee /etc/apt/sources.list >/dev/null <<EOF
deb ${mirror%/}/ ${suites[0]} $components
deb ${mirror%/}/ ${suites[1]} $components
deb ${mirror%/}/ ${suites[2]} $components
deb ${mirror%/}/ ${suites[3]} $components
EOF
}

configure_system_sources() {
    local backup_file="/etc/apt/sources.list.astradrone.bak"
    local endpoint="/dists/${VERSION_CODENAME}/main/binary-${ARCH}/Packages.gz"
    local mirrors=()

    log_step "配置 Ubuntu 软件源"
    if [[ "$ARCH" == "amd64" ]]; then
        log_info "当前使用 amd64 软件源分支"
    else
        log_info "当前使用 ubuntu-ports 软件源分支，目标架构：$ARCH"
    fi
    [[ -f /etc/apt/sources.list ]] && sudo cp /etc/apt/sources.list "$backup_file"
    backup_third_party_sources

    mapfile -t mirrors < <(get_ubuntu_mirror_candidates)
    mapfile -t mirrors < <(rank_mirrors_by_latency "$endpoint" "${mirrors[@]}")

    local mirror
    for mirror in "${mirrors[@]}"; do
        log_info "尝试 Ubuntu 镜像：$mirror"
        write_ubuntu_sources "$mirror"
        invalidate_apt_cache
        if retry 2 sudo apt-get update; then
            APT_UPDATED=1
            log_success "Ubuntu 软件源可用：$mirror"
            return 0
        fi
    done

    if [[ -f "$backup_file" ]]; then
        sudo cp "$backup_file" /etc/apt/sources.list
        invalidate_apt_cache
    fi
    log_error "Ubuntu 软件源配置失败"
    return 1
}

configure_ros_repository() {
    local keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
    local key_url="https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc"
    local endpoint="/dists/${VERSION_CODENAME}/main/binary-${ARCH}/Packages.gz"
    local ros_mirrors=()

    log_step "配置 ROS 软件源"
    log_info "ROS 目标架构：$ARCH"
    log_info "ROS 发行代号：$VERSION_CODENAME"
    apt_install curl gnupg2 ca-certificates lsb-release
    sudo rm -f "$keyring"
    curl -fsSL "$key_url" | sudo gpg --dearmor -o "$keyring"

    mapfile -t ros_mirrors < <(get_ros_mirror_candidates)
    mapfile -t ros_mirrors < <(rank_mirrors_by_latency "$endpoint" "${ros_mirrors[@]}")

    local mirror
    for mirror in "${ros_mirrors[@]}"; do
        log_info "尝试 ROS 镜像：$mirror"
        sudo tee /etc/apt/sources.list.d/ros1.list >/dev/null <<EOF
deb [arch=$ARCH signed-by=$keyring] ${mirror%/} $VERSION_CODENAME main
EOF
        invalidate_apt_cache
        if retry 2 sudo apt-get update; then
            APT_UPDATED=1
            log_success "ROS 软件源可用：$mirror"
            return 0
        fi
    done

    sudo rm -f /etc/apt/sources.list.d/ros1.list
    invalidate_apt_cache
    log_error "ROS 软件源配置失败"
    return 1
}

ensure_ros_bashrc() {
    if ! grep -qF "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    fi
}

install_ros_noetic_if_needed() {
    if check_ros_installed; then
        ensure_ros_bashrc
        return 0
    fi

    require_supported_ros_os
    configure_system_sources
    configure_ros_repository

    log_step "更新系统并安装基础工具"
    system_upgrade_once

    log_step "安装 ROS Noetic 桌面版"
    apt_install ros-noetic-desktop-full python3-catkin-tools python3-rosdep
    sudo rosdep init 2>/dev/null || true
    rosdep update || true
    ensure_ros_bashrc
}

install_vscode_if_needed() {
    if check_vscode_installed; then
        return 0
    fi

    case "$ARCH" in
        amd64|arm64|armhf)
            ;;
        *)
            log_warn "当前架构暂不支持自动安装 VSCode：$ARCH"
            return 0
            ;;
    esac

    log_step "安装 VSCode"
    apt_install wget gpg apt-transport-https
    sudo rm -f /usr/share/keyrings/microsoft.gpg
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /usr/share/keyrings/microsoft.gpg >/dev/null
    sudo tee /etc/apt/sources.list.d/vscode.list >/dev/null <<'EOF'
deb [arch=amd64,arm64,armhf signed-by=/usr/share/keyrings/microsoft.gpg] https://packages.microsoft.com/repos/code stable main
EOF
    invalidate_apt_cache
    apt_install code
}

add_aliases() {
    log_step "写入 AstraDrone 常用终端别名"
    local alias_header="# <<<<<<<astra<<<<<<<< (Added by 00_ubuntu_init.sh)"

    if grep -Fxq "$alias_header" ~/.bashrc; then
        log_success "终端别名已存在，跳过写入"
        return 0
    fi

    cat <<'EOF' >> ~/.bashrc

# <<<<<<<astra<<<<<<<< (Added by 00_ubuntu_init.sh)
alias cc='catkin_make -j8'
alias cb='catkin build -j8'
alias ss='source ./devel/setup.bash'
alias sb='source ~/.bashrc'
alias si='pkill rosmaster; pkill roscore; pkill gzserver; pkill gzclient; pkill gazebo'
alias sm='pkill rosmaster; pkill roscore; sleep 1; pkill -9 rosmaster; pkill -9 roscore'
alias sg='pkill gzserver; pkill gzclient; pkill gazebo; sleep 1; pkill -9 gzserver; pkill -9 gzclient; pkill -9 gazebo'
alias rc='roscore &'
alias cde='conda deactivate'

alias suav='ssh uav@192.168.33.2'
alias sxuav='ssh -X uav@192.168.33.2'
# >>>>>>>astra>>>>>>>> (End)
EOF
    log_success "终端别名写入完成"
}

main() {
    load_os_info
    ensure_sudo

    install_ros_noetic_if_needed

    log_step "安装项目依赖工具"
    system_upgrade_once
    apt_install \
        libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
        git cmake wget build-essential libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at \
        python3-dev python3-pip python3-venv shc \
        gdb clang-format libboost-all-dev \
        terminator htop tmux vim nano sshfs net-tools openssh-server mlocate libdw-dev libdwarf-dev libelf-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev

    install_vscode_if_needed

    log_step "配置 tmux"
    echo "set -g mouse on" > ~/.tmux.conf

    add_aliases
    safe_source_file ~/.bashrc

    log_success "环境初始化完成"
    log_info "ROS 验证命令：roscore"
    log_info "VSCode 验证命令：code"
    printf '%s[信息]%s 已写入的常用别名：\n' "$COLOR_CYAN" "$COLOR_RESET"
    printf '    cc -> catkin_make\n'
    printf '    cb -> catkin build\n'
    printf '    ss -> source setup.bash\n'
    printf '    sb -> source ~/.bashrc\n'
    printf '    si -> 停止 ROS 与 Gazebo\n'
    printf '    sm -> 强制停止 ROS\n'
    printf '    sg -> 强制停止 Gazebo\n'
    printf '    rc -> 后台启动 roscore\n'
    printf '    cde -> 退出 conda\n'
    printf '    suav -> ssh 到 uav@192.168.33.2\n'
    printf '    sxuav -> ssh -X 到 uav@192.168.33.2\n'
}

main "$@"
