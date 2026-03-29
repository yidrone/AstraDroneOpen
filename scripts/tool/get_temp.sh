#!/usr/bin/env bash

shopt -s nocasematch
shopt -s nullglob

# 将 /sys 中的温度值转换成摄氏度
to_celsius() {
    local raw="$1"

    if ! [[ "$raw" =~ ^[0-9]+$ ]]; then
        echo "N/A"
        return
    fi

    if (( raw >= 1000 )); then
        awk "BEGIN { printf \"%.1f\", $raw/1000 }"
    else
        awk "BEGIN { printf \"%.1f\", $raw }"
    fi
}

# 和“上次通用版”一样：从 thermal_zone 推测 CPU/GPU
get_cpu_gpu_temp_from_thermal() {
    local cpu=""; local gpu=""

    for temp_file in /sys/class/thermal/thermal_zone*/temp; do
        [[ -e "$temp_file" ]] || continue

        local dir type raw temp
        dir="$(dirname "$temp_file")"
        [[ -e "$dir/type" ]] || continue

        type=$(cat "$dir/type" 2>/dev/null)
        raw=$(cat "$temp_file" 2>/dev/null)
        temp=$(to_celsius "$raw")

        # CPU 候选：包含 cpu / x86_pkg_temp / soc 等
        if [[ -z "$cpu" ]] && { \
            [[ "$type" == *cpu* ]] || \
            [[ "$type" == *x86_pkg_temp* ]] || \
            [[ "$type" == *soc* ]]; }; then
            cpu="$temp"
        fi

        # GPU 候选：包含 gpu
        if [[ -z "$gpu" ]] && [[ "$type" == *gpu* ]]; then
            gpu="$temp"
        fi
    done

    echo "$cpu" "$gpu"
}

# 和“上次通用版”一样：优先用 nvidia-smi 获取 GPU 温度（如果有）
get_gpu_temp_nvidia() {
    if command -v nvidia-smi >/dev/null 2>&1; then
        local t
        t=$(nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader,nounits 2>/dev/null | head -n1)
        if [[ "$t" =~ ^[0-9]+$ ]]; then
            awk "BEGIN { printf \"%.1f\", $t }"
            return
        fi
    fi
    echo ""
}

# 用 tput 移动光标刷新某一行（不会整屏闪）
update_line() {
    local row="$1"
    local text="$2"
    tput cup "$row" 0
    printf "%s\033[K" "$text"
}

cleanup() {
    tput cnorm  # 恢复光标
    echo
    exit 0
}

trap cleanup INT TERM

clear
tput civis  # 隐藏光标

echo "====== Ubuntu Universal Temperature (smooth) ======"
echo                                                  # 行1：CPU
echo                                                  # 行2：GPU
echo                                                  # 行3：空行
echo                                                  # 行4：时间
echo "------------------------------------------------"

# 主循环
while true; do
    # 通用版逻辑：先从 thermal_zone 推测
    read cpu_guess gpu_guess < <(get_cpu_gpu_temp_from_thermal)

    # 有 NVIDIA 的话，用 nvidia-smi 覆盖 GPU 温度
    nvidia_gpu_temp=$(get_gpu_temp_nvidia)
    if [[ -n "$nvidia_gpu_temp" ]]; then
        gpu_guess="$nvidia_gpu_temp"
    fi

    [[ -z "$cpu_guess" ]] && cpu_guess="N/A"
    [[ -z "$gpu_guess" ]] && gpu_guess="N/A"

    update_line 1 "CPU Temperature (guess) : ${cpu_guess}°C"
    update_line 2 "GPU Temperature (guess) : ${gpu_guess}°C"
    update_line 3 ""
    update_line 4 "更新于：$(date '+%Y-%m-%d %H:%M:%S')"

    sleep 1
done

