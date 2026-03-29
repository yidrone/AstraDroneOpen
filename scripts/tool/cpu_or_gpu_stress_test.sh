#!/bin/bash

# Jetson Orin Nano 压力测试与监控脚本
# 使用直接系统文件读取获取信息

# 默认参数
DURATION_MIN=${1:-10}  # 默认测试10分钟
LOG_FILE="stress_test_log.txt"
INTERVAL=2             # 采集间隔(秒)

# 检查必要的工具
check_tools() {
    if ! command -v stress-ng &> /dev/null; then
        echo "正在安装 stress-ng..."
        sudo apt-get update
        sudo apt-get install -y stress-ng
    fi
    
    if ! command -v glmark2 &> /dev/null; then
        echo "错误: 未找到 glmark2 工具"
        echo "请安装: sudo apt-get install glmark2"
        exit 1
    fi
}

# 获取系统信息函数
get_cpu_usage() {
    # 使用更可靠的方法获取CPU使用率
    echo $(top -bn1 | grep "Cpu(s)" | awk '{print 100 - $8}' | cut -d. -f1)
}

get_gpu_usage() {
    # 使用tegrastats获取GPU使用率
    local stats=$(timeout 1 tegrastats | grep -o "GR3D.*%" | awk '{print $2}' | tr -d '%')
    echo ${stats:-0}
}

get_cpu_temp() {
    # 获取CPU温度
    local temp=$(cat /sys/devices/virtual/thermal/thermal_zone*/temp 2>/dev/null | head -1 | awk '{print $1/1000}')
    echo ${temp:-0}
}

get_gpu_temp() {
    # 获取GPU温度
    local temp=$(cat /sys/devices/virtual/thermal/thermal_zone*/temp 2>/dev/null | tail -1 | awk '{print $1/1000}')
    echo ${temp:-0}
}

get_cpu_freq() {
    # 获取CPU频率
    local freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null | awk '{print $1/1000}')
    echo ${freq:-0}
}

get_gpu_freq() {
    # 尝试获取GPU频率
    local freq=$(cat /sys/devices/*gpu*/cur_freq 2>/dev/null | head -1 | awk '{print $1/1000000}')
    if [ -z "$freq" ]; then
        echo "N/A"
    else
        echo $freq
    fi
}

# 主函数
main() {
    echo "Jetson Orin Nano 压力测试开始" | tee $LOG_FILE
    echo "持续时间: $DURATION_MIN 分钟" | tee -a $LOG_FILE
    echo "日志文件: $LOG_FILE" | tee -a $LOG_FILE
    echo "==========================================" | tee -a $LOG_FILE
    echo "时间戳                CPU使用率 GPU使用率 CPU温度 GPU温度 CPU频率 GPU频率" | tee -a $LOG_FILE
    echo "==========================================" | tee -a $LOG_FILE
    
    check_tools
    
    # 获取CPU核心数
    CPU_CORES=$(nproc)
    
    # 启动压力测试
    echo "启动CPU压力测试 (使用 $CPU_CORES 个核心)..." | tee -a $LOG_FILE
    stress-ng --cpu $CPU_CORES --timeout $(($DURATION_MIN * 60)) &
    CPU_STRESS_PID=$!
    
    # 启动GPU压力测试 (在后台运行，不输出到终端)
    echo "启动GPU压力测试 (使用glmark2)..." | tee -a $LOG_FILE
    glmark2 --run-forever > /dev/null 2>&1 &
    GPU_STRESS_PID=$!
    
    # 等待一段时间让测试稳定
    sleep 5
    
    # 监控循环
    echo "开始监控系统状态..." | tee -a $LOG_FILE
    END_TIME=$((SECONDS + ($DURATION_MIN * 60)))
    
    while [ $SECONDS -lt $END_TIME ]; do
        TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
        CPU_USAGE=$(get_cpu_usage)
        GPU_USAGE=$(get_gpu_usage)
        CPU_TEMP=$(get_cpu_temp)
        GPU_TEMP=$(get_gpu_temp)
        CPU_FREQ=$(get_cpu_freq)
        GPU_FREQ=$(get_gpu_freq)
        
        # 格式化输出到日志文件
        printf "%-20s %-9s %-9s %-8s %-8s %-8s %-8s\n" \
            "$TIMESTAMP" "$CPU_USAGE%" "$GPU_USAGE%" "${CPU_TEMP}°C" "${GPU_TEMP}°C" "${CPU_FREQ}MHz" "${GPU_FREQ}MHz" | tee -a $LOG_FILE
        
        sleep $INTERVAL
    done
    
    # 清理
    echo "测试完成，清理进程..." | tee -a $LOG_FILE
    kill $CPU_STRESS_PID 2>/dev/null
    kill $GPU_STRESS_PID 2>/dev/null
    
    echo "==========================================" | tee -a $LOG_FILE
    echo "压力测试完成! 数据已保存到 $LOG_FILE" | tee -a $LOG_FILE
}

# 运行主函数
main
