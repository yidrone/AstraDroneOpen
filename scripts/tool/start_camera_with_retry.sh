#!/bin/bash

MAX_RETRIES=3
RETRY_COUNT=0
SLEEP_TIME=2

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    echo "尝试启动相机 (第 $((RETRY_COUNT+1)) 次)"
    # 启动相机节点
    roslaunch csi_camera_driver csi_camera.launch &
    LAUNCH_PID=$!
    
    # 等待一段时间检查是否成功
    sleep 5
    
    # 检查节点是否仍在运行且发布了正确的话题
    if rostopic list | grep -q "/csi_camera/image_raw"; then
        echo "检测到话题 /csi_camera/image_raw，开始检测频率..."

        # 使用 timeout 限制 rostopic hz 运行时间（比如 8 秒）
        # rostopic hz 会持续打印，我们只取这段时间内的输出
        HZ_OUTPUT=$(timeout 8 rostopic hz /csi_camera/image_raw 2>/dev/null)

        # 从输出中解析 average rate（取最后一行，避免前面不稳定）
        RATE_LINE=$(echo "$HZ_OUTPUT" | grep -i "average rate" | tail -n 1)
        RATE=$(echo "$RATE_LINE" | awk '{print $3}')

        if [ -n "$RATE" ]; then
            echo "检测到相机话题平均频率：${RATE} Hz"

            # 使用 awk 做浮点比较：> 2.0 Hz 认为正常
            if awk -v r="$RATE" 'BEGIN{exit (r>2.0)?0:1}'; then
                echo "相机数据流频率大于 2 Hz，启动完成！"
                # 等待 roslaunch 进程结束（阻塞）
                wait $LAUNCH_PID
                exit 0
            else
                echo "警告：相机数据流频率 (${RATE} Hz) 小于或等于 2 Hz，视为启动异常"
            fi
        else
            echo "警告：无法从 rostopic hz 输出中解析频率（可能 8 秒内没有收到足够数据），视为启动失败"
        fi

        # --------- 走到这里说明本次检测失败，区分是否最后一次尝试 ---------
        if [ $((RETRY_COUNT+1)) -ge $MAX_RETRIES ]; then
            echo "已经是第 $MAX_RETRIES 次尝试，保持当前相机进程 (PID: $LAUNCH_PID) 运行，不再杀掉也不再重试。"
            echo "脚本将保持运行，仅做空闲等待。"
            # 进入空闲等待，不退出、不再动相机
            while true; do
                sleep 60
            done
        else
            echo "清理相机进程并准备重试..."
            kill $LAUNCH_PID 2>/dev/null
            sleep 2
            pkill -f "roslaunch csi_camera_driver" 2>/dev/null
            sleep $SLEEP_TIME
            ((RETRY_COUNT++))
        fi

    else
        echo "相机启动失败，未检测到话题 /csi_camera/image_raw"

        # --------- 话题都没起来的失败，也要区分是否最后一次尝试 ---------
        if [ $((RETRY_COUNT+1)) -ge $MAX_RETRIES ]; then
            echo "已经是第 $MAX_RETRIES 次尝试，保持当前相机进程 (PID: $LAUNCH_PID) 运行，不再杀掉也不再重试。"
            echo "脚本将保持运行，仅做空闲等待。"
            while true; do
                sleep 60
            done
        else
            kill $LAUNCH_PID 2>/dev/null
            sleep 2
            pkill -f "roslaunch csi_camera_driver" 2>/dev/null
            sleep $SLEEP_TIME
            ((RETRY_COUNT++))
        fi
    fi
done

# 理论上不会走到这里（因为最后一次失败时已经进入死循环等待）
echo "逻辑结束"
exit 1

