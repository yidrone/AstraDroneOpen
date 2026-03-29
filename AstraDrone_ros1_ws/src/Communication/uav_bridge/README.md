## uav_bridge文档

### 1 开机自启

使用systemd建立后台程序，基本`.service`文件如下：

```service
[Unit]  
Description=Bridge Service
Wants=network-online.target
After=network-online.target
 
[Service]  
Type=simple  
User=uav
ExecStart=/home/uav/AstraDroneOpen/scripts/run_sh/patrol_sh/auto_start.sh
ExecStop=/usr/bin/tmux kill-session -t lio
Restart=on-failure
 
[Install]  
WantedBy=multi-user.target
```

详细指令：https://www.runoob.com/linux/linux-comm-systemctl.html

服务文件放置于：/etc/systemd/system/

修改服务文件内容后进行重新加载：

```shell
sudo systemctl daemon-reload
```

开机自启配置以及日志查看，服务名为文件的名字：

```shell
# 启用服务（开机自启）
sudo systemctl enable [服务名]
# 禁用服务（取消开机自启）
sudo systemctl disable [服务名]
# 启动服务
sudo systemctl start [服务名]
# 停止服务
sudo systemctl stop [服务名]
# 重启服务
sudo systemctl restart [服务名]
# 查看单个服务状态
systemctl status [服务名]
#查看详细日志
journalctl -u [服务名] -xe
```

### 2 tmux后台启动须知

以`base.sh`为例：注意服务启动在`ros`之前，需要`source ros`环境以及工作空间

```shell
#!/bin/bash
SESSION="base"

# 1. 重置会话
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -x 80 -y 24 #指定后台（-d）与窗口大小

# 重置布局为 4 格
tmux kill-pane -a -t 0 # 清理一下
tmux split-window -h -t 0
tmux split-window -v -t 0
tmux split-window -v -t 2

tmux split-window -v -t 3

# Pane 0: Position
tmux select-pane -t 0
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "sleep 25s" C-m
tmux send-keys "rostopic echo /mavros/local_position/pose" C-m

# Pane 1: PX4
tmux select-pane -t 1
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "roslaunch mavros px4.launch" C-m

# Pane 2: Mid-360
tmux select-pane -t 2
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "astra" C-m
tmux send-keys "sleep 16s" C-m
tmux send-keys "roslaunch livox_ros_driver2 msg_MID360.launch" C-m

# Pane 3: A8mini
tmux select-pane -t 3
tmux send-keys "sleep 5s" C-m
tmux send-keys "python /home/uav/AstraDroneOpen/AstraDrone_ros1_ws/src/Utils/siyi_A8mini_sdk/control.py" C-m

# Pane 4: A8mini
tmux select-pane -t 4
tmux send-keys "sleep 10s" C-m
tmux send-keys "python /home/uav/AstraDroneOpen/AstraDrone_ros1_ws/src/Utils/siyi_A8mini_sdk/siyi_camera_a8_mini/zoom_controller.py" C-m

# 不可以进入会话，为后台启动
#tmux -2 attach-session -t $SESSION

exit 0
```

如果需要看终端打印的信息，运行以下命令：

```shell
tmux ls #tmux任务名
tmux attach -t [tmux任务名称] #SESSION="base"，base为任务名
```

### 3 SDK说明

#### 3.1 通信协议

| **字节偏移** | **长度** | **字段**    | **类型** | **说明**                             |
| ------------ | -------- | ----------- | -------- | ------------------------------------ |
| 0            | 1        | **Head 1**  | `uint8`  | 固定为 `0xFF`                        |
| 1            | 1        | **Head 2**  | `uint8`  | 固定为 `0xFE`                        |
| 2            | 2        | **Length**  | `uint16` | 包体长度 = `Payload长度` + 1 (CmdID) |
| 4            | 1        | **Cmd ID**  | `uint8`  | 功能指令码（见后文）                 |
| 5            | N        | **Payload** | `bytes`  | 数据内容                             |

#### 3.2 协议指令（下行）

#####  姿态与位置 (0x01)

**Cmd ID**: `0x01`

**Payload**: 20 字节 (`float` * 5)

| **顺序** | **数据**     | **单位** | **说明**       |
| -------- | ------------ | -------- | -------------- |
| 1        | Position X   | m        | 局部坐标系 X   |
| 2        | Position Y   | m        | 局部坐标系 Y   |
| 3        | Position Z   | m        | 局部坐标系 Z   |
| 4        | UAV Yaw      | deg      | 无人机偏航角   |
| 5        | Gimbal Pitch | deg      | 云台实际俯仰角 |

#####  电池电量 (0x0D)

- **Cmd ID**: `0x0D`
- **Payload**: 4 字节 (`int`)
- **说明**: 0-100 的百分比。*注：受 V5.7 版本的电量欺骗功能影响。*

##### 系统状态标志 (0x0E)

- **Cmd ID**: `0x0E`
- **Payload**: 8 字节 (`uint8` * 8)
- **说明**: 每一位代表一个模块的健康或运行状态 (1=OK/ON, 0=FAIL/OFF)。

| **索引** | **模块名称**            | **对应逻辑/变量** | **关键说明**                          |
| -------- | ----------------------- | ----------------- | ------------------------------------- |
| 0        | Overall                 | `status_overall`  | 全局状态 (需 Loc=1, Lidar=1, Bat>30%) |
| 1        | Lidar                   | `status_lidar`    | 激光雷达频率检查 (2-20Hz)             |
| **2**    | **Base Algo**           | `base_launched`   | **基础算法是否已启动** (`base.sh`)    |
| 3        | Depth                   | `status_depth`    | 深度相机频率检查                      |
| 4        | CSI                     | `status_csi`      | CSI 相机频率检查                      |
| **5**    | **Localization**（Loc） | `status_loc`      | **定位状态** (决定是否允许起飞)       |
| 6        | Flight State            | `flight_state`    | 0:静止, 1:手动, 2:自动/Offboard       |
| 7        | Mapping                 | `status_mapping`  | 建图算法是否运行 (`lio.sh`)           |

#### 3.3 协议指令（上行）

##### 任务与脚本控制

| Cmd ID   | **名称**             | **Payload 格式**                    | **说明**                                          |
| -------- | -------------------- | ----------------------------------- | ------------------------------------------------- |
| **0x02** | **Upload Waypoints** | `uint16`(count) + N * `struct`(28B) | 上传航点列表，写入 YAML 并启动巡逻脚本。          |
| **0x03** | **Fly To**           | `float` * 7 (28 Bytes)              | 单点飞行。参数: `x, y, z, yaw, pitch, zoom, time` |
| **0x13** | **Start Base**       | 无                                  | 启动基础算法 (`base.sh`)。起飞前必须执行。        |
| **0x14** | **Resume**           | 无                                  | 恢复任务 (发送 Mode 5)。                          |
| **0x0F** | **RTH**              | 无                                  | 返航 (Return to Home)。若未解锁则执行重启。       |
| **0x11** | **Emergency**        | 无                                  | 紧急降落 (切换至 Mode 2 自动降落)。               |
| **0x15** | **Reboot**           | 无                                  | 重启机载电脑 (仅在 DISARM 状态下有效)。           |
| **0x12** | **Color Map**        | 无                                  | 启动 `color_map.sh`。                             |

##### 手动控制 (Manual Control)

发送后 SDK 会发布 `Twist` 消息，并将模式切为 `Manual (3)`。

| **Cmd ID** | **功能** | **Payload**    | **说明**                   |
| ---------- | -------- | -------------- | -------------------------- |
| **0x04**   | 前进     | `float` (step) | 向前移动 step 米           |
| **0x05**   | 后退     | `float` (step) | 向后移动 step 米           |
| **0x06**   | 左移     | `float` (step) | 向左移动 step 米           |
| **0x07**   | 右移     | `float` (step) | 向右移动 step 米           |
| **0x08**   | 旋转     | `float` (deg)  | 偏航旋转。正=右转，负=左转 |
| **0x09**   | 升降     | `float` (step) | 垂直移动。正=上，负=下     |

##### 负载控制 (Payload)

| **Cmd ID** | **功能**   | **Payload**   | **说明**                   |
| ---------- | ---------- | ------------- | -------------------------- |
| **0x0A**   | 云台 Yaw   | `float` (deg) | 云台偏航微调 (+/-)         |
| **0x0B**   | 云台 Pitch | `float` (deg) | 云台俯仰微调 (+/-)         |
| **0x0C**   | 相机曝光   | `float` (val) | 设置曝光值                 |
| **0x10**   | 相机变焦   | `float` (val) | 设置变焦倍数 (如 1.0, 5.0) |

需要配置各个启动脚本的绝对路径：
```python
self.script_path = rospy.get_param('~script_path', '/home/uav/AstraDroneOpen/scripts/run_sh/patrol_sh')     
```

 对于astra_control需要配置yaml文件的绝对路径：

```python
self.yaml_path = rospy.get_param('~yaml_path', '/home/uav/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_control/config/astra_real.yaml')
```

#### 3.3 调试指令

在非专用电池或者电压不足的情况下调试SDK（适配器直接供电），可以进行电量欺骗，默认为60%

| **Cmd ID** | **功能**          | **Payload**     | **说明**                                                   |
| ---------- | ----------------- | --------------- | ---------------------------------------------------------- |
| **0x16**   | **Battery Spoof** | `int` (percent) | **电量欺骗**。 >0: 锁定为该百分比 <0: 关闭欺骗，恢复真实值 |



### 4 安全机制

SDK 在执行 `0x02` (Waypoints) 或 `0x03` (FlyTo) 前，会强制检查以下三个条件。任一条件不满足，指令将被**拒绝** (Reject)。

1. **Localization Ready**: `status_loc == 1`

   即里程计频率 > 2Hz 且 协方差 < 0.2。

2. **Base Algorithm Launched**: `base_launched == True`

   即必须启动`base.sh`。

3. **Battery Safety**: `battery_percent >= 30%`

   即电量充足（或已开启电量欺骗）。

### 5 测试脚本

​	提供了指令测试和状态监测，以及是否起飞的可视化，直接运行，在命令行中输出即可。test.py