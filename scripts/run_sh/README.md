# run_sh 目录说明

本目录包含若干通过 `tmux` 启动的脚本，方便一次性拉起演示、示例或录制所需的 ROS 节点。每个脚本都会先结束同名的 tmux 会话，然后按预设的分屏布局启动相关命令。概览如下：

## 脚本作用

- **demonstration.sh**：分成 6 个窗格，依次启动 `roscore`、PX4 MAVROS、Livox 驱动、Fast-LIO、探索管理器和 RViz，完成完整演示流程。
- **demonstration_control.sh**：分成 8 个窗格，启动 `roscore`、MAVROS、Livox 驱动、Fast-LIO、CSI 摄像头、astra_control 实机控制、RealSense 摄像头以及 rqt 等工具，适合实机控制演示。
- **echo.sh**：分成 4 个窗格，分别 `rostopic echo` 常用的 MAVROS 位姿、状态和期望位置话题，便于快速观察状态。
- **onboard_example.sh**：分成 6 个窗格，串行启动 `roscore`、MAVROS、Livox 驱动、Fast-LIO、CSI 摄像头和 RealSense 摄像头，用于机载示例流程。
- **pc_example.sh**：分成 5 个窗格，启动 `roscore`、PX4 仿真示例、Fast-LIO（关闭 RViz）、offboard 自动控制以及 QGroundControl 客户端，适合 PC 端示例。
- **record.sh**：在单窗格中进入 `~/bag` 目录并执行 `rosbag record`，录制相机、雷达、MAVROS 等主要话题。
- **test.sh**：分成 4 个窗格，启动 `roscore`、MAVROS、Livox 驱动和 Fast-LIO，便于基本功能联调测试。

## tmux 简介

`tmux` 是一款终端复用器，可在一个终端中创建多个会话、窗口和窗格，支持分屏、后台运行和会话恢复，适合管理多进程或长时间运行的任务。

## tmux 快速使用

### 创建与附着会话
- 新建会话：`tmux new -s <session_name>`
- 附着到已有会话：`tmux attach -t <session_name>` 或 `tmux a -t <session_name>`
- 列出会话：`tmux ls`
- 退出并关闭会话：在 tmux 中输入 `exit`，或在外部运行 `tmux kill-session -t <session_name>`

### 窗口与窗格操作（默认前缀为 `Ctrl+b`）
- 水平分屏：`Ctrl+b` 然后 `%`
- 垂直分屏：`Ctrl+b` 然后 `"`
- 切换窗格：`Ctrl+b` + 方向键
- 调整窗格大小：`Ctrl+b` 按住后再用方向键调整
- 创建新窗口：`Ctrl+b c`
- 切换窗口：`Ctrl+b n`（下一个）、`Ctrl+b p`（上一个）、`Ctrl+b <数字>`（跳转指定窗口）

### 会话管理
- 断开但保持运行：`Ctrl+b d`（detach），进程仍在后台运行。
- 重命名会话：`tmux rename-session -t <old> <new>`
- 复制模式：`Ctrl+b [` 进入滚屏/复制，`q` 退出。

## 运行脚本的小贴士

- 脚本默认会终止同名 tmux 会话并重新创建，运行前请确认对应会话中的任务可以安全结束。
- 如果需要在脚本基础上追加命令，可在脚本中相应窗格的 `tmux send-keys` 之后添加新的指令。
- 若想在附着时直接进入脚本创建的会话，可运行：`bash <script_name>.sh`，脚本末尾会自动执行 `tmux attach`。
