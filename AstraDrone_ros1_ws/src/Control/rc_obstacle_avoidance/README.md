# rc_obstacle_avoidance

基于 MAVROS 遥控输入，融合 Fast-Planner 规划与 PX4 控制的遥控避障节点：**将遥控转换成规划目标，订阅 Fast-Planner 输出的避障轨迹（位置/速度），再转发给 PX4**，实现“规划 + 控制”闭环。

## 功能概述
- 订阅 `/mavros/rc/in` 获取遥控器通道，支持自定义通道映射与死区设置（默认 1-4 对应横滚/俯仰/油门/偏航）。
- 订阅 `/mavros/local_position/pose` 获取当前位姿，在当前位置附近生成偏移目标。
- 将生成的目标发布到 `/fastplanner/goal`，由 Fast-Planner 输出避障轨迹。
- 订阅 `/fastplanner/setpoint_position/local` 与 `/fastplanner/setpoint_velocity/cmd_vel`，优先使用位置指令，其次使用速度指令，并转发到 `/mavros/setpoint_position/local` 与 `/mavros/setpoint_velocity/cmd_vel`，让 PX4 跟踪规划器的避障轨迹。
- 内置发布定时器，固定频率刷新规划目标；若长时间未收到遥控量，会自动保持当前位置防止漂移。

## 参数说明（在 `config/rc_obstacle_avoidance.yaml` 中调整）
> 修改 YAML 后重新启动节点即可生效，无需改 launch。YAML 中的中文注释直接说明调节后无人机的飞行效果。

| 参数 | 说明 | 默认值 |
| --- | --- | --- |
| `rc_topic` | 遥控输入话题 | `/mavros/rc/in` |
| `pose_topic` | 位置参考话题 | `/mavros/local_position/pose` |
| `goal_topic` | Fast-Planner 目标话题 | `/fastplanner/goal` |
| `planner_pose_topic` | Fast-Planner 规划输出（位置） | `/fastplanner/setpoint_position/local` |
| `planner_vel_topic` | Fast-Planner 规划输出（速度） | `/fastplanner/setpoint_velocity/cmd_vel` |
| `mavros_position_topic` | 转发到 PX4 的位置指令话题 | `/mavros/setpoint_position/local` |
| `mavros_velocity_topic` | 转发到 PX4 的速度指令话题 | `/mavros/setpoint_velocity/cmd_vel` |
| `deadband` | 摇杆死区 (us)，小于该范围视为 0 | `50` |
| `max_xy_step` | 横滚/俯仰对应的水平偏移（米） | `1.0` |
| `max_z_step` | 油门对应的垂直偏移（米） | `0.6` |
| `max_yaw_change` | 每次命令的偏航增量（弧度） | `0.35` |
| `roll_channel` | 横滚通道索引（从 1 开始） | `1` |
| `pitch_channel` | 俯仰通道索引 | `2` |
| `throttle_channel` | 油门通道索引 | `3` |
| `yaw_channel` | 偏航通道索引 | `4` |
| `publish_rate` | 目标发布频率 (Hz) | `20.0` |
| `command_timeout` | 遥控超时后回到当前位置的时长 (s) | `0.5` |
| `use_fastplanner` | 是否向 Fast-Planner 发布目标 | `true` |
| `enable_planner_follow` | 是否转发 Fast-Planner 规划到 PX4 | `true` |
| `planner_cmd_timeout` | 规划指令过期阈值 (s)，过期则不转发 | `0.3` |

### 参数调整对飞行的影响（示例）
- **死区 `deadband`**：增大能抑制中位抖动，悬停更稳，但细微调整会被忽略；减小则更灵敏但可能微漂。
- **平移/升降步长 `max_xy_step` / `max_z_step`**：增大后满杆偏移更大，规划轨迹转向更快；减小则动作更柔和、适合室内小场景。
- **偏航增量 `max_yaw_change`**：数值越大，机头转向越快，易产生急转弯；减小可让转向更顺滑。
- **发布频率 `publish_rate`**：提高后遥控“跟手”感增强，但链路负载上升；降低则节省带宽，适合调试。
- **超时保护 `command_timeout`**：时间越短，断杆后更快回到当前位置，悬停更安全；时间过长则可能在无人输入时继续漂移。
- **规划转发开关 `enable_planner_follow`**：关闭后仅生成目标不跟随规划轨迹，适合单独调试 Fast-Planner；开启后 PX4 会沿避障轨迹飞行。
- **规划超时 `planner_cmd_timeout`**：减小可防止跟随陈旧轨迹；增大则允许规划输出暂时停顿仍继续执行。

## 使用步骤
1. **准备环境**：确保 PX4 已刷入 1.15.4，`mavros`、`fast_planner`、`fastlio` 等节点正常工作，且 EKF 已收敛；地面站检查 RC 通道与参数对应关系。
2. **编译**：在 `AstraDrone_ros1_ws` 下运行 `catkin_make`，确认该包被编译。
3. **启动节点**：
   ```bash
   roslaunch rc_obstacle_avoidance rc_obstacle_avoidance.launch
   ```
4. **切换模式**：通过地面站或 `/mavros/set_mode` 进入 `OFFBOARD`，并解锁电机。
5. **遥控飞行（规划 + 控制闭环）**：
   - 横滚/俯仰通道控制左右、前后平移；
   - 油门通道控制上升/下降（最低限制 0 米以下的目标高度会被裁剪）；
   - 偏航通道控制航向角累加。
   节点会把 RC 偏移生成的目标发送到 `/fastplanner/goal`，再**订阅规划器输出并转发给 PX4**：
   - 优先使用 `/fastplanner/setpoint_position/local`，无位置指令时自动转发 `/fastplanner/setpoint_velocity/cmd_vel`；
   - 规划指令超过 `planner_cmd_timeout` 未更新则不再转发，防止陈旧轨迹；
   - 如果遥控杆回到中位或 `command_timeout` 超时，会刷新目标为当前位置，规划器会生成“原地”轨迹确保稳定悬停。

## 启动文件示例
`launch/rc_obstacle_avoidance.launch` 会自动加载 `config/rc_obstacle_avoidance.yaml` 并启动节点：
```xml
<launch>
  <rosparam command="load" file="$(find rc_obstacle_avoidance)/config/rc_obstacle_avoidance.yaml" />
  <node pkg="rc_obstacle_avoidance" type="rc_obstacle_avoidance_node" name="rc_obstacle_avoidance" output="screen" />
</launch>
```

## 话题接口
- 订阅：`/mavros/rc/in`、`/mavros/local_position/pose`、`/fastplanner/setpoint_position/local`、`/fastplanner/setpoint_velocity/cmd_vel`
- 发布：`/fastplanner/goal`、`/mavros/setpoint_position/local`、`/mavros/setpoint_velocity/cmd_vel`

## 常见问题
- **没有规划输出？** 确认 `/fastplanner/goal` 已更新且 Fast-Planner 正在运行；如需调试，可打开 `/fastplanner/goal`、`/fastplanner/setpoint_position/local` 回显。
- **飞机不响应遥控？** 检查 RC 通道映射是否与 `roll/pitch/throttle/yaw_channel` 匹配；同时确保进入 `OFFBOARD` 并已解锁。
- **轨迹跟随突停？** 可能是规划超时触发保护，可适当增大 `planner_cmd_timeout` 或检查 Fast-Planner 是否持续输出。
