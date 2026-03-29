本文档旨在展示与介绍工作空间中的各个程序包与代码。

## **Communication**  ：通信接口，负责上位机与下位机的消息传输与协议适配。

其中共包括两个程序包：`siriel tool`与`uav_bridge`。提供了上位机与下位机的通讯工具。

前者提供了ROS与下位机位姿数据的连接，`serial_read.py`将下位机数据进行二进制数据包的解码并作为位姿数据传给ROS，`serial_send.py`则对位姿数据进行二进制数据打包，通过与下位机连接的串口如`/dev/ttyUSB1`进行发送。

```python
while not rospy.is_shutdown():
        try:
            # 数据包大小：6个float64 + 1个校验位
            packet_size = 48 + 1
            packet = ser.read(packet_size)

            if len(packet) == packet_size:
                data = packet[:-1]
                checksum = packet[-1]
                calculated_checksum = sum(data) % 256

                if checksum == calculated_checksum:
                    # 解包数据
                    values = struct.unpack('6d', data)
                    pose_msg = Pose()
                    pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = values[0:3]
                    pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z = values[3:6]
                    pose_msg.orientation.w = 1.0  # 默认设为1.0，后续可优化

                    pub.publish(pose_msg)
                    rospy.loginfo(f"Received data: {values}, checksum: {checksum}")
                else:
                    rospy.logwarn("Checksum mismatch!")
        except Exception as e:
            rospy.logerr(f"Failed to receive data: {e}")

        rate.sleep()
```

进行数据解包，并发送到`/received_pose`话题中。

同时，在发送时，回调函数中二进制数据的打包并发送给串口：

```python
def pose_callback(msg):
    try:
        # 数据打包：'6d' 表示 6 个 float64
        data = struct.pack('6d', msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        checksum = calculate_checksum(data)
        
        # 构造完整数据包：前导码 00 02 17 + 数据 + 校验和
        packet = bytes([0x00, 0x02, 0x17]) + data + struct.pack('B', checksum)
        
        ser.write(packet)
        rospy.loginfo(f"Sent data: {msg.position.x}, {msg.position.y}, {msg.position.z}, {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, checksum: {checksum}")
    except Exception as e:
        rospy.logerr(f"Failed to send data: {e}")
```

`uav_bridge` 与 **外部 TCP 客户端（如地面站或远程服务器）** 之间的通信桥梁。**下行 (TX)**: 订阅 `/bridge/tx`，将收到的字节流直接通过 TCP 发送给客户端。**上行 (RX)**: 监听 TCP 端口，将收到的字节流发布到 ROS 话题 `/bridge/rx`。

```C++
// 发送：ROS -> TCP
send(client_fd_, msg->data.data(), msg->data.size(), MSG_NOSIGNAL);
// 接收：TCP -> ROS
recv(client_fd_, buf, sizeof(buf), 0);
```

`command_parser_node.cpp`：解释接收到的指令，并驱动无人机动作。调用外部的Shell脚本驱动无人机。

```C++
// 构造最简单的系统命令
// 格式: gnome-terminal -- /path/to/script.sh arg1
std::string cmd = "gnome-terminal -- " + full_path + " " + args;

// 执行
int ret = system(cmd.c_str());
```

`px4_unlocker_node.cpp`：提供与PX4的底层交互，解决OFFBOARD的模式切换问题，可以用于维持飞控的板外状态。接收到解锁请求后，按顺序执行：开启设定点流 -> 等待稳定 -> 切换 OFFBOARD -> ARM。

```C++
// 核心线程：以 20Hz 频率发送悬停点
void streamLoop() {
    ros::Rate rate(20.0);
    while (ros::ok()) {
        if (is_streaming_) {
            // ... (构造 Pose)
            pose.pose.position.z = 1.0; 
            local_pos_pub_.publish(pose);
        }
        rate.sleep();
    }
}
```

`arm_offboard_node.cpp`：自动起飞，提供了一键连接、解锁与起飞到指定高度的自动化过程。

```C++
// --- 关键步骤 1: 发送一些设定点 ---
// 在切换模式前，必须先发送一段时间的设定点
for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    // ...
}
```

`rtsp_sender_node.cpp`：订阅 ROS 图像话题，推流到 RTSP 服务器，并针对**NVIDIA Jetson** 系列进行硬编码减少占用。

## **Control** ： 无人机控制逻辑，包括姿态控制、位置控制等。

本模块包含`rc_obstacle_avoidance`程序包，适用于需要**人工介入但又希望保留自动避障能力**的场景。

本模块基于 MAVROS 遥控输入，融合 Fast-Planner 规划与 PX4 控制的遥控避障节点：**将遥控转换成规划目标，订阅 Fast-Planner 输出的避障轨迹（位置/速度），再转发给 PX4**，实现“规划 + 控制”闭环。最后又提供了初始起飞的保护，防止响应摇杆乱飞。

通过修改 config/rc_obstacle_avoidance.yaml 后启动launch文件。

```C++
const ros::Time now = ros::Time::now();
// 检查规划指令的新鲜度，超过超时时间则不再使用
const bool pose_fresh = has_planner_pose_ && (now - last_planner_pose_time_).toSec() <= planner_cmd_timeout_;

if (pose_fresh && mavros_position_pub_)
{
  geometry_msgs::PoseStamped cmd = planner_pose_cmd_;
  cmd.header.stamp = now;  // 更新时间戳保持 PX4 接收
  mavros_position_pub_.publish(cmd);
  return;
}
```

## **Detection**：目标检测算法，提供了几种经典的目标检测与预测方法。

`apriltag_ros`：用于在 ROS 环境中检测 AprilTag 并估计其位姿。是一个比较成熟的ros包。

`aruco_localization`：利用 ArUco 标记图（Marker Map）来实现相机或机器人的定位。

`yolo_detect`：基于 **YOLOv8** 和 **ROS 1** 的无人机视觉感知程序包。

·`pt2eng.py`：加载标准的 PyTorch 模型 (`.pt`) 并将其导出为 TensorRT 引擎文件 (`.engine`)。加速模型加载。

·`testEnv.py`：一个不依赖 ROS 的独立测试脚本。它加载模型并读取一张程序包内本地图片 (`bus.jpg`) 进行循环检测，并在窗口中显示结果和 FPS。用于检测 YOLO 环境（ultralytics 库）、OpenCV 和模型路径是否配置正确。

·`yolo_detect.py`：基础的2D检测节点，订阅 RGB 图像话题 `/csi_camera/image_raw`，并发布标注后的图像到 `/yolo/detect_image`。

·`yolo_detect_fusion.py`：识别物体并在图像上获得中心点 (u, v)，结合相机内参，将 2D 像素点反投影为相机坐标系下的 3D 点 (x, y, z)。并利用 TF 树，将坐标从**相机坐标系**转换到**无人机机体坐标系 (FMU)**。利用深度学习进行目标检测，并将 2D 图像检测结果与深度相机数据融合，实现目标的 **3D 空间定位**。

`target_prediction`： 利用 **卡尔曼滤波（Kalman Filter）** 及其变体（**EKF** 和 **UKF**），对一个移动目标（通常是带有 ArUco 码的车辆或平台）进行状态估计和未来轨迹预测。

**LKF (线性卡尔曼滤波)**：基于恒速模型 (Constant Velocity)，适用于线性运动。

**EKF (扩展卡尔曼滤波)**：基于非线性运动学模型，适用于做曲线运动的车辆。

**UKF (无迹卡尔曼滤波)**：处理强非线性，代码中提供了带有实时 Matplotlib 可视化的 Python 版本。

核心运动学模型：

```python
# 状态向量定义: [x, y, z, φ, v, a, K, curvature_speed, vz]
# 关键：这里定义了“曲率 K”和“偏航角 φ”的非线性关系，这是处理转弯的关键。
def fx(x, dt):
    # ...
    # 非线性状态更新
    x_pos += v * np.cos(phi) * dt      # X 位置随速度和角度变化
    y_pos += v * np.sin(phi) * dt      # Y 位置随速度和角度变化
    phi += K * v * dt                  # 关键：偏航角随曲率(K)和速度(v)变化 (w = K*v)
    v += a * dt                        # 速度随加速度变化
    K += curvature_speed * dt          # 曲率随曲率变化率更新
    # ...
    return np.array([x_pos, y_pos, z_pos, phi, v, a, K, curvature_speed, vz])
```

 雅可比矩阵：

```python
# 状态向量定义: [x, y, z, φ, v, a, K, curvature_speed, vz]
# 关键：这里定义了“曲率 K”和“偏航角 φ”的非线性关系，这是处理转弯的关键。
def fx(x, dt):
    # ...
    # 非线性状态更新
    x_pos += v * np.cos(phi) * dt      # X 位置随速度和角度变化
    y_pos += v * np.sin(phi) * dt      # Y 位置随速度和角度变化
    phi += K * v * dt                  # 关键：偏航角随曲率(K)和速度(v)变化 (w = K*v)
    v += a * dt                        # 速度随加速度变化
    K += curvature_speed * dt          # 曲率随曲率变化率更新
    # ...
    return np.array([x_pos, y_pos, z_pos, phi, v, a, K, curvature_speed, vz])
```

计算目标未来位置：

```python
# 获取预测时间参数（通常设为 1.2秒）
predicted_time = float(rospy.get_param('~predicted_time', 1.2))

# 关键：基于当前估计的状态(x, v, phi)，外推未来的位置
# 这不是滤波器的"预测步骤"，而是业务逻辑上的"未来位置计算"
predicted_x = x[0] + x[4] * np.cos(x[3]) * predicted_time
predicted_y = x[1] + x[4] * np.sin(x[3]) * predicted_time
predicted_z = x[2] + x[8] * predicted_time
```

进行坐标变换：

```c++
// 等待并查找 TF 变换 (Map -> Aruco/Target)
// 只有拿到这个变换，才能知道目标在世界坐标系下的位置
geometry_msgs::TransformStamped transform = 
    tf_buffer_.lookupTransform("map", "target", ros::Time(0), ros::Duration(1.0));

// 将四元数转换为欧拉角 (提取 Yaw/Phi)
// 滤波器需要 Yaw 角作为观测值之一
tf2::Matrix3x3 m(q);
double roll, pitch, yaw;
m.getRPY(roll, pitch, yaw);
z(3) = yaw; // 将 Yaw 角放入观测向量
```



## **SLAM**  ：定位与建图模块，支持激光,扩展 Fast‑LIO2）。

提供两个基本的程序包。`FAST-LIO`与`FreeDom`.

`FAST-LIO`：通过融合激光雷达与imu数据，使用相应的滤波方法实现无人机的定位。

`FreeDom`：实现了实时生成高质量体素地图和点云地图的能力,有效解决了自动驾驶、机器人导航等领域中动态物体对地图构建的干扰问题。建好的地图提供给Planner相关算法进行避障规划。

## **Track**  ：目标检测与跟踪相关算法。

`pix_tracker.py`：无人机追踪控制器

这个脚本是系统的核心，实现了基于图像反馈的无人机位置控制。实现了一个标准的PID控制算法。

通过 PID 控制器，根据图像中目标像素点与图像中心的偏差，计算无人机的速度指令。控制无人机在水平面移动，以使目标位于图像中心，维持固定的飞行高度。在非 OFFBOARD 模式下，保持当前位置悬停。

```python
def calculate_control_velocities(self):
    # ...
    # 1. 计算像素误差：目标位置 - 图像中心
    error_u = self.target_pixel[0] - self.image_center[0]  # 水平方向偏差 (图像 U轴)
    error_v = self.target_pixel[1] - self.image_center[1]  # 垂直方向偏差 (图像 V轴)
    
    # ... (死区检查) ...

    # 2. 坐标系转换与 PID 计算
    # 注意：vel_x (机头方向) 对应 -error_v (图像垂直方向的负值)
    # 这意味着：如果目标在图像上方 (pixel y 小)，error_v 为负，-error_v 为正，无人机向前飞 (vel_x > 0)
    print("vel_x pid:")
    vel_x = self.pid_x.update(-error_v)
    
    # 注意：vel_y (机身右侧) 对应 -error_u (图像水平方向的负值)
    # 这意味着：如果目标在图像右侧 (pixel x 大)，error_u 为正，-error_u 为负，无人机向左飞 (vel_y < 0) ?? 
    # (需根据实际相机安装方向确认，这里逻辑通常对应相机下视且机头朝向图像上方)
    print("vel_y pid:")
    vel_y = self.pid_y.update(-error_u)

    # 3. 速度限幅
    vel_x = max(-self.max_velocity, min(self.max_velocity, vel_x))
    vel_y = max(-self.max_velocity, min(self.max_velocity, vel_y))
    return vel_x, vel_y
```



`target_pulisher.py`：这个脚本主要用于调试和可视化，帮助用户直观地看到追踪目标在图像中的位置。

```python
def draw_markers(self):
    # ...
    # 1. 绘制图像中心（绿色十字），代表无人机的正前方/正下方
    cv2.drawMarker(self.current_image, (center_x, center_y),
                   (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

    # 2. 绘制目标点（红色圆圈），代表目标在视野中的位置
    if self.target_point.x > 1 and self.target_point.y > 1:
        # ...
        cv2.circle(self.current_image, (target_x, target_y),
                   10, (0, 0, 255), 2)
        
        # 3. 绘制误差线（蓝色线），直观显示 PID 需要消除的距离
        cv2.line(self.current_image, (center_x, center_y),
                 (target_x, target_y), (255, 0, 0), 1)
```

这套代码的核心在于 `calculate_control_velocities` 中的 PID 映射，它将二维图像上的像素差值转换为了无人机在三维空间中的速度矢量。

## **Planner**：路径规划器，包含局部规划和全局规划算法。

本模块提供了经典的`ego-planner`和`fast-planner`两种经典的规划算法。

## **Utils**：工具库、通用函数。

提供了许多的通用工具以及相应的驱动、SDK。

`camera_sdk`：提供了一个基本的相机SDK。支持多种种类的RGB相机。

`catkin_simple`：`catkin_simple` 是一个旨在简化 ROS `catkin` 软件包 `CMakeLists.txt` 文件编写的工具。它通过一系列自定义宏，减少了样板代码，使构建配置更加简洁。自动处理依赖、自动包含路径、自动生成消息、服务、动作、自动处理动态重配置。

`csi_camera_driver`： CSI 摄像头驱动ROS包，名为 `csi_camera_driver`。

它的核心目标是在不依赖庞大的OpenCV库的情况下，高效地通过GStreamer获取 CSI 摄像头的图像，并发布标准的ROS图像话题 (`sensor_msgs/Image`) 和相机内参 (`sensor_msgs/CameraInfo`)。

`cv_bridge`： 是 ROS 生态系统中一个用于图像格式转换的核心功能包。它的主要作用是在ROS的图像消息格式和OpenCV的图像数据格式 之间架起一座桥梁（Bridge）。

`fake_slam`：模拟激光SLAM系统的地图构建过程。它不包含真正的SLAM算法（如闭环检测、位姿图优化等），而是通过订阅外部提供的 精确位姿和实时激光点云PointCloud2），将点云根据位姿拼接到全局坐标系中，从而“伪造”出一个全局点云地图。

`imu_2_euler`：这个 ROS 软件包 (`imu_2_euler`) 是一个工具包，主要用于订阅 IMU 数据，将其从四元数转换为欧拉角（俯仰角 Pitch 和翻滚角 Roll），并进行实时记录和可视化。

`lidar_cam_fusion`：`lidar_cam_fusion` 是一个用于将 2D 相机图像与 3D 激光雷达点云进行融合的 ROS 软件包。它主要用于生成具有深度信息的彩色点云，或从点云生成深度图像，从而赋予单目相机深度感知能力。

`livox_ros_driver`、`livox_ros_driver2`、`siyi_A8mini_sdk`、`OrbbecSDK_ROS1`：提供了livox雷达、Orbbec相机、siyi_A8mini相机云台等传感器的驱动和启动文件等。

`pixel2map`：这个 ROS 软件包 (`pixel2map`) 的主要功能是将图像中的 2D 像素坐标转换为世界坐标系下的 3D 坐标。它订阅 YOLO 等目标检测算法输出的像素结果（中心点坐标），结合相机的内参、外参以及从 TF 树中获取的机器人位姿，计算出目标在世界坐标系中的位置。

`pointcloudcut`：这个 ROS 软件包 (`pointcloudcut`) 包含一个主要的 Python 脚本，用于对 `PointCloud2` 格式的点云数据进行基于高度的过滤。

`pub_cam_info`：这个 ROS 软件包 (`pub_cam_info`) 是一个简单的辅助工具，用于发布相机的内参信息 (`sensor_msgs/CameraInfo`)。

`rc_topic`：这个 ROS 软件包 (`rc_topic`) 是一个用于监控遥控器信号并触发紧急状态的简单工具。硬件接管无人机或者切断当前控制、触发返航或降落。

`rtk2local`：这个 ROS 软件包 (`rtk2local`) 的核心功能是实现**全球坐标系 (Global Frame)** 与 **局部坐标系 (Local Frame, FLU)** 之间的双向转换和原点管理。

`serial`：C++ 串口通信库。提供了一套类似 Python `pySerial` 的简洁面向对象接口，支持完善的读写超时控制与数据流管理，非常适合机器人与嵌入式上位机开发。

`topic_tf_tran`：是一个实用的TF与话题互转工具。它主要解决在调试或集成不同机器人模块时，数据格式不匹配的问题（例如：有的模块需要 TF 树，有的模块只发布 Odom 消息；或者反过来，只发布 TF 但需要 Pose 消息）。
