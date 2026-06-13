# Livox ROS Driver 2

Livox ROS Driver 2 是 Livox 第二代 ROS 驱动包，用于连接和发布 Livox LiDAR 数据，支持 ROS1 和 ROS2。ROS1 推荐使用 Noetic，ROS2 推荐使用 Foxy、Humble 或 Jazzy。

  **注意：**

  Livox ROS Driver 主要用于调试、验证和二次开发，不建议直接作为量产软件使用。正式工程落地时，应基于源码按项目需求进行优化、测试和封装。

## 1. 准备工作

### 1.1 系统要求

  * Ubuntu 18.04：ROS Melodic
  * Ubuntu 20.04：ROS Noetic、ROS2 Foxy
  * Ubuntu 22.04：ROS2 Humble
  * Ubuntu 24.04：ROS2 Jazzy

  **提示：**

  Colcon 是 ROS2 常用构建工具。

  Colcon 安装方法请参考：[Colcon 安装说明](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

### 1.2 安装 ROS / ROS2

ROS Melodic 安装请参考：
[ROS Melodic 安装说明](https://wiki.ros.org/melodic/Installation)

ROS Noetic 安装请参考：
[ROS Noetic 安装说明](https://wiki.ros.org/noetic/Installation)

ROS2 Foxy 安装请参考：
[ROS Foxy 安装说明](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

ROS2 Humble 安装请参考：
[ROS Humble 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

ROS2 Jazzy 安装请参考：
[ROS Jazzy 安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

推荐安装 Desktop-Full 版本。

## 2. 构建和运行 Livox ROS Driver 2

### 2.1 克隆 Livox ROS Driver 2 源码

```shell
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

  **注意：**

  请确保源码位于 `[work_space]/src/` 目录下，例如上面的 `ws_livox/src/livox_ros_driver2`。否则可能因为构建工具限制导致编译错误。

### 2.2 构建并安装 Livox-SDK2

  **注意：**

  请参考 [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md) 完成 SDK2 安装。

### 2.3 构建 Livox ROS Driver 2

#### ROS1 示例，以 Noetic 为例

```shell
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

#### ROS2 Foxy

```shell
source /opt/ros/foxy/setup.sh
./build.sh ROS2
```

#### ROS2 Humble

```shell
source /opt/ros/humble/setup.sh
./build.sh humble
```

#### ROS2 Jazzy

```shell
source /opt/ros/jazzy/setup.sh
./build.sh jazzy
```

### 2.4 运行 Livox ROS Driver 2

#### ROS1

```shell
source ../../devel/setup.sh
roslaunch livox_ros_driver2 [launch file]
```

其中：

* **livox_ros_driver2**：Livox ROS Driver 2 的 ROS 包名
* **[launch file]**：需要使用的 ROS1 launch 文件，示例文件位于 `launch_ROS1` 目录

HAP 雷达的 RViz 启动示例：

```shell
roslaunch livox_ros_driver2 rviz_HAP.launch
```

#### ROS2

```shell
source ../../install/setup.sh
ros2 launch livox_ros_driver2 [launch file]
```

其中：

* **[launch file]**：需要使用的 ROS2 launch 文件，示例文件位于 `launch_ROS2` 目录

HAP 雷达的 RViz 启动示例：

```shell
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 3. Launch 文件和 livox_ros_driver2 参数说明

### 3.1 Launch 文件说明

ROS1 的 launch 文件位于 `ws_livox/src/livox_ros_driver2/launch_ROS1`，ROS2 的 launch 文件位于 `ws_livox/src/livox_ros_driver2/launch_ROS2`。不同 launch 文件面向不同雷达和发布格式：

| launch 文件 | 说明 |
| ----------- | ---- |
| rviz_HAP.launch | 连接 HAP 雷达<br>发布 pointcloud2 格式点云<br>自动启动 RViz |
| msg_HAP.launch | 连接 HAP 雷达<br>发布 Livox 自定义点云消息 |
| rviz_MID360.launch | 连接 MID360 雷达<br>发布 pointcloud2 格式点云<br>自动启动 RViz |
| msg_MID360.launch | 连接 MID360 雷达<br>发布 Livox 自定义点云消息 |
| rviz_mixed.launch | 连接 HAP 和 MID360 混合雷达<br>发布 pointcloud2 格式点云<br>自动启动 RViz |
| msg_mixed.launch | 连接 HAP 和 MID360 混合雷达<br>发布 Livox 自定义点云消息 |

### 3.2 主要内部参数说明

Livox_ros_driver2 的内部参数主要在 launch 文件中配置。下面列出常用参数，以及 MID360 NoSync + FAST-LIO 场景新增的时间戳保护参数：

| 参数 | 说明 | 默认值 |
| ---- | ---- | ------ |
| publish_freq | 点云发布频率，浮点数类型。常用值为 5.0、10.0、20.0、50.0 等，最大 100.0 Hz。 | 10.0 |
| multi_topic | 是否为每台雷达单独发布 topic。<br>0 -- 所有雷达共用同一个点云 topic<br>1 -- 每台雷达使用独立点云 topic | 0 |
| xfer_format | 点云格式。<br>0 -- Livox pointcloud2(PointXYZRTLT)<br>1 -- Livox 自定义点云格式<br>2 -- PCL 标准 PointXYZI 点云格式，仅 ROS1 使用 | 0 |
| nosync_time_mode | MID360 上报 NoSync 时间时的时间戳策略。<br>steady_raw -- 将 raw uint64 LiDAR 上电时间戳(ns)按 handle 映射到单调 ROS epoch，推荐 FAST-LIO 使用<br>legacy_system -- 使用主机系统时间，保留官方原始行为用于 A/B 回滚 | steady_raw |
| nosync_publish_mode | NoSync 模式下的点云切帧策略。<br>sensor_time -- 按传感器时间累计跨度发布，避免系统时间跳变导致停发或追赶式高频发布<br>legacy_timer -- 使用官方原始主机 timer 切帧，用于 A/B 回滚 | sensor_time |
| min_points_per_frame | 丢弃低于该点数的点云帧。默认 0 表示不强制丢弃，保持 legacy 兼容。FAST-LIO 现场测试时，可在确认 MID360 正常单帧点数后再设置保守阈值。 | 0 |

  **注意：**

未完全理解前，不建议修改本表未列出的其它参数。

MID360 处于 NoSync 状态并配合 FAST-LIO 使用时，推荐保持 `nosync_time_mode=steady_raw` 和 `nosync_publish_mode=sensor_time`。这样 LiDAR/IMU 时间戳不再依赖 NTP、chrony、systemd-timesyncd 等机制可能调整的主机系统时间。PTP/GPS/gPTP 已同步分支仍保持 raw packet timestamp，不会被该策略转换或 clamp。

### 3.3 Livox_ros_driver2 点云数据格式说明

1. Livox pointcloud2(PointXYZRTLT) 点云格式：

```c
float32 x               # X 轴，单位:m
float32 y               # Y 轴，单位:m
float32 z               # Z 轴，单位:m
float32 intensity       # 反射率，0.0~255.0
uint8   tag             # Livox 点标签
uint8   line            # 雷达线号
float64 timestamp       # 点时间戳
```

  **注意：**

  每帧点数可能不同，但每个点都会携带自己的时间戳。

2. Livox 自定义数据包格式：

```c
std_msgs/Header header     # ROS 标准消息头
uint64          timebase   # 第一个点的时间
uint32          point_num  # 点云总点数
uint8           lidar_id   # 雷达设备 id
uint8[3]        rsvd       # 保留字段
CustomPoint[]   points     # 点云数据
```

自定义数据包中的 CustomPoint 格式：

```c
uint32  offset_time     # 相对 timebase 的偏移时间
float32 x               # X 轴，单位:m
float32 y               # Y 轴，单位:m
float32 z               # Z 轴，单位:m
uint8   reflectivity    # 反射率，0~255
uint8   tag             # Livox 点标签
uint8   line            # 雷达线号
```

3. PCL 标准 pointcloud2(pcl::PointXYZI) 格式，仅 ROS1 支持：

请参考 PCL 库 `point_types.hpp` 文件中的 `pcl::PointXYZI` 数据结构。

## 4. LiDAR 配置

LiDAR 的 IP、端口、数据类型等参数可通过 JSON 配置文件设置。单 HAP、单 MID360 和混合雷达配置文件均位于 `config` 目录。launch 文件中的 `user_config_path` 参数用于指定 JSON 配置文件路径。

### 4.1 单 HAP 配置示例

下面是 HAP 雷达配置示例，文件位于 `config/HAP_config.json`：

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # 协议类型索引，请不要修改该值
  },
  "HAP": {
    "device_type" : "HAP",
    "lidar_ipaddr": "",
    "lidar_net_info" : {
      "cmd_data_port": 56000,  # 命令端口
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # 主机 IP，可按实际修改
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # 主机 IP
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # 主机 IP
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # 要配置的 LiDAR IP
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

上面 JSON 文件中 LiDAR 配置参数说明：

**LiDAR 配置参数**

| 参数 | 类型 | 说明 | 默认值 |
| :--- | ---- | ---- | ------ |
| ip | String | 要配置的 LiDAR IP | 192.168.1.100 |
| pcl_data_type | Int | 选择点云数据分辨率。<br>1 -- 笛卡尔坐标数据，32 bit<br>2 -- 笛卡尔坐标数据，16 bit<br>3 -- 球坐标数据 | 1 |
| pattern_mode | Int | 空间扫描模式。<br>0 -- 非重复扫描模式<br>1 -- 重复扫描模式<br>2 -- 重复扫描模式，低扫描速率 | 0 |
| blind_spot_set | Int | 设置盲区，仅 HAP 支持。范围 50 cm 到 200 cm。 | 50 |
| extrinsic_parameter | Object | 设置外参。`roll`、`pitch`、`yaw` 为 float；`x`、`y`、`z` 为 int。 | |

HAP 配置文件的更多说明请参考：
[HAP 配置文件说明](https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description)

### 4.2 多雷达混合配置示例

连接多台雷达时，需要在 `lidar_configs` 数组中为每台雷达添加对应对象。下面是 HAP + MID360 混合配置示例：

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # 协议类型索引，请不要修改该值
  },
  "HAP": {
    "lidar_net_info" : {  # HAP 端口，请不要修改这些值
      "cmd_data_port": 56000,  # HAP 命令端口
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # 主机 IP
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # 主机 IP
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # 主机 IP
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "MID360": {
    "lidar_net_info" : {  # MID360 端口，请不要修改这些值
      "cmd_data_port": 56100,  # MID360 命令端口
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # 主机 IP
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.5",  # 主机 IP
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.5",  # 主机 IP
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.5",  # 主机 IP
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # 要配置的 HAP IP
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    },
    {
      "ip" : "192.168.1.12",  # 要配置的 MID360 IP
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

### 4.3 多网卡连接多台 MID360 示例

当主机使用多张网卡连接多台雷达时，需要为不同雷达分别准备配置文件，并分别运行不同 launch 文件。下面是两个 MID360 配置文件和两个 launch 的示例。

**MID360_config1：**

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # 协议类型索引，请不要修改该值
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,  # 命令端口
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": [
      {
        "lidar_ip": ["192.168.1.100"],  # 雷达 IP
        "host_ip": "192.168.1.5",  # 主机 IP
        "cmd_data_port": 56101,
        "push_msg_port": 56201,
        "point_data_port": 56301,
        "imu_data_port": 56401,
        "log_data_port": 56501
      }
    ]
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.100",  # 要配置的 LiDAR IP
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

**MID360_config2：**

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # 协议类型索引，请不要修改该值
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,  # 命令端口
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": [
      {
        "lidar_ip": ["192.168.2.100"],  # 雷达 IP
        "host_ip": "192.168.2.5",  # 主机 IP
        "cmd_data_port": 56101,
        "push_msg_port": 56201,
        "point_data_port": 56301,
        "imu_data_port": 56401,
        "log_data_port": 56501
      }
    ]
  },
  "lidar_configs": [
    {
      "ip": "192.168.2.100",  # 要配置的 LiDAR IP
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

**Launch1：**

```xml
<launch>
    <!-- ROS 用户配置参数开始 -->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!-- ROS 用户配置参数结束 -->

    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config1.json"/> <!-- MID360_config1 文件名 -->
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

    <node name="livox_lidar_publisher1" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>

    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
              args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
              args="-a"/>
    </group>
</launch>
```

**Launch2：**

```xml
<launch>
    <!-- ROS 用户配置参数开始 -->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!-- ROS 用户配置参数结束 -->

    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config2.json"/> <!-- MID360_config2 文件名 -->
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

    <node name="livox_lidar_publisher2" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>

    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
              args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
              args="-a"/>
    </group>
</launch>
```

## 5. 支持的 LiDAR 类型

* HAP
* MID360
* 更多类型将持续支持

## 6. 常见问题

### 6.1 使用 `livox_lidar_rviz_HAP.launch` 启动后，RViz 网格上没有点云显示？

请检查 RViz 左侧 `Display` 面板中的 `Global Options - Fixed Frame` 字段。将该字段设置为 `livox_frame`，并确认已勾选 `PointCloud2` 显示项。

### 6.2 使用 `ros2 launch livox_lidar_rviz_HAP_launch.py` 启动时报错：无法打开 `liblivox_sdk_shared.so`？

请将 `/usr/local/lib` 添加到环境变量 `LD_LIBRARY_PATH`。

如果只想对当前终端生效：

```shell
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
```

如果希望对当前用户长期生效：

```shell
vim ~/.bashrc
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
source ~/.bashrc
```
