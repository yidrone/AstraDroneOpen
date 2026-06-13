import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = os.path.expanduser('~/livox_test.lvx')
cmdline_bd_code = 'livox0000000001'
# NoSync 时间戳模式：
# steady_raw    推荐值。NoSync 下用 MID360 包内 raw uint64 ns 上电时间，映射成单调 ROS 时间；
#               Jetson 被 NTP/chrony/systemd-timesyncd 校时也不会让 /livox/lidar 或 /livox/imu 时间戳回退。
# legacy_system 回滚值。恢复官方原始行为：NoSync 下使用主机系统时间；仅用于 A/B 对比。
#               如果系统时间跳变，FAST-LIO 可能出现 lidar/imu loop back、清 buffer、odom 停止。
nosync_time_mode = 'steady_raw'
# NoSync 发布切帧模式：
# sensor_time   推荐值。按雷达时间累计到 publish_freq 对应跨度后发布一帧；
#               可避免系统时间跳变导致 /livox/lidar 停发、追赶式补发、1000Hz 小帧。
# legacy_timer  回滚值。恢复官方主机 timer 切帧；仅用于确认问题是否来自新切帧逻辑。
nosync_publish_mode = 'sensor_time'
# 小帧过滤阈值：
# 0             默认值，不按点数丢帧，最大兼容官方行为。
# >0            丢弃点数低于该值的点云帧，可过滤异常 tiny frame；阈值过高会误删正常帧。
#               建议先 ros2 topic echo/统计正常 MID360 单帧点数，再设置保守值。
min_points_per_frame = 0

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
    {"nosync_time_mode": nosync_time_mode},
    {"nosync_publish_mode": nosync_publish_mode},
    {"min_points_per_frame": min_points_per_frame}
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([
        livox_driver,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
