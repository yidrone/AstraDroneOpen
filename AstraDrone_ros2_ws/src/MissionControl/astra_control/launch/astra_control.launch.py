import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 加载参数文件
    config = os.path.join(
        get_package_share_directory('astra_control'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='astra_control',
            executable='astra_control_node',
            name='astra_control',
            output='screen',
            parameters=[config]
        )
    ])