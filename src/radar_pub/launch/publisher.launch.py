from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
            get_package_share_directory('radar_pub'),
            'config',
            'params.yaml')

    return LaunchDescription([
        Node(
            package='radar_pub',
            executable='radar_pub',
            name='radar_pub',
            output='screen',
            parameters=[config_file],
        ),
    ])
