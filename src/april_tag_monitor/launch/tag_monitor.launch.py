from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('april_tag_monitor')
    config_path = os.path.join(pkg_share, 'config', 'tag_monitor_params.yaml')

    return LaunchDescription([
        Node(
            package='april_tag_monitor',
            executable='tag_monitor_node',
            name='tag_monitor_node',
            output='screen',
            parameters=[config_path]
        )
    ])
