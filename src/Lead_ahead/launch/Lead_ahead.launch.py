from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lead_ahead')
    config = os.path.join(pkg_share, 'config', 'Lead_ahead_params.yaml')

    return LaunchDescription([
        Node(
            package='lead_ahead',
            executable='lead_ahead_node',
            name='lead_ahead_node',
            output='screen',
            parameters=[config]
        )
    ])
