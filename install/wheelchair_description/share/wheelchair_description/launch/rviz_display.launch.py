from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    pkg = get_package_share_directory('wheelchair_description')
    urdf_path = Path(pkg) / 'wheelchair' / 'robot.urdf'
    robot_desc = urdf_path.read_text()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc,
                         'publish_frequency': 50.0}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(package='rviz2', executable='rviz2', name='rviz2')
    ])
