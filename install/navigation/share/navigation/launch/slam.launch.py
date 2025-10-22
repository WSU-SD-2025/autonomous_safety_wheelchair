from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    scan_topic   = LaunchConfiguration('scan_topic')

    pkg_share = get_package_share_directory('navigation')
    default_params = PathJoinSubstitution([pkg_share, 'config', 'mapper_params_online_async.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file',  default_value=default_params),
        DeclareLaunchArgument('scan_topic',   default_value='/lidar/scan'),

        # slam_toolbox (online async)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            # remappings=[('/scan', scan_topic)]
        ),
    ])
