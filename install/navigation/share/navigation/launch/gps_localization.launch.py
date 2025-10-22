from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('navigation')    
    config_all = PathJoinSubstitution([pkg, 'config', 'ekf.yaml'])

    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[config_all],  
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[config_all],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/gps', '/odometry/gps')
        ]
    )

    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map',
        output='screen',
        parameters=[config_all],
        remappings=[('odometry/filtered', '/odometry/map')]
    )

    return LaunchDescription([ekf_odom, navsat, ekf_map])
