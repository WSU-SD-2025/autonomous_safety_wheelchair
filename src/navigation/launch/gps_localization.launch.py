from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('navigation')    
    config_all = PathJoinSubstitution([pkg, 'config', 'ekf.yaml'])


    set_rmw = SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp')

    common_params = [{
        'use_sim_time': True,
        'qos_overrides./clock.subscription.reliability': 'best_effort',
        'qos_overrides./clock.subscription.durability': 'volatile',
        'qos_overrides./clock.subscription.history': 'keep_last',
        'qos_overrides./clock.subscription.depth': 1,
    }]

    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[config_all] + common_params,  
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[config_all] + common_params,
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
        parameters=[config_all] + common_params,
        remappings=[('odometry/filtered', '/odometry/map')]
    )


    delayed_ekf_odom = TimerAction(period=0.5, actions=[ekf_odom])
    delayed_navsat   = TimerAction(period=0.8, actions=[navsat])
    delayed_ekf_map  = TimerAction(period=1.0, actions=[ekf_map])

    return LaunchDescription([
        set_rmw,
        delayed_ekf_odom,
        delayed_navsat,
        delayed_ekf_map
    ])
