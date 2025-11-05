from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    scan_topic   = LaunchConfiguration('scan_topic')

    pkg_share = get_package_share_directory('navigation')
    default_params = PathJoinSubstitution([
        pkg_share, 'config', 'mapper_params_online_async.yaml'
    ])

    # --- Environment setup: force CycloneDDS & consistent domain ---
    set_rmw   = SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp')
    set_dom   = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='88')  # same as gps_localization
    set_assert = SetEnvironmentVariable(name='RCL_ASSERT_RMW_ID_MATCHES', value='rmw_cyclonedds_cpp')

    # --- Shared parameters for sim time and /clock QoS ---
    common_params = [{
        'use_sim_time': True,
        'qos_overrides./clock.subscription.reliability': 'best_effort',
        'qos_overrides./clock.subscription.durability': 'volatile',
        'qos_overrides./clock.subscription.history': 'keep_last',
        'qos_overrides./clock.subscription.depth': 1,
    }]

    # --- SLAM Toolbox (online async mode) ---
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file] + common_params,
        remappings=[('/scan', scan_topic)],
    )

    # --- Launch Description ---
    return LaunchDescription([
        set_rmw,
        set_dom,
        set_assert,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file',  default_value=default_params),
        DeclareLaunchArgument('scan_topic',   default_value='/lidar/scan'),
        slam_toolbox_node
    ])
