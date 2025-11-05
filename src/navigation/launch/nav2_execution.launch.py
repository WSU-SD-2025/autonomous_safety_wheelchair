from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetRemap, Node

def generate_launch_description():
    # ---------------------------
    # Launch arguments
    # ---------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')
    nav2_params  = LaunchConfiguration('nav2_params')

    scan_in = LaunchConfiguration('scan_in')        # raw LiDAR topic
    scan_out = LaunchConfiguration('scan_out')       # filtered LiDAR topic
    scan_filter_yaml = LaunchConfiguration('scan_filter_yaml')

    pkg_nav = get_package_share_directory('navigation')
    default_params = PathJoinSubstitution([pkg_nav, 'config', 'nav2_params.yaml'])
    default_scan_filter = PathJoinSubstitution([pkg_nav, 'config', 'scan_filter.yaml'])
    default_mux_yaml = PathJoinSubstitution([get_package_share_directory('wheelchair_mux'), 'config', 'twist_mux.yaml'])

    return LaunchDescription([
        # Use simulation time throughout Nav2 nodes
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Autostart Nav2 lifecycle nodes
        DeclareLaunchArgument('autostart',    default_value='true'),
        # Nav2 parameter file
        DeclareLaunchArgument('nav2_params',  default_value=default_params),


        # LiDAR filter I/O & config
        DeclareLaunchArgument('scan_in', default_value='/lidar/scan'),
        DeclareLaunchArgument('scan_out', default_value='/scan'),
        DeclareLaunchArgument('scan_filter_yaml', default_value=default_scan_filter),


        # Force RMW to CycloneDDS so Gazebo↔ROS2 bridges and Nav2 use the same DDS
        SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),


        # LiDAR filter chain
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filters',
            output='screen',
            parameters=[scan_filter_yaml],
            remappings=[('scan', scan_in), ('scan_filtered', scan_out)],
        ),


        # Twist mux (Keyboard + Nav2)
        Node(
            package = 'twist_mux',
            executable = 'twist_mux',
            name = 'twist_mux',
            output = 'screen',
            parameters = [default_mux_yaml],
            remappings=[('/cmd_vel_out', '/cmd_vel')],
        ),


        # Global remaps for Nav2 graph:
        SetRemap(src='controller_server/cmd_vel', dst='/cmd_vel_nav'),
        SetRemap(src='velocity_smoother/cmd_vel', dst='/cmd_vel_nav'),
        SetRemap(src='velocity_smoother/cmd_vel_smoothed',   dst='/cmd_vel_smooth'),
        SetRemap(src='behavior_server/cmd_vel', dst='/cmd_vel_nav'),
        SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),


        # Bring up core Nav2 stack (planner/controller/bt_navigator/…)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                # get_package_share_directory('nav2_bringup'),
                pkg_nav,
                '/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart':    autostart,
                'params_file':  nav2_params
            }.items()
        ),


    ])
