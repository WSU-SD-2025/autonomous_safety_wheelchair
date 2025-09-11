import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ===================================================================================
    # ================================ Config Paths =====================================
    # ===================================================================================
    pkg_navigation = get_package_share_directory('navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_wheelchair_description = get_package_share_directory('wheelchair_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Config files
    ekf_config_path = PathJoinSubstitution([pkg_navigation, 'config', 'ekf.yaml'])
    nav2_params_path = PathJoinSubstitution([pkg_navigation, 'config', 'nav2_params.yaml'])
    slam_params_path = PathJoinSubstitution([pkg_navigation, 'config', 'mapper_params_online_async.yaml'])
    gz_bridge_config_path = PathJoinSubstitution([pkg_navigation, 'config', 'gz_bridge.yaml'])

    # URDF/xacro
    robot_xacro_path = PathJoinSubstitution([
        pkg_wheelchair_description, 'wheelchair', 'urdf', 'robot.xacro'
    ])

    # Common arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ===================================================================================
    # ================================ Nodes & Actions ==================================
    # ===================================================================================

    # 0) Robot State Publisher
    #    - Publishes the static TF tree from URDF (base_link, lidar_link, etc.)
    robot_description = ParameterValue(
        Command(['xacro ', robot_xacro_path]),
        value_type=str
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # 0.5) ros_gz_bridge (parameter_bridge)
    #     - Bridges Ignition(GZ) topics (odom, scan, imu, clock, cmd_vel...) to ROS 2 using gz_bridge.yaml
    #     - Note: ros_gz_bridge does NOT rewrite message frame_id/child_frame_id. TF is published by EKF/SLAM.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': gz_bridge_config_path,
            # Make /tf_static durable for late joiners (typical pattern)
            'qos_overrides./tf_static.publisher.durability': 'transient_local'
        }]
    )

    # 1) Robot Localization (EKF)
    #    - Consumes /odom and /imu/data and publishes odom->base_* TF
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    # 2) SLAM Toolbox (official launch include)
    #    - Subscribes to /lidar/scan, publishes /map and map->odom TF (mapping mode)
    #    - Uses mapper_params_online_async.yaml in the "slam_toolbox: ros__parameters:" structure.
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': slam_params_path,
        }.items()
    )

    # 3) Nav2 Bringup (navigation_launch.py)
    #    - Starts the Nav2 stack (BT navigator, planner, controller, etc.)
    start_nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': nav2_params_path,
        }.items()
    )

    # ===================================================================================
    # ============================ Event Handlers / Ordering =============================
    # ===================================================================================

    # Start Nav2 only after SLAM Toolbox has started.
    nav2_start_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_slam_toolbox_cmd,
            on_start=[
                LogInfo(msg='SLAM Toolbox started, launching Nav2...'),
                start_nav2_bringup_cmd
            ]
        )
    )

    # ===================================================================================
    # ================================ Launch Description ===============================
    # ===================================================================================
    # Order:
    #   - robot_state_publisher + bridge first (URDF TF + ROS<->GZ topics ready)
    #   - EKF next (publishes odom->base_* TF)
    #   - SLAM next (publishes map->odom TF, map building)
    #   - Nav2 after SLAM is alive
    return LaunchDescription([
        robot_state_publisher,
        gz_bridge,
        start_robot_localization_cmd,
        start_slam_toolbox_cmd,
        nav2_start_event_handler,
    ])
