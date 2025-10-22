import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Package Path
    pkg_wheelchair_description = get_package_share_directory('wheelchair_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_navigation = get_package_share_directory('navigation')
    pkg_human = get_package_share_directory('human_dummy')

    # URDF Path
    urdf_path = os.path.join(pkg_wheelchair_description, 'wheelchair', 'urdf', 'robot.urdf')
    assert os.path.exists(urdf_path), f'URDF not found: {urdf_path}'

    human_path = os.path.join(pkg_human, 'human', 'human_dummy.sdf')
    assert os.path.exists(human_path), f'Human SDF not found: {human_path}'

    # World file
    world_path = os.path.join(pkg_wheelchair_description, 'worlds', 'empty.sdf')
    #world_path = os.path.join(pkg_wheelchair_description, 'worlds', 'hospital.world')

    # Bridge YAML (navigation 패키지 안의 config/gz_bridge.yaml 사용)
    bridge_config_path = PathJoinSubstitution([pkg_navigation, 'config', 'gz_bridge.yaml'])

    # Gazebo Execution
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # Robot State Publisher: URDF
    with open(urdf_path, 'r') as f:
        urdf_xml = f.read()
    robot_description = ParameterValue(urdf_xml, value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }]
    )

    # Spawn in Gazebo directly from URDF file
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', urdf_path,
            '-name', 'wheelchair',
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.3'
        ],
    )

    spawn_actor = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', human_path,
            '-name', 'human_actor',
            '-allow_renaming', 'true',
            '-x', '2.0', '-y', '0.0', '-z', '0.0'
        ],
    )

    # ROS ↔ Gazebo Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        spawn_actor,
        gz_bridge
    ])
