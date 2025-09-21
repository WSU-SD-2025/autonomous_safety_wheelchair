import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Packages
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_wheelchair_description = get_package_share_directory('wheelchair_description')

    pkg_navigation = get_package_share_directory('navigation')
    
    # Paths
    robot_xacro_path = PathJoinSubstitution([
        pkg_wheelchair_description, 'wheelchair', 'urdf', 'robot.xacro'
    ])


    #world_path = os.path.join(pkg_wheelchair_description, 'worlds', 'empty.sdf')
    world_path = os.path.join(pkg_wheelchair_description, 'worlds', 'course_world.sdf')

    bridge_config_path = PathJoinSubstitution(
        [pkg_navigation, 'config', 'gz_bridge.yaml']
    )

    # 1) Gazebo Ignition Execution
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}'
        }.items(),
    )

    # 2) Robot State Publisher Execution
    from launch.substitutions import TextSubstitution

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            TextSubstitution(text=' '),  # adds space
            robot_xacro_path
        ]),
        value_type=str
    )



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

    # 3) Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'wheelchair',
                   '-allow_renaming', 'true',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.3'],
    )

    # 4) ROS-Gazebo Bridge (Corrected Version)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # The long list of arguments is now managed by the YAML file
        arguments=[],
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
        gz_bridge
    ])