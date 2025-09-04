import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Packages
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_wheelchair_description = get_package_share_directory('wheelchair_description')
    
    # Paths
    robot_xacro = PathJoinSubstitution(
        [pkg_wheelchair_description, 'wheelchair', 'urdf', 'robot.xacro']
    )

    world_path = os.path.join(pkg_wheelchair_description, 'worlds', 'empty.sdf')




    # 1)Gazebo Ignition Execution
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            
            # Map
            'gz_args': f'-r {world_path}'
            }.items(),
    )

    #Robot State Publisher Execution
    #xacro -> robot_description
    robot_description =  ParameterValue(
        Command(['xacro ', robot_xacro]),
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


    #Spawn Robot

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'wheelchair',
                   '-allow_renaming', 'true',

                   # Spawn position
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.3'],
    )


    # ROS2 Gazebo Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Simulation Time Synchronization): Gazebo -> ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Odom Gazebo -> ROS
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Control Ros -> Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',




            #--------------------Sensors--------------------#

            # LiDAR Sensor Gazebo -> ROS
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',



            # RGB/Depth Camera Gazebo-> ROS#
            '/front_rgbd/rgb/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_rgbd/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
            'rear_rgbd/rgb/image@sensor_msgs/msg/Image[gz.msgs.Image',
            'rear_rgbd/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',

            '/front_rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/rear_rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        gz_bridge
    ])