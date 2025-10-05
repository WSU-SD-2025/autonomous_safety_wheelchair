from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')
    nav2_params  = LaunchConfiguration('nav2_params')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart',    default_value='true'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([
                get_package_share_directory('navigation'), 'config', 'nav2_params.yaml'
            ])
        ),

        # ✅ Nav2 Bringup (Navigation only — no map_server, no amcl)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/navigation_launch.py'  
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart':    autostart,
                'params_file':  nav2_params
            }.items()
        ),
    ])
