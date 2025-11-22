from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Use simulation time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        #TF (human_actor/odom -> odom(wheelchair))
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_actor_to_odom',
            arguments=['2', '0', '0', '0', '0', '0', 'odom', 'human_actor/odom']
        ),

        # Caregiver Pose Node
        Node(
            package='caregiver',
            executable='caregiver_pose_node',
            name='caregiver_pose_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'offset_distance': 0.5,
                'odom_topic': '/human_actor/odometry',
                'output_topic': '/caregiver_pos',
                'global_frame': 'odom'
            }]
        ),

        # Mode Manager Node
        # Node(
        #     package='caregiver',
        #     executable='mode_manager',
        #     name='mode_manager',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'mode_key_topic': '/mode_key',
        #         'navigation_mode_topic': '/navigation_mode',
        #         'global_frame': 'odom'
        #     }]
        # )
    ])
