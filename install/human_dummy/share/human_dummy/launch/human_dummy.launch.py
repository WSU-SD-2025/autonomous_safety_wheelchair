import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    ### DATA INPUT ###
    urdf_file = 'human_dummy.urdf'
    package_description = "human_dummy"
    
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "human", urdf_file)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': ParameterValue(Command(['xacro ', robot_desc_path]), value_type=str)}],
        output="screen"
    )

    return LaunchDescription(
        [
            robot_state_publisher_node
        ]
    )