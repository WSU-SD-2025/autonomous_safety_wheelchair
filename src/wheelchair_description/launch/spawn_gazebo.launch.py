import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ======================= Package Paths ======================= #
    pkg_wheelchair_description = get_package_share_directory("wheelchair_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_navigation = get_package_share_directory("navigation")
    pkg_human = get_package_share_directory("human_dummy")



    # ======================= File Paths ========================== #
    # Wheelchair URDF
    urdf_path = os.path.join(
        pkg_wheelchair_description, "wheelchair", "urdf", "robot.urdf"
    )
    assert os.path.exists(urdf_path), f"URDF not found: {urdf_path}"

    # Actor robot (instead of human)
    actor_sdf_path = os.path.join(pkg_human, "human", "actor_robot.sdf")
    assert os.path.exists(actor_sdf_path), f"Actor SDF not found: {actor_sdf_path}"

    # World
    world_path = os.path.join(pkg_wheelchair_description, "worlds", "empty.sdf")

    # ROS ↔ Gazebo bridge (all topics are defined in gz_bridge.yaml)
    bridge_config_path = PathJoinSubstitution(
        [pkg_navigation, "config", "gz_bridge.yaml"]
    )



    # ================== Robot Description (URDF) ================= #
    # robot_state_publisher
    with open(urdf_path, "r") as f:
        urdf_xml = f.read()
    robot_description = ParameterValue(urdf_xml, value_type=str)

    # ================== Global Environment Setting =============== #
    # Force to use CycloneDDS
    set_rmw = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value="rmw_cyclonedds_cpp",
    )



    # ==================== Gazebo / Simulation ==================== #
    # Ignition Gazebo 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    # ROS ↔ Gazebo Bridge
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": bridge_config_path,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )



    # ================= Robot State Publisher ===================== #
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description,
                "qos_overrides./clock.subscription.reliability": "best_effort",
                "qos_overrides./clock.subscription.durability": "volatile",
                "qos_overrides./clock.subscription.history": "keep_last",
                "qos_overrides./clock.subscription.depth": 1,
            }
        ],
    )



    # ===================== Spawn Entities ======================== #
    # Spawn wheelchair URDF
    spawn_wheelchair = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            urdf_path,
            "-name",
            "wheelchair",
            "-allow_renaming",
            "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.3",
        ],
    )

    # Spawn actor robot (instead of caregiver)
    spawn_actor = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            actor_sdf_path,
            "-name",
            "human_actor",
            "-allow_renaming",
            "true",
            "-x", "2.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )
    #caregiver position publisher node
    caregiver_pos_node = Node(
    package="human_dummy", 
    executable="caregiver_pos_publisher.py",
    name="caregiver_pos_publisher",
    output="screen",
    parameters=[
        {"use_sim_time": True}
    ],
)


    # ===================== Delayed Actions ======================= #
    delayed_rsp = TimerAction(period=1.5, actions=[robot_state_publisher])
    delayed_spawn_wheelchair = TimerAction(period=1.5, actions=[spawn_wheelchair])



    # ==================== Launch Description ===================== #
    # 1) set_rmw
    # 2) gazebo
    # 3) gz_bridge
    # 4) robot_state_publisher (delayed)
    # 5) wheelchair spawn (delayed)
    # 6) actor spawn
    # 7) caregiver position publisher node
    return LaunchDescription(
        [
            set_rmw,
            gazebo,
            gz_bridge,
            delayed_rsp,
            delayed_spawn_wheelchair,
            spawn_actor,
            caregiver_pos_node,
        ]
    )
