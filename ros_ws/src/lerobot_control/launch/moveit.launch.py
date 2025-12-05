from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription

LEROBOT_DESC_DIR = get_package_share_path("lerobot_description")
LEROBOT_CTRL_DIR = get_package_share_path("lerobot_control")

def build_nodes_runtime(context):
    usb_port = LaunchConfiguration("usb_port").perform(context)
    hardware_type = LaunchConfiguration("hardware_type").perform(context)

    # Define xacro mappings for the robot description file
    launch_arguments: dict[SomeSubstitutionsType, SomeSubstitutionsType] = {
        "usb_port": usb_port,
        "ros2_control_hardware_type": hardware_type,
    }

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "lerobot", package_name="lerobot_moveit_config"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", os.path.join(LEROBOT_DESC_DIR, "rviz", "display.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("lerobot_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lerobot_arm_controller", "-c", "/controller_manager"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lerobot_gripper_controller", "-c", "/controller_manager"],
    )

    return [
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        hand_controller_spawner,
    ]


def generate_launch_description():
    # ----- Set up LaunchArguments ----- #

    usb_port_arg = DeclareLaunchArgument(
        "usb_port",
        default_value="/dev/LeRobot",
        description="USB port for robot. Used when `hardware_type` is real",
    )

    hardware_type_arg = DeclareLaunchArgument(
        "hardware_type",
        default_value="mock_components",
        description="Hardware type for robot. Supported types: [mock_components, real]",
    )

    return LaunchDescription(
        [
            usb_port_arg,
            hardware_type_arg,
            OpaqueFunction(function=build_nodes_runtime),
        ]
    )
