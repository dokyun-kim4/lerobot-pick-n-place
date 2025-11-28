import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Pre-define all controller nodes to launch using `controller_manager spawner`
STARTUP_CONTROLLERS = ["joint_state_broadcaster", "gripper_controller", "arm_controller"]

LEROBOT_DESC_DIR = get_package_share_path("lerobot_description")
LEROBOT_CTRL_DIR = get_package_share_path("lerobot_control")


def build_nodes_runtime(context):
    usb_port = LaunchConfiguration("usb_port").perform(context)
    hardware_type = LaunchConfiguration("hardware_type").perform(context)
    ros2_control_xacro_file = LaunchConfiguration("ros2_control_xacro_file").perform(context)
    controller_config_file = LaunchConfiguration("controller_config_file").perform(context)

    # ---- Build xacro file ---- #
    xacro_file = os.path.join(LEROBOT_DESC_DIR, "urdf", "lerobot.xacro")
    xacro_args = {
        "ros2_control_file": ros2_control_xacro_file,
        "usb_port": usb_port,
        "ros2_control_hardware_type": hardware_type,
    }

    cmd = ["xacro ", xacro_file]
    for key, value in xacro_args.items():
        cmd.append(f" {key}:={value}")

    robot_description = ParameterValue(Command(cmd), value_type=str)

    all_nodes = []

    # ----- Set up robot_state_publisher node ----- #
    all_nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description},
            ],
        )
    )

    # ----- Set up controller manager nodes ----- #
    all_nodes.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, controller_config_file],
            output="screen",
            emulate_tty=True,
        )
    )

    # ----- Set up all controllers defined in STARTUP_CONTROLLERS ----- #
    for controller in STARTUP_CONTROLLERS:
        all_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
            )
        )

    # ----- Rviz for visualization ----- #
    all_nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(LEROBOT_DESC_DIR, "rviz", "display.rviz")],
        )
    )

    return all_nodes


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

    ros2_control_xacro_file_arg = DeclareLaunchArgument(
        "ros2_control_xacro_file",
        default_value=os.path.join(LEROBOT_DESC_DIR, "urdf", "lerobot_ros2_control.xacro"),
        description="Full path to the ros2_control xacro file",
    )

    controller_config_file_arg = DeclareLaunchArgument(
        "controller_config_file",
        default_value=os.path.join(LEROBOT_CTRL_DIR, "config", "lerobot_controllers.yaml"),
        description="Full path to the controller configuration file to use",
    )

    return LaunchDescription(
        [
            usb_port_arg,
            hardware_type_arg,
            ros2_control_xacro_file_arg,
            controller_config_file_arg,
            OpaqueFunction(function=build_nodes_runtime),
        ]
    )
