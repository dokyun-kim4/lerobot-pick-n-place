import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Pre-define all controller nodes to launch using `controller_manager spawner`
startup_controllers = [
                        "joint_state_broadcaster",
                        "gripper_controller",
                        "arm_controller"
                    ]
        
def generate_launch_description():
    lerobot_desc_dir = get_package_share_path("lerobot_description")
    lerobot_ctrl_dir = get_package_share_path("lerobot_control")


    # ----- Set up LaunchArguments ----- #
    usb_port_arg = DeclareLaunchArgument(
        "usb_port",
        default_value="/dev/LeRobot",
        description="USB port for robot. Used when `hardware_type` is real"
    )

    hardware_type_arg = DeclareLaunchArgument(
        "hardware_type",
        default_value="mock_components",
        description="Hardware type for robot. Supported types: [mock_components, real]"
    )

    ros2_control_xacro_file_arg = DeclareLaunchArgument(
        "ros2_control_xacro_file",
        default_value=os.path.join(
            lerobot_desc_dir, "urdf", "lerobot_ros2_control.xacro"
        ),
        description="Full path to the ros2_control xacro file",
    )

    controller_config_file_arg = DeclareLaunchArgument(
        "controller_config_file",
        default_value=os.path.join(lerobot_ctrl_dir, "config", "lerobot_controllers.yaml"),
        description="Full path to the controller configuration file to use",
    )

    ros2_controllers_file = LaunchConfiguration("controller_config_file")


    # ----- Set up StatePublisher node ----- #
    xacro_file = os.path.join(lerobot_desc_dir, 'urdf', 'lerobot.xacro')
    xacro_args = {
        "ros2_control_file": os.path.join(
            lerobot_desc_dir, "urdf", "lerobot_ros2_control.xacro"
        ),
        "usb_port": "/dev/ttyACM0",
        "ros2_control_hardware_type": "real"
    }
    
    cmd = ["xacro ", xacro_file]
    for key, value in xacro_args.items():
        cmd.append(f" {key}:={value}")

    robot_description = ParameterValue(
        Command(cmd),
        value_type=str
    )

    state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
        ],
    )
    # ----- Set up ALL controller nodes ----- #
    controller_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller],
        )
        for controller in startup_controllers
    ]

    return LaunchDescription(
        [
            hardware_type_arg,
            usb_port_arg,
            ros2_control_xacro_file_arg,
            controller_config_file_arg,

            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[{"robot_description": robot_description}, ros2_controllers_file],
                output="screen",
                emulate_tty=True,
            ),
            state_publisher_node
        ]
        +
        controller_nodes
    )  
