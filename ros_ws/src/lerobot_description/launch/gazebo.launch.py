import os
from ament_index_python import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Assuming ROS2 Humble
IS_IGNITION = "True"
ENGINE = ""

def generate_launch_description():

    pkg_share = get_package_share_directory("lerobot_description")
    xacro_file = os.path.join(pkg_share, 'urdf', 'lerobot.xacro')
    robot_description = Command([
        'xacro ', xacro_file
        ])


    # Ensure Gazebo can find mesh files; required to use `package://` in xacro
    gazebo_resource_path = SetEnvironmentVariable(
                                                    name="GZ_SIM_RESOURCE_PATH",
                                                    value=[str(Path(pkg_share).parent.resolve())]
                                                )

    gazebo_launch = IncludeLaunchDescription(
                                                PythonLaunchDescriptionSource([
                                                                                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                                                                                "/gz_sim.launch.py"
                                                ]),
                                                launch_arguments=[
                                                                    ("gz_args", [" -v 4 -r empty.sdf ", ENGINE]) # `-v 4` makes terminal display Gazebo outputs
                                                                ]
    )

    state_publisher_node = Node(
                                package = 'robot_state_publisher',
                                executable = 'robot_state_publisher',
                                parameters = [{'robot_description': robot_description, "use_sim_time": True}]
    )

    robot_spawn_node = Node(
                            package='ros_gz_sim',
                            executable='create',
                            arguments=['-topic', 'robot_description', '-name', 'lerobot'],
                            output='screen'
    )

    return LaunchDescription(
                            [
                                gazebo_resource_path,
                                gazebo_launch,
                                state_publisher_node,
                                robot_spawn_node
                            ]
                            )
