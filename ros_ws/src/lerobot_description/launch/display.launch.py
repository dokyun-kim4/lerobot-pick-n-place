from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory("lerobot_description")
    xacro_file = os.path.join(pkg_share, 'urdf', 'lerobot.xacro')
    robot_description = ParameterValue(Command([
        'xacro ', xacro_file
        ]),
        value_type=str
    )


    robot_state_publisher = Node(
                                    package= "robot_state_publisher",
                                    executable= "robot_state_publisher",
                                    parameters= [{"robot_description": robot_description}]
                                )
    
    joint_state_publisher = Node(
                                    package= "joint_state_publisher_gui",
                                    executable= "joint_state_publisher_gui"
                                )
    
    rviz2 = Node(
                    package= "rviz2",
                    executable= "rviz2",
                    name= "rviz2",
                    output= "screen",
                    arguments=["-d", os.path.join(pkg_share, "rviz", "display.rviz")]
                )




    return LaunchDescription(
                            [
                                robot_state_publisher,
                                joint_state_publisher,
                                rviz2
                            ]
                            )
