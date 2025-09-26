from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    description_path = get_package_share_directory("lerobot_description")
    urdf_path = os.path.join(description_path, "urdf", "lerobot.urdf")

    model_arg = DeclareLaunchArgument(
                                        name="model", 
                                        default_value=urdf_path,
                                        description="Absolute path to the robot urdf file"
                                    )
    
    with open(urdf_path, 'r') as file:
        urdf = file.read()


    robot_state_publisher = Node(
                                    package= "robot_state_publisher",
                                    executable= "robot_state_publisher",
                                    parameters= [{"robot_description": urdf}]
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
                    arguments=["-d", os.path.join(description_path, "rviz", "display.rviz")]
                )




    return LaunchDescription(
                            [
                                model_arg,
                                robot_state_publisher,
                                joint_state_publisher,
                                rviz2
                            ]
                            )
