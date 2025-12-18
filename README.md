# LeRobot Arm Behaviors
This repository uses HuggingFace's SO-101 arm to perform tasks using ROS2 and MoveIt. The project is containzerized in a devcontainer for repeatability. 

## Setup
```bash
# 1. Install Docker and VSCode with Dev Containers extension

# 2. Enable display forwarding in host machine
xhost +local:host

# 3. Clone the repository
git clone https://github.com/dokyun-kim4/lerobot-arm-behaviors.git

# 4. Open the repository in VSCode and open in a devcontainer. This will take a few minutes to build the container.

# 5. Build & source the ROS2 workspace
colcon build
source install/setup.bash
```

## Arm Bringup (Default)
Connect to the arm with the `lerobot_control` package. The launch file takes in 4 parameters:
- **usb_port**: Serial port of the arm (default: `/dev/LeRobot`); Override with your own port.
- **hardware_type**: Type of hardware being used (default: `mock_components`). Override with `real` for physical arm.
- **ros2_control_xacro_file**: Path to the xacro file for ros2_control (default: `lerobot_control/urdf/lerobot_ros2_control.xacro`).
- **controller_config_file**: Path to the controller config file (default: `lerobot_control/config/lerobot_controllers.yaml`).
```bash
# For physical arm
ros2 launch lerobot_control controller.launch.py \
usb_port:=/dev/ttyACM0 \
hardware_type:=real
```
For launching the arm in simulation, run the command above with no additional parameters.

## Arm Bringup (with MoveIt)

Start the arm with MoveIt with the `lerobot_control moveit.launch.py` launch file. The launch file takes in 2 parameters:
- **usb_port**: Serial port of the arm (default: `/dev/LeRobot`); Override with your own port.
- **hardware_type**: Type of hardware being used (default: `mock_components`). Override with `real` for physical arm.
After sourcing your workspace, run
```bash
ros2 launch lerobot_control moveit.launch.py \
usb_port:=/dev/ttyACM0 \
hardware_type:=real
```
For launching the arm in simulation, run the command above with no additional parameters.

TODO
Add GIF of arm control via moveit

### Editing MoveIt Configuration

`lerobot_moveit_config` package created with MoveIt setup assistant. Run 
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
choose 'Edit Existing MoveIt Configuration Package', and select the `lerobot_moveit_config` package directory to make changes.

When files are regenerated, you need to change the `max_velocity` values in [`joint_limits.yaml`](/ros_ws/src/lerobot_moveit_config/config/joint_limits.yaml) to be floats and not integers (change from `10` to `10.0`). If you don't, ROS2 will throw an error.

## Teleop Control

To control the robot arm with the keyboard, first [bring up the arm](#arm-bringup-default). Unfortunately, teleop 
does not currently work with MoveIt because it uses different ROS controllers. Once the robot
has been started, run the teleop node with
```bash
ros2 run lerobot_teleop teleop
```

To control the robot, use the following controls:
| Keys | Command                          |
| ---- | ------------------------------ |
| Q/A  | Shoulder Pan +/-               |
| W/S  | Shoulder Lift +/-              |
| E/D  | Elbow Flex +/-                 |
| R/F  | Wrist Flex +/-                 |
| T/G  | Wrist Roll +/-                 |
| Y/H  | Gripper -/+                    |
| P    | Print the current joint states |
| L    | Quit the program               |

If you stop the program with `Ctrl+C`, you will need to press a key on the keyboard to exit the I/O loop and fully quit the program.

## Teaching a new behavior

This project provides the `lerobot_routine` package for teaching new behaviors to the arm. All behaviors are stored as YAML files under the `lerobot_routine/routines` directory.

To start recording a new behavior, run the `teach_routine` node.
```bash
ros2 run lerobot_routine teach_routine --ros-args -p routine_name:=<routine_name>
```
Replace `<routine_name>` with the desired name for the new behavior. This will save the routine to a new YAML file under `lerobot_routine/routines` with the specified name.

<video src="https://github.com/user-attachments/assets/e57877a2-b086-4985-bd37-f004141f3c1f">
</video>

Once the node is running, either use the teleop controls or the MoveIt GUI to move the arm through the desired sequence of positions. Each time you move the arm to a new position, press the `s` key in the terminal to record the current end effector position as a waypoint in the routine.

When you are finished recording the behavior, press `q` to stop the node. The new routine will be saved and can be executed using the `run_routine` node.

```bash
ros2 run lerobot_routine run_routine --ros-args -p routine_name:=<routine_name>
```
We provide a sample routine called `sample.yaml` that will run a simple pick and place sequence.

<video src="https://github.com/user-attachments/assets/cb4fb787-f29a-4944-9502-5e6ab826070c">
</video>
