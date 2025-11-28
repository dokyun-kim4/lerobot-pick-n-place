# LeRobot Pick N Place

This repository uses HuggingFace's SO-101 arm to perform a simple pick-n-place using ROS2 and MoveIt. The project is containzerized in a devcontainer for repeatability. 

## Setup
```bash
# 1. Install Docker and VSCode with Dev Containers extension

# 2. Enable display forwarding in host machine
xhost +local:host

# 3. Clone the repository
git clone https://github.com/dokyun-kim4/lerobot-pick-n-place.git

# 4. Open the repository in VSCode and open in a devcontainer. This will take a few minutes to build the container.

# 5. Build & source the ROS2 workspace
colcon build
source install/setup.bash
```

## Arm Bringup
Connect to the arm with the `lerobot_control` package. The launch file takes in 4 parameters:
- **usb_port**: Serial port of the arm (default: `/dev/LeRobot`); Override with your own port.
- **hardware_type**: Type of hardware being used (default: `mock_components`). Override with `real` for physical arm.
- **ros2_control_xacro_file**: Path to the xacro file for ros2_control (default: `lerobot_control/urdf/lerobot_ros2_control.xacro`).
- **controller_config_file**: Path to the controller config file (default: `lerobot_control/config/lerobot_controllers.yaml`).
```bash
# For physical arm
ros2 launch lerobot_control lerobot_control.launch.py \
usb_port:=/dev/ttyACM0 \
hardware_type:=real
```

## Moveit

### Running MoveIt Demo

After sourcing your workspace, run
```bash
ros2 launch lerobot_moveit_config demo.launch.py
```

### Editing MoveIt Configuration

`lerobot_moveit_config` package created with MoveIt setup assistant. Run 
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
choose 'Edit Existing MoveIt Configuration Package', and select the `lerobot_moveit_config` package directory to make changes.

When files are regenerated, you need to change the `max_velocity` values in [`joint_limits.yaml`](/ros_ws/src/lerobot_moveit_config/config/joint_limits.yaml) to be floats and not integers (change from `10` to `10.0`). If you don't, ROS2 will throw an error.
