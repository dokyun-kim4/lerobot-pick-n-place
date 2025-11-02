

Make sure to run
```bash
xhost +local:host
```
on the host machine

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