#include <memory>
#include <vector>
#include <string>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "lerobot_routine/routine.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "run_routine",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("run_routine");
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group = MoveGroupInterface(node, "lerobot_arm");
  auto gripper_group = MoveGroupInterface(node, "lerobot_gripper");

  std::string routine_name;
  node->get_parameter("routine_name", routine_name);

  if (routine_name == "") {
    RCLCPP_ERROR(logger, "No routine_name parameter provided");
    return 1;
  }

  std::string routine_file_path = "/workspaces/lerobot-pick-n-place/ros_ws/src/lerobot_routine/config/" + routine_name + ".yaml";
  std::vector<Task> routine;
  try {
    routine = load_routine(routine_file_path);
    RCLCPP_INFO(logger, "Loaded routine: %s with %zu steps", routine_name.c_str(), routine.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Failed to load routine: %s", e.what());
    return 1;
  }
  


  // lambda for executing task
  auto run_task = [&](const Task task) -> bool
  {
    std::string group = task.target;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (group == "arm")
    {
      arm_group.setStartStateToCurrentState();
      if (task.named_target == "") // given pose
      {
        arm_group.setPoseTarget(task.pose);
      }
      else // given named_target
      {
        arm_group.setNamedTarget(task.named_target);
      }

      bool success = static_cast<bool>(arm_group.plan(plan));
    
      if (success) {
        arm_group.execute(plan);
        return true;
      } else {
        RCLCPP_ERROR(logger, "Arm planning failed");
        return false;
      }

    }
    else if (group == "gripper")
    {
      gripper_group.setJointValueTarget({task.gripper_position});
      bool success = static_cast<bool>(gripper_group.plan(plan));
    
      if (success) {
        gripper_group.execute(plan);
        return true;
      } else {
        RCLCPP_ERROR(logger, "Gripper planning failed");
        return false;
      }
    }
  };

  // Execute routine steps
  RCLCPP_INFO(logger, "Starting routine execution...");
  
  for (size_t idx=0; idx < routine.size(); idx++) {
    Task task = routine[idx];
    RCLCPP_INFO(logger, "Executing step %zu: target=%s", idx, task.target.c_str());
    
    run_task(task);  
  }

  RCLCPP_INFO(logger, "routine completed successfully!");

  rclcpp::shutdown();
  return 0;
}
