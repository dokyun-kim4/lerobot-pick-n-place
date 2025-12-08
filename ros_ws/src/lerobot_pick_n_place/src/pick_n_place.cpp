#include <memory>
#include <vector>
#include <string>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "lerobot_pick_n_place/mission.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pick_n_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("pick_n_place");
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group = MoveGroupInterface(node, "lerobot_arm");
  auto gripper_group = MoveGroupInterface(node, "lerobot_gripper");

  // Load mission from yaml
  std::string mission_file_path = "/workspaces/lerobot-pick-n-place/ros_ws/src/lerobot_pick_n_place/config/mission.yaml";
  std::vector<Task> mission;
  try {
    mission = load_mission(mission_file_path);
    RCLCPP_INFO(logger, "Loaded mission with %zu steps", mission.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Failed to load mission: %s", e.what());
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

  // Execute mission steps
  RCLCPP_INFO(logger, "Starting mission execution...");
  
  for (size_t idx=0; idx < mission.size(); idx++) {
    Task task = mission[idx];
    RCLCPP_INFO(logger, "Executing step %zu: target=%s", idx, task.target.c_str());
    
    run_task(task);  
  }

  RCLCPP_INFO(logger, "Mission completed successfully!");

  rclcpp::shutdown();
  return 0;
}
