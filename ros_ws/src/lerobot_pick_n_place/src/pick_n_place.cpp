#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pick_n_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("pick_n_place");

  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group = MoveGroupInterface(node, "lerobot_arm");
  auto gripper_group = MoveGroupInterface(node, "lerobot_gripper");

  // Define multiple target poses
  std::vector<geometry_msgs::msg::Pose> target_poses;
  
  // Pose 1: Pick
  target_poses.push_back([](){
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = -0.20679;
    pose.orientation.y = -0.3542;
    pose.orientation.z = -0.45621;
    pose.orientation.w = 0.78972;
    pose.position.x = 0.12712;
    pose.position.y = -0.2098;
    pose.position.z = 0.12846;
    return pose;
  }());

  // Pose 2: Lift
  target_poses.push_back([](){
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = -0.41798;
    pose.orientation.y = -0.35225;
    pose.orientation.z = -0.59529;
    pose.orientation.w = 0.58894;
    pose.position.x = 0.013342;
    pose.position.y = -0.10319;
    pose.position.z = 0.27606;
    return pose;
  }());

  // Pose 3: Place
  target_poses.push_back([](){
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = -0.46036;
    pose.orientation.y = -0.20238;
    pose.orientation.z = -0.78912;
    pose.orientation.w = 0.3527;
    pose.position.x = -0.18208;
    pose.position.y = -0.24734;
    pose.position.z = 0.10674;
    return pose;
  }());

  // Helper lambda to execute arm movement
  auto execute_arm_movement = [&](const geometry_msgs::msg::Pose& target) -> bool {
    arm_group.setPoseTarget(target);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(arm_group.plan(plan));
    
    if (success) {
      arm_group.execute(plan);
      RCLCPP_INFO(logger, "Arm movement executed successfully");
      return true;
    } else {
      RCLCPP_ERROR(logger, "Arm planning failed");
      return false;
    }
  };

  // Helper lambda to control gripper (forward position control)
  auto control_gripper = [&](double position) -> bool {
    // For position controllers, set joint target directly
    std::vector<double> gripper_joint_values = {position}; // Adjust based on your gripper joint count
    gripper_group.setJointValueTarget(gripper_joint_values);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(gripper_group.plan(plan));
    
    if (success) {
      gripper_group.execute(plan);
      RCLCPP_INFO(logger, "Gripper moved to position: %f", position);
      return true;
    } else {
      RCLCPP_ERROR(logger, "Gripper planning failed");
      return false;
    }
  };

  // Execute pick and place sequence
  RCLCPP_INFO(logger, "Starting pick and place sequence...");

  // // 1. Open gripper
  // if (!control_gripper(0.02)) {  // Open position (adjust value as needed)
  //   RCLCPP_ERROR(logger, "Failed to open gripper");
  //   return 1;
  // }

  // 2. Move to pick location
  if (!execute_arm_movement(target_poses[0])) {
    return 1;
  }

  // // 3. Close gripper to pick object
  // if (!control_gripper(0.0)) {  // Closed position
  //   RCLCPP_ERROR(logger, "Failed to close gripper");
  //   return 1;
  // }

    // 5. Move to lift location
  if (!execute_arm_movement(target_poses[1])) {
    return 1;
  }


  // 5. Move to place location
  if (!execute_arm_movement(target_poses[2])) {
    return 1;
  }

  // // 6. Open gripper to release object
  // if (!control_gripper(0.02)) {  // Open position
  //   RCLCPP_ERROR(logger, "Failed to open gripper for release");
  //   return 1;
  // }

  RCLCPP_INFO(logger, "Pick and place sequence completed successfully!");

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
