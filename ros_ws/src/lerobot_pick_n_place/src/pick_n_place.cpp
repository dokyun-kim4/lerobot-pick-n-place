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
  
  // Pose 1: enter pick pose
  target_poses.push_back([](){
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = -0.32288;
    pose.orientation.y = -0.5751;
    pose.orientation.z = -0.35788;
    pose.orientation.w = 0.661;
    pose.position.x = 0.11077;
    pose.position.y = -0.16598;
    pose.position.z = 0.058271;
    return pose;
  }());
  
  // move into object
  target_poses.push_back([](){
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = -0.32602;
    pose.orientation.y = -0.57861;
    pose.orientation.z = -0.35897;
    pose.orientation.w = 0.65579;
    pose.position.x = 0.1247;
    pose.position.y = -0.19031;
    pose.position.z = 0.057139;
    return pose;
  }());

  //  Lift
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

  // Place
  target_poses.push_back([](){
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = 0.50247;
    pose.orientation.y = 0.20858;
    pose.orientation.z = 0.75091;
    pose.orientation.w = -0.37438;
    pose.position.x = -0.1777;
    pose.position.y = -0.2515;
    pose.position.z = 0.16232;

    return pose;
  }());

  // Helper lambda to execute arm movement
  auto execute_arm_movement = [&](const geometry_msgs::msg::Pose& target) -> bool {
    arm_group.setStartStateToCurrentState();
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

  // 1. Open gripper
  if (!control_gripper(1.0)) {  // Open position (adjust value as needed)
    RCLCPP_ERROR(logger, "Failed to open gripper");
    return 1;
  }


  // 2. Move to pick location
  if (!execute_arm_movement(target_poses[0])) {
    RCLCPP_ERROR(logger, "Failed to move to pick location");
    return 1;
  }

  if (!execute_arm_movement(target_poses[1])) {
    RCLCPP_ERROR(logger, "Failed to move into object");
    return 1;
  }

  if (!control_gripper(0.2)) {  // Open position (adjust value as needed)
    RCLCPP_ERROR(logger, "Failed to open gripper");
    return 1;
  }

    // 5. Move to lift location
  if (!execute_arm_movement(target_poses[2])) {
    RCLCPP_ERROR(logger, "Failed to move to lift location");
    return 1;
  }

  RCLCPP_INFO(logger, "-------placing-----------");
  // 5. Move to place location
  if (!execute_arm_movement(target_poses[3])) {
    RCLCPP_ERROR(logger, "Failed to move to place location");
    return 1;
  }

  // 6. Open gripper to release object
  if (!control_gripper(1.0)) {  // Open position
    RCLCPP_ERROR(logger, "Failed to open gripper for release");
    return 1;
  }

  // 7. Move to home position
  arm_group.setStartStateToCurrentState();
  arm_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  bool home_success = static_cast<bool>(arm_group.plan(home_plan));
  if (home_success) {
    arm_group.execute(home_plan);
    RCLCPP_INFO(logger, "Arm returned to home position successfully");
  } else {
    RCLCPP_ERROR(logger, "Failed to return arm to home position");
    return 1;
  }

  if (!control_gripper(0.0)) {  // Open position (adjust value as needed)
    RCLCPP_ERROR(logger, "Failed to open gripper");
    return 1;
  }

  RCLCPP_INFO(logger, "Pick and place sequence completed successfully!");

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
