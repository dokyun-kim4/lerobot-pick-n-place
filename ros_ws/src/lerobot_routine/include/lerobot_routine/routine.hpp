#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose.hpp>

struct Task {
  std::string target = "";
  geometry_msgs::msg::Pose pose;
  double gripper_position = -1.0;
  std::string named_target = "";
};

std::vector<Task> load_routine(const std::string& file_path) {
  std::vector<Task> routine;
  
  try {
    // Read YAML from given path
    YAML::Node config = YAML::LoadFile(file_path);
    
    // Check for invalid YAML (missing "routine" or "tasks")
    if (!config["routine"] || !config["routine"]["tasks"]) {
      throw std::runtime_error("Missing key: routine or tasks");
    }
    
    for (const auto& cur_task : config["routine"]["tasks"]) {
      Task task;
      task.target = cur_task["target"].as<std::string>();
      
      if (task.target == "arm") {
        if (cur_task["pose"]) {
          auto pose_node = cur_task["pose"];
          auto pos = pose_node["position"];
          auto orient = pose_node["orientation"];
          
          task.pose.position.x = pos["x"].as<double>();
          task.pose.position.y = pos["y"].as<double>();
          task.pose.position.z = pos["z"].as<double>();
          task.pose.orientation.x = orient["x"].as<double>();
          task.pose.orientation.y = orient["y"].as<double>();
          task.pose.orientation.z = orient["z"].as<double>();
          task.pose.orientation.w = orient["w"].as<double>();
        } else if (cur_task["named_target"]) {
          task.named_target = cur_task["named_target"].as<std::string>();
        }
      } else if (task.target == "gripper") {
        task.gripper_position = cur_task["position"].as<double>();
      }
      
      routine.push_back(task);
    }
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("YAML parsing error: " + std::string(e.what()));
  }
  return routine;
}
