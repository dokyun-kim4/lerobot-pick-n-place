#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose.hpp>
#include <fstream>

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


void save_routine(const std::string& routine_name, const std::vector<Task>& routine_vec)
{
  YAML::Emitter out;
  out << YAML::BeginMap; // Begin File
  out << YAML::Key << "routine" << YAML::Value << YAML::BeginMap; // Begin Routine
  out << YAML::Key << "name" << YAML::Value << routine_name;
  out << YAML::Key << "tasks" << YAML::Value << YAML::BeginSeq; // Begin Tasks

  for (Task task: routine_vec)
  { 
    out << YAML::BeginMap;
    out << YAML::Key << "target" << YAML::Value << task.target;
    out << YAML::Key << "pose" << YAML::Value << YAML::BeginMap; // Start Pose
    out << YAML::Key << "position" << YAML::Value << YAML::BeginMap; // Start Position
    out << YAML::Key << "x" << YAML::Value << task.pose.position.x;
    out << YAML::Key << "y" << YAML::Value << task.pose.position.y;
    out << YAML::Key << "z" << YAML::Value << task.pose.position.z;
    out << YAML::EndMap; // End Position

    out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap; // Start Orientation
    out << YAML::Key << "x" << YAML::Value << task.pose.orientation.x;
    out << YAML::Key << "y" << YAML::Value << task.pose.orientation.y;
    out << YAML::Key << "z" << YAML::Value << task.pose.orientation.z;
    out << YAML::Key << "w" << YAML::Value << task.pose.orientation.w;
    out << YAML::EndMap; // End Orientation
    out << YAML::EndMap; // End of Pose
    out << YAML::EndMap;
  };

  out << YAML::EndSeq; // End Tasks
  out << YAML::EndMap; // End Routine
  out << YAML::EndMap; // End File

  std::ofstream fout("/workspaces/lerobot-pick-n-place/ros_ws/src/lerobot_routine/config/" + routine_name + ".yaml");
  fout << out.c_str();
  fout.close();
}
