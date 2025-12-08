#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <termios.h>
#include <unistd.h>

#include "lerobot_routine/routine.hpp"

// #include "lerobot_routine/routine.hpp"

/*
press a key (C) to save current position as a Task struct

for arm group:
get TF position & orientation value

for gripper group:
get position from joint value

need to implement vec2yaml conversion
*/


class RoutineBuilder : public rclcpp::Node {
 public:
  RoutineBuilder() : Node("teach_routine_node") {
    // TF2 buffer and listener for arm pose
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    routine_vec_ = std::vector<Task>();
    
    RCLCPP_INFO(this->get_logger(), "Teach routine node started. Waiting for transforms...");
    rclcpp::sleep_for(std::chrono::seconds(2));
  }
  
  void run() {
    
  }

 private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<Task> routine_vec_;

  void get_eef_pose()
  {

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tf_buffer_->lookupTransform("gripper", "base_link", tf2::TimePointZero);

    
    Task task;
    task.target = "arm";
    task.pose = [](auto &transformStamped)
    {
      geometry_msgs::msg::Pose msg;
      msg.position.x = transformStamped.transform.translation.x;
      msg.position.y = transformStamped.transform.translation.y;
      msg.position.z = transformStamped.transform.translation.z;
      msg.orientation.x = transformStamped.transform.rotation.x;
      msg.orientation.y = transformStamped.transform.rotation.y;
      msg.orientation.z = transformStamped.transform.rotation.z;
      msg.orientation.w = transformStamped.transform.rotation.w;
      return msg;
    }(transformStamped);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto routine_builder = std::make_shared<RoutineBuilder>();
  routine_builder->run();
  rclcpp::shutdown();
  return 0;
}
