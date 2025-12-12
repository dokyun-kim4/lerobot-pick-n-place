#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <termios.h>
#include <unistd.h>

#include "lerobot_routine/routine.hpp"

class RoutineBuilder : public rclcpp::Node {
 public:
  RoutineBuilder() : Node("teach_routine_node") {
    
    this->declare_parameter("routine_name", "new_routine");
    routine_name_ = this->get_parameter("routine_name").as_string();
    RCLCPP_INFO(this->get_logger(), "%s", routine_name_.c_str());

    // TF2 buffer and listener for arm pose
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options;
        options.callback_group = cb_group_;

    jaw_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        double jaw_joint_val = (msg->position)[5]; // get jaw joint value
        if (!jaw_position_set_)
        {
          jaw_position_prev_ = jaw_joint_val;
          jaw_position_set_ = true;
        } else {
          jaw_position_latest_ = jaw_joint_val; 
        }
      }, options);

    // Run main loop 
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RoutineBuilder::run, this));
    routine_vec_ = std::vector<Task>();
    
    RCLCPP_INFO(this->get_logger(), "Routine Teach Mode. Press `s` to save current position, `q` to export sequence to yaml.");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  
  void run() {
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio); // Get current terminal settings
    new_tio = old_tio;
    new_tio.c_lflag &= (~ICANON & ~ECHO);       // Disable canonical mode and echoing
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio); // Apply new settings

    bool running = true; 
    char c;

    while (running)
    {
        if (read(STDIN_FILENO, &c, 1) > 0)
        { // Read one character
            std::cout << "Key pressed: " << c << std::endl;

            switch (c)
            {
            case 'q':
                running = false;
                save_routine(routine_name_.c_str(), routine_vec_);
                break;        
            case 's':
                savePoseAsTask();
                break;
            }

            if (!rclcpp::ok() || !running) {
                break;
            }
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio); // Restore original settings
    rclcpp::shutdown();
  }

 private:

  std::string routine_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<Task> routine_vec_;
  bool jaw_position_set_ = false;
  double jaw_position_latest_;
  double jaw_position_prev_;
  double JAW_DELTA_TOLERANCE = 0.01;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jaw_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  void savePoseAsTask()
  { 
    // Handle change in jaw joint position
    if (abs(jaw_position_latest_ - jaw_position_prev_) > JAW_DELTA_TOLERANCE)
    {
      Task task = [](auto &jaw_position_latest_)
      {
        Task task;
        task.target = "gripper";
        task.gripper_position = jaw_position_latest_;
        return task;
      }(jaw_position_latest_);
      jaw_position_prev_ = jaw_position_latest_;

      RCLCPP_INFO(this->get_logger(), "Saved current jaw position as task: %.2f", jaw_position_latest_);
      routine_vec_.push_back(task);
    }

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tf_buffer_->lookupTransform("base_link", "gripper", tf2::TimePointZero);

    Task task = [](auto &transformStamped)
    {
      Task task;
      task.target = "arm";

      geometry_msgs::msg::Pose msg;
      msg.position.x = transformStamped.transform.translation.x;
      msg.position.y = transformStamped.transform.translation.y;
      msg.position.z = transformStamped.transform.translation.z;
      msg.orientation.x = transformStamped.transform.rotation.x;
      msg.orientation.y = transformStamped.transform.rotation.y;
      msg.orientation.z = transformStamped.transform.rotation.z;
      msg.orientation.w = transformStamped.transform.rotation.w;

      task.pose = msg;
      return task;
    }(transformStamped);

    routine_vec_.push_back(task);

    RCLCPP_INFO(this->get_logger(), "Saved current eef pose as task: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                task.pose.position.x, task.pose.position.y, task.pose.position.z, task.pose.orientation.x, task.pose.orientation.y, task.pose.orientation.z, task.pose.orientation.w);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto routine_builder = std::make_shared<RoutineBuilder>();
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(routine_builder);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
