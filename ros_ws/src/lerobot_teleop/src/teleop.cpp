#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <termios.h>
#include <unistd.h>

class KeyboardTeleop : public rclcpp::Node
{
    public:
        KeyboardTeleop() : Node("keyboard_teleop")
        {
            publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);
            RCLCPP_INFO(this->get_logger(), "Entering Keyboard Teleop Mode");

            JOINT_NAMES = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper_joint"};
        
        }

    
    private:
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
        std::vector<std::string> JOINT_NAMES;
};


/*
Q & A -> shoulder pan
W & S -> shoulder_lift
E & D -> elbow_flex
R & F -> wrist_flex
T & G -> wrist_roll
Y & H -> gripper_joint

user would press a key and robot would immediately move
*/
