#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <termios.h>
#include <unistd.h>

using std::placeholders::_1;

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop() : Node("keyboard_teleop")
    {
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&KeyboardTeleop::setJointState, this, _1));
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);
        RCLCPP_INFO(this->get_logger(), "Entering Keyboard Teleop Mode");

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&KeyboardTeleop::getInput, this));

        JOINT_NAMES = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper_joint"};
    }

private:
    void setJointState(const sensor_msgs::msg::JointState::SharedPtr state)
    {
        this->joint_states_ = *state;
    }

    void getInput() 
    {
        this->timer_->cancel();

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
                case 'l': // Exit on 'l'
                    running = false; 
                    break;               
                case 'p':
                    std::cout << joint_states_.position[0] << std::endl;
                    break;
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio); // Restore original settings
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    sensor_msgs::msg::JointState joint_states_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> JOINT_NAMES;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto keyboard_teleop_node = std::make_shared<KeyboardTeleop>();

    executor.add_node(keyboard_teleop_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

/*
Q & A -> shoulder pan
W & S -> shoulder_lift
E & D -> elbow_flex
R & F -> wrist_flex
T & G -> wrist_roll
Y & H -> gripper_joint

user would press a key and robot would immediately move
*/
