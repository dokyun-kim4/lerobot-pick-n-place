#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <termios.h>
#include <unistd.h>

using std::placeholders::_1;

const float STEP = 0.01;
const float TIME = 20000000;

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop() : Node("keyboard_teleop")
    {
        cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions options;
        options.callback_group = cb_group_;
        
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&KeyboardTeleop::setJointState, this, _1), options);
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
        RCLCPP_INFO(this->get_logger(), "Entering Keyboard Teleop Mode");

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&KeyboardTeleop::getInput, this), cb_group_);

        JOINT_NAMES = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"};
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

                std::vector<double> joint_angles;

                joint_angles = this->joint_states_.position;
                joint_angles.pop_back(); // Remove the gripper joint angle

                switch (c)
                {
                case 'l':
                    running = false;
                    break;        
                case 'p':
                    std::cout << joint_states_.position[0] << std::endl;
                    break;
                case 'q':
                    joint_angles[0] += STEP;
                    break;
                case 'a':
                    joint_angles[0] -= STEP;
                    break;
                case 'w':
                    joint_angles[1] += STEP;
                    break;
                case 's':
                    joint_angles[1] -= STEP;
                    break;
                case 'e':
                    joint_angles[2] += STEP;
                    break;
                case 'd':
                    joint_angles[2] -= STEP;
                    break;
                case 'r':
                    joint_angles[3] += STEP;
                    break;
                case 'f':
                    joint_angles[3] -= STEP;
                    break;
                case 't':
                    joint_angles[4] += STEP;
                    break;
                case 'g':
                    joint_angles[4] -= STEP;
                    break;
                }

                if (!rclcpp::ok() || !running) {
                    break;
                }

                trajectory_msgs::msg::JointTrajectory joint_trajectory;
                joint_trajectory.header.stamp = this->get_clock()->now();
                joint_trajectory.header.frame_id = "base_link";
                joint_trajectory.joint_names = JOINT_NAMES;

                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = joint_angles;
                point.time_from_start = rclcpp::Duration(0, TIME);
                joint_trajectory.points.push_back(point);

                publisher_->publish(joint_trajectory);
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio); // Restore original settings
        rclcpp::shutdown();
    }

    rclcpp::CallbackGroup::SharedPtr cb_group_;

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
