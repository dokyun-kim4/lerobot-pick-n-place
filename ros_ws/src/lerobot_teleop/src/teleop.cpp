#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <termios.h>
#include <unistd.h>

using std::placeholders::_1;

// Constants for arm and gripper jog steps and the time they should take.
const float ARM_STEP = 0.02;
const float GRIPPER_STEP = 0.05;
const float TIME = 0.1;

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop() : Node("keyboard_teleop")
    {
        // Create a reentrant callback group for the I/O loop. This way it can be blocking and subscriber 
        // callbacks will still fire because of the multithread executor.
        cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions options;
        options.callback_group = cb_group_;

        // Subscribe to joint states 
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&KeyboardTeleop::setJointState, this, _1), options);

        // Create publishers for the arm and gripper contorller topics
        arm_controller_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
        gripper_controller_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_controller/commands", 10);
        RCLCPP_INFO(this->get_logger(), "Entering Keyboard Teleop Mode");

        // Set up a timer to launch the I/O loop
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&KeyboardTeleop::getInput, this), cb_group_);

        // Helper variable for the arm controller joint names
        JOINT_NAMES = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"};
    }

private:
    /**
     * Update the internally stored joint states whenever new ones come in from the publisher
     */
    void setJointState(const sensor_msgs::msg::JointState::SharedPtr state)
    {
        this->joint_states_ = *state;
    }

    /**
     * The main I/O loop. Blocking and needs to be run in a separate thread
     */
    void getInput()
    {
        // Destroy the time so this only runs once
        this->timer_->cancel();

        // Set up terminal interaction with termios
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio); // Get current terminal settings
        new_tio = old_tio;
        new_tio.c_lflag &= (~ICANON & ~ECHO);       // Disable canonical mode and echoing
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio); // Apply new settings

        bool running = true; // Whether to continue looping
        char c; // The pressed key

        // Print the available commands
        std::cout << "Commands:" << std::endl
                  << "Q/A: Shoulder Pan +/-" << std::endl
                  << "W/S: Shoulder Lift +/-" << std::endl
                  << "E/D: Elbow Flex +/-" << std::endl
                  << "R/F: Wrist Flex +/-" << std::endl
                  << "T/G: Wrist Roll +/-" << std::endl
                  << "Y/H: Gripper +/-" << std::endl
                  << "P: Print the current joint states" << std::endl
                  << "L: Quit" << std::endl;

        while (running)
        {
            if (read(STDIN_FILENO, &c, 1) > 0)
            { // Read one character
                std::cout << "Key pressed: " << (char)toupper(c) << std::endl;

                std::vector<double> arm_angles;
                double gripper_angle;

                // Set the joint angles to their current angles
                arm_angles = this->joint_states_.position;
                arm_angles.pop_back(); // Remove the gripper joint angle

                // Set the gripper angle to it's current angle
                gripper_angle = this->joint_states_.position.back();

                switch (c)
                {
                case 'l': // Stop running
                    running = false;
                    break;
                case 'p': // Print the current joint states
                    std::cout << joint_states_.position[0] << std::endl;
                    break;
                case 'q':
                    arm_angles[0] += ARM_STEP;
                    break;
                case 'a':
                    arm_angles[0] -= ARM_STEP;
                    break;
                case 'w':
                    arm_angles[1] += ARM_STEP;
                    break;
                case 's':
                    arm_angles[1] -= ARM_STEP;
                    break;
                case 'e':
                    arm_angles[2] += ARM_STEP;
                    break;
                case 'd':
                    arm_angles[2] -= ARM_STEP;
                    break;
                case 'r':
                    arm_angles[3] += ARM_STEP;
                    break;
                case 'f':
                    arm_angles[3] -= ARM_STEP;
                    break;
                case 't':
                    arm_angles[4] += ARM_STEP;
                    break;
                case 'g':
                    arm_angles[4] -= ARM_STEP;
                    break;
                case 'y':
                    gripper_angle -= GRIPPER_STEP;
                    break;
                case 'h':
                    gripper_angle += GRIPPER_STEP;
                    break;
                }

                // If ros2 wants to close (for example on Ctrl+C) or if the user triggered shutdown, break before sending commands
                if (!rclcpp::ok() || !running)
                {
                    break;
                }

                // Set up a joint trajectory to send to the arm controller
                trajectory_msgs::msg::JointTrajectory joint_trajectory;
                joint_trajectory.header.stamp = this->get_clock()->now();
                joint_trajectory.header.frame_id = "base_link";
                joint_trajectory.joint_names = JOINT_NAMES;

                // Create the goal point for the joint trajectory and add it
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = arm_angles;
                point.time_from_start = rclcpp::Duration(0, int(TIME * 1e9));
                joint_trajectory.points.push_back(point);

                // Command the arm
                arm_controller_->publish(joint_trajectory);

                // Set up a gripper command
                std_msgs::msg::Float64MultiArray gripper_command;
                gripper_command.data.push_back(gripper_angle);

                // Command the gripper
                gripper_controller_->publish(gripper_command);
            }
        }

        // Clean up the terminal changes and shut down automatically if Ctrl+C was not pressed.
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio); // Restore original settings
        rclcpp::shutdown();
    }

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_controller_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_controller_;
    sensor_msgs::msg::JointState joint_states_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> JOINT_NAMES;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // Set up the multi-thread executor so the blocking I/O loop can run
    rclcpp::executors::MultiThreadedExecutor executor;

    // Run the main node
    auto keyboard_teleop_node = std::make_shared<KeyboardTeleop>();
    executor.add_node(keyboard_teleop_node);
    executor.spin();

    // Cleanup
    rclcpp::shutdown();
    return 0;
}
