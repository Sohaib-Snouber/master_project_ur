#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class GripperController : public rclcpp::Node
{
public:
    GripperController()
        : Node("gripper_controller"), 
          open_position_(0.0), 
          close_position_(0.7), 
          is_opening_(true),
          elapsed_time_(0.0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&GripperController::update_joint_state, this));
        joint_state_msg_.name = {"finger_joint"};
        joint_state_msg_.position = {open_position_};
    }

private:
    void update_joint_state()
    {
        if (elapsed_time_ < 5.0) {
            if (is_opening_)
            {
                joint_state_msg_.position[0] -= 0.01;
                if (joint_state_msg_.position[0] <= close_position_)
                {
                    is_opening_ = false;
                }
            }
            else
            {
                joint_state_msg_.position[0] += 0.01;
                if (joint_state_msg_.position[0] >= open_position_)
                {
                    is_opening_ = true;
                }
            }

            elapsed_time_ += 0.1; // Increment elapsed time by 100ms
        }

        joint_state_msg_.header.stamp = this->get_clock()->now();
        publisher_->publish(joint_state_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_msg_;
    double open_position_;
    double close_position_;
    bool is_opening_;
    double elapsed_time_;  // To track the elapsed time
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperController>());
    rclcpp::shutdown();
    return 0;
}
