#include "RobotiqCModelURCap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "service_interfaces/srv/gripper_control.hpp"

class GripperControlNode : public rclcpp::Node
{
public:
    GripperControlNode()
        : Node("gripper_control_node"), gripper_("10.130.1.100")
    {
        gripper_.activate(true);

        service_ = this->create_service<service_interfaces::srv::GripperControl>(
            "gripper_control",
            std::bind(&GripperControlNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));
        
        gripper_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&GripperControlNode::publish_gripper_state, this));

        RCLCPP_INFO(this->get_logger(), "Gripper Control Service Initialized");
    }

private:
    RobotiqCModelURCap gripper_;
    rclcpp::Service<service_interfaces::srv::GripperControl>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void handle_service(
        const std::shared_ptr<service_interfaces::srv::GripperControl::Request> request,
        std::shared_ptr<service_interfaces::srv::GripperControl::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received command: %s", request->command.c_str());

        bool success = true;
        std::string message = "Success";

        if (request->command == "activate")
        {
            gripper_.activate();
        }
        else if (request->command == "auto_calibrate_gripper")
        {
            gripper_.auto_calibrate_gripper();
        }
        else if (request->command == "open")
        {
            gripper_.move(0);
        }
        else if (request->command == "close")
        {
            gripper_.move(255);
        }
        else if (request->command == "set")
        {
            gripper_.move(request->position);
        }
        else if (request->command == "move_and_wait_for_pos")
        {
            gripper_.move_and_wait_for_pos(request->position);
        }
        else
        {
            success = false;
            message = "Unknown command: " + request->command;
            RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
        }

        response->success = success;
        response->message = message;

        RCLCPP_INFO(this->get_logger(), "Service response: success=%d, message=%s", success, message.c_str());
    }

    void publish_gripper_state()
    {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->now();

        // Assuming the gripper has one joint named "gripper_finger_joint"
        joint_state_msg.name.push_back("finger_joint");

        // Get the gripper position from the gripper class (assuming get_current_position is implemented)
        int gripper_position = gripper_.get_current_position();

        // Convert the position to a value between 0.0 (fully open) and 0.7 (fully closed)
        double gripper_joint_value = (gripper_position / 255.0) * 0.7;

        joint_state_msg.position.push_back(gripper_joint_value);

        // Publish the joint state
        gripper_state_publisher_->publish(joint_state_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
