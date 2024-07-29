#ifndef CLIENT_H
#define CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_interfaces/action/full_drive.hpp"
#include <vector>
#include <chrono>

class ClientActions : public rclcpp::Node {
public:
    using FullDrive = action_interfaces::action::FullDrive;
    using GoalHandleFullDrive = rclcpp_action::ClientGoalHandle<FullDrive>;

    explicit ClientActions(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("client_actions", options) {
        this->client_ = rclcpp_action::create_client<FullDrive>(this, "full_drive");
    }

    bool addCollisionObject(const std::string &target_name, const shape_msgs::msg::SolidPrimitive &primitive,
                            const geometry_msgs::msg::Pose &pose, const std_msgs::msg::ColorRGBA &color) {
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = FullDrive::Goal();
        goal_msg.target_name = target_name;
        goal_msg.object_primitive = primitive;
        goal_msg.object_pose = pose;
        goal_msg.color = color;

        goal_msg.add_collision_object = true;
        goal_msg.delete_collision_object = false;
        goal_msg.attach_object = false;
        goal_msg.detach_object = false;
        goal_msg.move_to = false;
        goal_msg.move_linear = false;
        goal_msg.allow_collision = false;
        goal_msg.reenable_collision = false;

        return sendGoal(goal_msg);
    }
    
    bool deleteObject(const std::string &target_name) {
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = FullDrive::Goal();
        goal_msg.target_name = target_name;

        goal_msg.add_collision_object = false;
        goal_msg.delete_collision_object = true;
        goal_msg.attach_object = false;
        goal_msg.detach_object = false;
        goal_msg.move_to = false;
        goal_msg.move_linear = false;
        goal_msg.allow_collision = false;
        goal_msg.reenable_collision = false;
        
        return sendGoal(goal_msg);
    }

    bool attachObject(const std::string &target_name) {
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = FullDrive::Goal();
        goal_msg.target_name = target_name;
        goal_msg.link = "flange";

        goal_msg.add_collision_object = false;
        goal_msg.delete_collision_object = false;
        goal_msg.attach_object = true;
        goal_msg.detach_object = false;
        goal_msg.move_to = false;
        goal_msg.move_linear = false;
        goal_msg.allow_collision = false;
        goal_msg.reenable_collision = false;
        
        return sendGoal(goal_msg);
    }

    bool detachObject(const std::string &target_name) {
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = FullDrive::Goal();
        goal_msg.target_name = target_name;
        goal_msg.link = "flange";

        goal_msg.add_collision_object = false;
        goal_msg.delete_collision_object = false;
        goal_msg.attach_object = false;
        goal_msg.detach_object = true;
        goal_msg.move_to = false;
        goal_msg.move_linear = false;
        goal_msg.allow_collision = false;
        goal_msg.reenable_collision = false;
        
        return sendGoal(goal_msg);
    }

    bool moveTo(const geometry_msgs::msg::PoseStamped &target_pose, bool constrain, bool motion_speed) {
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = FullDrive::Goal();
        goal_msg.target_pose = target_pose;
        goal_msg.constrain = constrain;
        goal_msg.motion_speed = motion_speed;
        
        goal_msg.add_collision_object = false;
        goal_msg.delete_collision_object = false;
        goal_msg.attach_object = false;
        goal_msg.detach_object = false;
        goal_msg.move_to = true;
        goal_msg.move_linear = false;
        goal_msg.allow_collision = false;
        goal_msg.reenable_collision = false;
        
        return sendGoal(goal_msg);
    }

    bool moveLinear(const geometry_msgs::msg::PoseStamped &target_pose) {
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = FullDrive::Goal();
        goal_msg.target_pose = target_pose;
        
        goal_msg.add_collision_object = false;
        goal_msg.delete_collision_object = false;
        goal_msg.attach_object = false;
        goal_msg.detach_object = false;
        goal_msg.move_to = false;
        goal_msg.move_linear = true;
        goal_msg.allow_collision = false;
        goal_msg.reenable_collision = false;

        return sendGoal(goal_msg);
    }

    bool allowCollision(const std::string &object1, const std::string &object2) {
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = FullDrive::Goal();
        goal_msg.target_name = object1;
        goal_msg.object_name = object2;
        
        goal_msg.add_collision_object = false;
        goal_msg.delete_collision_object = false;
        goal_msg.attach_object = false;
        goal_msg.detach_object = false;
        goal_msg.move_to = false;
        goal_msg.move_linear = false;
        goal_msg.allow_collision = true;
        goal_msg.reenable_collision = false;

        return sendGoal(goal_msg);
    }

    bool reenableCollision(const std::string &object1, const std::string &object2) {
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = FullDrive::Goal();
        goal_msg.target_name = object1;
        goal_msg.object_name = object2;
        
        goal_msg.add_collision_object = false;
        goal_msg.delete_collision_object = false;
        goal_msg.attach_object = false;
        goal_msg.detach_object = false;
        goal_msg.move_to = false;
        goal_msg.move_linear = false;
        goal_msg.allow_collision = false;
        goal_msg.reenable_collision = true;

        return sendGoal(goal_msg);
    }

private:
    rclcpp_action::Client<FullDrive>::SharedPtr client_;

    bool sendGoal(FullDrive::Goal &goal_msg) {
        auto send_goal_options = rclcpp_action::Client<FullDrive>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](GoalHandleFullDrive::SharedPtr goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };
        send_goal_options.result_callback = [this](const GoalHandleFullDrive::WrappedResult &wrapped_result) {
            switch (wrapped_result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    break;
            }
        };

        // Send the goal asynchronously
        auto goal_handle_future = this->client_->async_send_goal(goal_msg, send_goal_options);

        // Block until the goal handle is ready
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
            return false;
        }

        // Get the result asynchronously
        auto result_future = this->client_->async_get_result(goal_handle_future.get());

        // Block until the result is ready
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Get result call failed");
            return false;
        }

        // Check the result code
        return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
    }
};

#endif // CLIENT_H
    
    
    /* bool openGripper() {
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = FullDrive::Goal();
        goal_msg.open_gripper = true;

        return sendGoal(goal_msg);
    }

    bool closeGripper() {
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = FullDrive::Goal();
        goal_msg.close_gripper = true;

        return sendGoal(goal_msg);
    } */