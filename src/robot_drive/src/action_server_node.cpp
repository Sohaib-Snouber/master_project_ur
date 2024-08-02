#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_interfaces/action/full_drive.hpp"
#include "robot_drive/update_planning_scene.h"
#include <thread>
#include <chrono>
#include <random>
#include <unordered_map>
#include <string>
#include <vector>
#include <algorithm>
#include <mutex>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/robot_trajectory/robot_trajectory.h>

using FullDrive = action_interfaces::action::FullDrive;
using GoalHandleFullDrive = rclcpp_action::ServerGoalHandle<FullDrive>;

// Structure to hold object properties
struct ObjectProperties {
    bool exists;
    bool collision_allowed;
    std::vector<std::string> allowed_collision_objects;
};

class FullDriveActionServer : public rclcpp::Node {
public:
    FullDriveActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("robot_drive_action_server", options),
        planning_scene_interface_(std::make_shared<moveit::planning_interface::PlanningSceneInterface>()){
        this->action_server_ = rclcpp_action::create_server<FullDrive>(
            this,
            "robot_drive",
            std::bind(&FullDriveActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FullDriveActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FullDriveActionServer::handle_accepted, this, std::placeholders::_1));


        // Defer the initialization of shared_from_this()
        auto timer_callback = [this]() {
            planning_scene_updater_ = std::make_shared<PlanningSceneUpdater>(this->shared_from_this());
            RCLCPP_INFO(this->get_logger(), "Initialized the PlanninSceneUpdater class");
            // Initialize MoveGroupInterface instances
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_arm");
            
            robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(), "robot_description");
            moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();
            planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model);

            // Initialize planning scene
            setupPlanningScene();
            loadExistingCollisionObjects();

            initialization_timer_->cancel(); // Cancel the timer after initialization
        };
        // Use a timer to defer the callback
        initialization_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), timer_callback);
    }

private:
    rclcpp_action::Server<FullDrive>::SharedPtr action_server_;

    // Map to store objects and their properties
    std::unordered_map<std::string, ObjectProperties> object_map_;
    rclcpp::TimerBase::SharedPtr initialization_timer_;  // Store the timer to prevent it from being destroyed
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
    std::shared_ptr<PlanningSceneUpdater> planning_scene_updater_;

    void setupPlanningScene(){
        // Create collision objects
        shape_msgs::msg::SolidPrimitive primitive_wall1;
        primitive_wall1.type = primitive_wall1.BOX;
        primitive_wall1.dimensions = {0.1, 1.4, 0.5};
        geometry_msgs::msg::Pose wall1_pose;
        wall1_pose.orientation.w = 1.0;
        wall1_pose.position.x = 0.8;
        wall1_pose.position.y = 0.2;
        wall1_pose.position.z = 0.25;
        moveit_msgs::msg::CollisionObject collision_object_wall1 = createCollisionObject("wall1", primitive_wall1, wall1_pose);
        
        shape_msgs::msg::SolidPrimitive primitive_wall2;
        primitive_wall2.type = primitive_wall2.BOX;
        primitive_wall2.dimensions = {1.4, 0.1, 0.5};
        geometry_msgs::msg::Pose wall2_pose;
        wall2_pose.orientation.w = 1.0;
        wall2_pose.position.x = 0.2;
        wall2_pose.position.y = 0.8;
        wall2_pose.position.z = 0.25;
        moveit_msgs::msg::CollisionObject collision_object_wall2 = createCollisionObject("wall2", primitive_wall2, wall2_pose);

        shape_msgs::msg::SolidPrimitive primitive_wall3;
        primitive_wall3.type = primitive_wall3.BOX;
        primitive_wall3.dimensions = {0.1, 1.4, 0.5};
        geometry_msgs::msg::Pose wall3_pose;
        wall3_pose.orientation.w = 1.0;
        wall3_pose.position.x = -0.6;
        wall3_pose.position.y = 0.2;
        wall3_pose.position.z = 0.25;
        moveit_msgs::msg::CollisionObject collision_object_wall3 = createCollisionObject("wall3", primitive_wall3, wall3_pose);
        
        shape_msgs::msg::SolidPrimitive primitive_wall4;
        primitive_wall4.type = primitive_wall4.BOX;
        primitive_wall4.dimensions = {1.4, 0.1, 0.5};
        geometry_msgs::msg::Pose wall4_pose;
        wall4_pose.orientation.w = 1.0;
        wall4_pose.position.x = 0.2;
        wall4_pose.position.y = -0.6;
        wall4_pose.position.z = 0.25;
        moveit_msgs::msg::CollisionObject collision_object_wall4 = createCollisionObject("wall4", primitive_wall4, wall4_pose);

        shape_msgs::msg::SolidPrimitive primitive_surface;
        primitive_surface.type = primitive_surface.BOX;
        primitive_surface.dimensions = {0.75, 0.8, 0.1};
        geometry_msgs::msg::Pose surface_pose;
        surface_pose.orientation.w = 1.0;
        surface_pose.position.x = 0.16;
        surface_pose.position.y = 0.25;
        surface_pose.position.z = -0.05;
        moveit_msgs::msg::CollisionObject collision_object_surface = createCollisionObject("surface", primitive_surface, surface_pose);

        // Create object colors
        moveit_msgs::msg::ObjectColor surface_color = createObjectColor("surface", 0.5, 0.5, 0.5, 1.0);
        moveit_msgs::msg::ObjectColor wall1_color = createObjectColor("wall1", 0.1, 0.2, 0.3, 1.0); // -y axis
        moveit_msgs::msg::ObjectColor wall2_color = createObjectColor("wall2", 1.0, 1.0, 1.0, 1.0); // +x axis
        moveit_msgs::msg::ObjectColor wall3_color = createObjectColor("wall3", 1.0, 1.0, 1.0, 1.0); // +y axis
        moveit_msgs::msg::ObjectColor wall4_color = createObjectColor("wall4", 1.0, 1.0, 1.0, 1.0); //- x axis

        // Create a PlanningScene message and add the collision objects and their colors
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.collision_objects = {collision_object_surface, collision_object_wall1,collision_object_wall2,collision_object_wall3,collision_object_wall4};
        planning_scene_msg.object_colors = {surface_color, wall1_color, wall2_color, wall3_color, wall4_color};

        // Apply the planning scene
        planning_scene_interface_->applyPlanningScene(planning_scene_msg);

        // Modify the ACM to allow collision between base_link_inertia and surface
        collision_detection::AllowedCollisionMatrix& acm = planning_scene_->getAllowedCollisionMatrixNonConst();
        acm.setEntry("base_link_inertia", "surface", true);

        // Update the planning scene with the modified ACM
        moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
        acm.getMessage(acm_msg);
        planning_scene_msg.allowed_collision_matrix = acm_msg;
        planning_scene_interface_->applyPlanningScene(planning_scene_msg);

        RCLCPP_INFO(this->get_logger(), "Allowed collision between base_link_inertia and surface.");

        RCLCPP_INFO(this->get_logger(), "Planning scene setup complete");
    }

    moveit_msgs::msg::CollisionObject createCollisionObject(
        const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive, const geometry_msgs::msg::Pose& pose)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "world";
        collision_object.id = id;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;
        return collision_object;
    }

    moveit_msgs::msg::ObjectColor createObjectColor(
        const std::string& id, float r, float g, float b, float a)
    {
        moveit_msgs::msg::ObjectColor color;
        color.id = id;
        color.color.r = r;
        color.color.g = g;
        color.color.b = b;
        color.color.a = a;
        return color;
    }

    /* // example of Adding an object to the map
    object_map_["object1"] = {true, false, {"object2", "object3"}}; */

    void loadExistingCollisionObjects() {
        auto collision_objects = planning_scene_interface_->getObjects();

        for (const auto& object_pair : collision_objects) {
            const std::string& object_id = object_pair.first;
            RCLCPP_INFO(this->get_logger(), "Found existing collision object: %s", object_id.c_str());
            object_map_[object_id] = {true, false, {}};
        }
        // Load the robot's link names and add them to the object map
        const std::vector<std::string>& link_names = move_group_->getRobotModel()->getLinkModelNames();

        for (const auto& link_name : link_names) {
            RCLCPP_INFO(this->get_logger(), "Found robot link: %s", link_name.c_str());
            object_map_[link_name] = {true, false, {}};
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu existing collision objects into object_map_", object_map_.size());
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FullDrive::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        // Print the goal details
        std::cout << "Goal Details:" << std::endl;
        std::cout << "add_collision_object: " << goal->add_collision_object << std::endl;
        std::cout << "delete_collision_object: " << goal->delete_collision_object << std::endl;
        std::cout << "attach_object: " << goal->attach_object << std::endl;
        std::cout << "detach_object: " << goal->detach_object << std::endl;
        std::cout << "move_to: " << goal->move_to << std::endl;
        std::cout << "move_linear: " << goal->move_linear << std::endl;
        std::cout << "allow_collision: " << goal->allow_collision << std::endl;
        std::cout << "reenable_collision: " << goal->reenable_collision << std::endl;
        std::cout << "object_name: " << goal->object_name << std::endl;
        std::cout << "target_name: " << goal->target_name << std::endl;
        std::cout << "task: " << goal->task << std::endl;
        std::cout << "id: " << goal->id << std::endl;
        std::cout << "link: " << goal->link << std::endl;
        std::cout << "constrain: " << goal->constrain << std::endl;
        std::cout << "motion_speed: " << goal->motion_speed << std::endl;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFullDrive> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFullDrive> goal_handle) {
        std::thread{std::bind(&FullDriveActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    bool execute(const std::shared_ptr<GoalHandleFullDrive> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<FullDrive::Result>();
        bool success = false;
        
        if (goal->add_collision_object) {
            if (object_map_.find(goal->target_name) != object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Collision object already exists: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Collision object already exists.";
                goal_handle->abort(result);
                return true;
            }
            RCLCPP_INFO(this->get_logger(), "Adding collision object");
            planning_scene_updater_->addCollisionObject(goal->target_name, goal->object_primitive, goal->object_pose, goal->color);
            object_map_[goal->target_name] = {true, false, {}};
            success = true;

        } else if (goal->delete_collision_object) {
            if (object_map_.find(goal->target_name) == object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Collision object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Collision object does not exist.";
                goal_handle->abort(result);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Deleting collision object");
            planning_scene_updater_->removeCollisionObject(goal->target_name);
            object_map_.erase(goal->target_name);
            success = true;

        } else if (goal->attach_object) {
            if (object_map_.find(goal->target_name) == object_map_.end() || !object_map_[goal->target_name].exists) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Attaching object");
            success = attachObject(goal->target_name, goal->link);
            result->success = success;

        } else if (goal->detach_object) {
            if (object_map_.find(goal->target_name) == object_map_.end() || !object_map_[goal->target_name].exists) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Detaching object");
            success = detachObject(goal->target_name, goal->link);
            result->success = success;

        } else if (goal->move_to) {
            RCLCPP_INFO(this->get_logger(), "Moving to pose");
            success = moveToPose(goal->target_pose, goal->constrain, goal->motion_speed);
            result->success = success;

        } else if (goal->move_linear) {
            RCLCPP_INFO(this->get_logger(), "Moving linearly to pose");
            success = moveLinear(goal->target_pose);
            result->success = success;

        } else if (goal->allow_collision) {
            if (object_map_.find(goal->target_name) == object_map_.end() || object_map_.find(goal->object_name) == object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Allowing collision");
            success = allowCollision(goal->target_name, goal->object_name, true);
            result->success = success;

        } else if (goal->reenable_collision) {
            if (object_map_.find(goal->target_name) == object_map_.end() || object_map_.find(goal->object_name) == object_map_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Object does not exist: %s", goal->target_name.c_str());
                result->success = false;
                result->message = "Object does not exist.";
                goal_handle->abort(result);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Re-enabling collision");
            success = allowCollision(goal->target_name, goal->object_name, false);
            result->success = success;
        }
        
        // Set the result and return
        if (success) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
            goal_handle->abort(result);
        }

        return success;
    }

    bool allowCollision(const std::string& object1, const std::string& object2, bool allow) {
        collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
        acm.setEntry(object1, object2, allow);

        moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
        acm.getMessage(acm_msg);

        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.allowed_collision_matrix = acm_msg;

        planning_scene_interface_->applyPlanningScene(planning_scene_msg);
        RCLCPP_INFO(this->get_logger(), "Allowed collision between %s and %s: %d", object1.c_str(), object2.c_str(), allow);
        return true;
    }

    bool attachObject(const std::string& object_id, const std::string& link) {
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = link;
        attached_object.object.id = object_id;
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        planning_scene_interface_->applyAttachedCollisionObject(attached_object);
        RCLCPP_INFO(this->get_logger(), "Attached object %s to %s", object_id.c_str(), link.c_str());
        return true;
    }

    bool detachObject(const std::string& object_id, const std::string& link) {
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = link;
        attached_object.object.id = object_id;
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        planning_scene_interface_->applyAttachedCollisionObject(attached_object);
        RCLCPP_INFO(this->get_logger(), "Detached object %s from %s", object_id.c_str(), link.c_str());
        return true;
    }

    bool moveToPose(const geometry_msgs::msg::PoseStamped& target_pose, bool constrain = false, double motion_speed = 0.1) 
    {
        moveit_msgs::msg::Constraints path_constraints;
        if (constrain) {
            moveit_msgs::msg::JointConstraint shoulder_constraint;
            shoulder_constraint.joint_name = "shoulder_lift_joint";
            shoulder_constraint.position = -M_PI_2;
            shoulder_constraint.tolerance_above = 0.5;
            shoulder_constraint.tolerance_below = 0.5;
            shoulder_constraint.weight = 1.0;
            path_constraints.joint_constraints.push_back(shoulder_constraint);

            move_group_->setPathConstraints(path_constraints);
        }

        if(motion_speed > 1.0){
            motion_speed = 1.0;
        } else if (motion_speed < 0.0)
        {
            motion_speed = 0.1;
        }
        
        move_group_->setMaxVelocityScalingFactor(motion_speed);

        move_group_->setPoseTarget(target_pose);
        // Specify the planner ID to use Pilz PTP
        move_group_->setPlannerId("PTP");

        // Plan and execute the motion
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success){
            auto result = move_group_->execute(my_plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_INFO(this->get_logger(), "Motion executed successfully to target pose.");
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Motion execution failed to target pose.");
                success = false;
            }
        } else{
            RCLCPP_ERROR(this->get_logger(), "Motion plan failed to target pose.");
        }
        // Clear path constraints if they were set
        if (constrain) {
            move_group_->clearPathConstraints();
        }

        return success;
    }

    bool moveLinear(const geometry_msgs::msg::PoseStamped& target_pose){
        // Define waypoints
        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Get current pose
        geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;

        // Add intermediate waypoints for a linear path
        int num_intermediate_points = 10; // Number of intermediate waypoints for a finer resolution
        for (int i = 0; i <= num_intermediate_points; ++i) {
            geometry_msgs::msg::Pose waypoint;
            waypoint.position.x = current_pose.position.x + (target_pose.pose.position.x - current_pose.position.x) * i / num_intermediate_points;
            waypoint.position.y = current_pose.position.y + (target_pose.pose.position.y - current_pose.position.y) * i / num_intermediate_points;
            waypoint.position.z = current_pose.position.z + (target_pose.pose.position.z - current_pose.position.z) * i / num_intermediate_points;
            waypoint.orientation = target_pose.pose.orientation; // Assuming the orientation remains constant
            waypoints.push_back(waypoint);
        }

        // Plan the Cartesian path
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        bool success = (fraction > 0.95);

        if (success){
            // Create a plan from the computed trajectory
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            my_plan.trajectory = trajectory;

            // Specify the planner ID to use Pilz PTP
            move_group_->setPlannerId("PTP");
            
            // Execute the plan
            auto result = move_group_->execute(my_plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_INFO(this->get_logger(), "Cartesian path executed successfully to target pose.");
            }else{
                RCLCPP_ERROR(this->get_logger(), "Cartesian path execution failed to target pose.");
                success = false;
            }
        }else{
            RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed to target pose with fraction %.2f.", fraction);
        }

        return success;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FullDriveActionServer>();
    
    // Create a multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4); // 4 threads
    
    executor.add_node(node);
    executor.spin();    
    rclcpp::shutdown();
    return 0;
}
