#ifndef UPDATE_PLANNING_SCENE_H
#define UPDATE_PLANNING_SCENE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <string>
#include <vector>
#include <moveit_msgs/msg/object_color.hpp>
#include <algorithm>
#include <exception>

class PlanningSceneUpdater {
public:
    PlanningSceneUpdater(const rclcpp::Node::SharedPtr& node)
        : node_(node), planning_scene_interface_() {
        RCLCPP_INFO(node_->get_logger(), "Initialized the PlanningSceneUpdater class");
    }

    bool addCollisionObject(const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive, const geometry_msgs::msg::Pose& pose, const std_msgs::msg::ColorRGBA& color) {
        bool success = false;
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = id;
        collision_object.header.frame_id = "world";
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        moveit_msgs::msg::ObjectColor object_color;
        try {
            object_color = addColor(id, color);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed adding color: %s", e.what());
            return success;
        }
        
        // Create a PlanningScene message and add the collision object and its color
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        planning_scene_msg.object_colors.push_back(object_color);

        // Apply the planning scene
        try {
            planning_scene_interface_.applyPlanningScene(planning_scene_msg);
            RCLCPP_INFO(node_->get_logger(), "Added collision object: %s", id.c_str());
            success = true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to apply planning scene: %s", e.what());
        }

        return success;
    }

    bool removeCollisionObject(const std::string& id) {
        bool success = false;
        // Check if the object exists in the planning scene
        auto existing_objects = planning_scene_interface_.getKnownObjectNames();
        if (std::find(existing_objects.begin(), existing_objects.end(), id) == existing_objects.end()) {
            RCLCPP_WARN(node_->get_logger(), "Object ID %s does not exist in the planning scene.", id.c_str());
            return success;
        }

        std::vector<std::string> object_ids;
        object_ids.push_back(id);
        
        // Log the objects to be removed
        RCLCPP_INFO(node_->get_logger(), "Removing collision object: %s", id.c_str());
        
        // Perform the removal
        planning_scene_interface_.removeCollisionObjects(object_ids);
        
        // Confirm removal
        existing_objects = planning_scene_interface_.getKnownObjectNames();
        if (std::find(existing_objects.begin(), existing_objects.end(), id) == existing_objects.end()) {
            RCLCPP_INFO(node_->get_logger(), "Successfully removed collision object: %s", id.c_str());
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to remove collision object: %s", id.c_str());
            return success;
        }
        success = true;
        return success;
    }

private:
    moveit_msgs::msg::ObjectColor addColor(const std::string& id, const std_msgs::msg::ColorRGBA& color) {
        moveit_msgs::msg::ObjectColor object_color;
        object_color.id = id;
        object_color.color = color;
        RCLCPP_INFO(node_->get_logger(), "Added color for object: %s", id.c_str());
        return object_color;
    }

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

#endif // UPDATE_PLANNING_SCENE_H