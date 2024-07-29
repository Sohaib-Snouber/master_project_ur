#include "full_drive/client.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <functional>
#include <Eigen/Dense>


class ClientNode : public rclcpp::Node {
public:
    ClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("client_node", options), client_actions_(std::make_shared<ClientActions>()) {

        // Start executeActions
        executeActions();
        }

private:
    std::shared_ptr<ClientActions> client_actions_;
    const int max_attempts = 500;

    bool tryAction(std::function<bool()> action, const std::string& action_name) {
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            if (action()) {
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "Attempt %d of %d for %s failed. Retrying...", attempt + 1, max_attempts, action_name.c_str());
        }
        RCLCPP_ERROR(this->get_logger(), "All %d attempts for %s failed.", max_attempts, action_name.c_str());
        return false;
    }

    void executeActions() {
        tf2::Quaternion q1;
        q1.setRPY(M_PI, 0, 0);
        // Second moveTo with additional 90 degrees rotation on y-axis
        tf2::Quaternion q2 = q1 * tf2::Quaternion(tf2::Vector3(0, 1, 0), M_PI_2);
        // First moveLinear with additional 90 degrees rotation on z-axis
        tf2::Quaternion q3 = q2 * tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI_2);
        double piece_height = 0.015;  // the height of the piece is 1.5cm
        double piece_diameter = 0.025;  // the diameter of the piece is 2.5cm
        double robot_base_height = 0.015; // 1,5 cm is the hieght of the base of the real robot, that it is not configured in rviz 
        double dif_btw_tip_tcp = 0.24; // 22cm the distacne between the tip of the gripper and the TCP point of the wrist3_link (the planning target will get to that link point)
        double dist_btw_objects = 0.005; //0.5 cm distance between the objects, that will be grasped
        double dist_btw_targets = 0.012; // distannce between vertical targets 
        double gripper_finger_width = 0.005; // the gripper finger width is 0.5 cm
        double num_objects = 12; // this will make like counter of the number of objects that have been grasped
        double pallet_width = 0.06;  // 6cm the width of tha pallet, that the objects are going to be placed on.
        double pallet_height = 0.09;
        double pallet_offset = 0.01;  // the pallet have material with 1cm above the surfce of the robot

        double x_targets1 = 0.342; // the distance from robot origin till the lower edge of the first pallet
        double y_targets1 = 0.1345;  // the distance from robot origin till the lower left edge of the first pallet
        double x_objects1 = 0.372; // the distance from robot origin til the mid of all objects x values in robot coordinates
        double y_objects1 = 0.273; // this is the distance from robot origin till the right edge of the first object

        int cols_counter = 0;
        int rows_counter =0;

        bool add_constrain;
        double motion_speed;
        // information about the set gripper function:
        // the 0.0 is open, 0.7 is close, and in between is not linear estimation
        // Value | gripper state in cm
        //  0.0  |    14   cm  (full open)
        //  0.1  |    13.5 cm
        //  0.2  |    11.5 cm
        //  0.3  |    9    cm
        //  0.4  |    6.5  cm
        //  0.5  |    4    cm
        //  0.6  |    2    cm
        //  0.7  |    0    cm   (full close)

        for (int i = 0; i < num_objects; ++i) {
            std::string object_name = "cylinder_object" + std::to_string(i);
            RCLCPP_INFO(this->get_logger(), "Adding the %s", object_name.c_str());
            
        }
        
        // Create object poses, the output will be like: object_poses = [0,1,2,3,4,5,6,7,8,9,10,11,12], where 0 is the objects poses, and the other ones are the actual poses for every object with its number respectively
        std::vector<geometry_msgs::msg::PoseStamped> object_poses;
        for (int i = 0; i <= num_objects; ++i) {
            double y_position = y_objects1 + (i + 1) * (piece_diameter / 2.0) + i * (piece_diameter / 2.0) + i * dist_btw_objects;
            geometry_msgs::msg::PoseStamped pose = createTargetPose(x_objects1, y_position, (2 * piece_height / 3) - robot_base_height + dif_btw_tip_tcp, q1);
            object_poses.push_back(pose);
        }
        
        std::vector<geometry_msgs::msg::PoseStamped> object_mid_poses;
        for (int i = 0; i <= num_objects; ++i) {
            double y_position = y_objects1 + (i + 1) * (piece_diameter / 2.0) + i * (piece_diameter / 2.0) + i * dist_btw_objects;
            geometry_msgs::msg::PoseStamped pose = createTargetPose(x_objects1, y_position, (2 * piece_height / 3) - robot_base_height + dif_btw_tip_tcp + 0.1, q1); // 10cm above the object
            object_mid_poses.push_back(pose);
        }

        // Create target poses, the output will be like: target_poses = [0,1,2,3,4,5,6], where 0 is the targets poses, and the other ones are the actual poses for every target with its number respectively
        // the first pallet positions is added now
        // Create target poses
        std::vector<geometry_msgs::msg::PoseStamped> target_poses;
        for (int i = 0; i <= 6; ++i) {
            double x_position = x_targets1 + pallet_height - (i / 2 + 1) * (piece_diameter / 2.0) - (i / 2) * (piece_diameter / 2.0) - (i / 2) * dist_btw_targets; // examples on i/2 > 1/2 = 0, 3/2 = 1
            double y_position = y_targets1 - (i % 2 + 1) * (piece_diameter / 2.0) - (i % 2) * (piece_diameter / 2.0) - (i % 2) * dist_btw_objects;  //examples on i%2 > 1%2 = 1, 2%2 = 0, 0%2 = 0
            geometry_msgs::msg::PoseStamped pose = createTargetPose(x_position, y_position, pallet_offset + dif_btw_tip_tcp, q1);
            target_poses.push_back(pose);
        }

        std::vector<geometry_msgs::msg::PoseStamped> target_mid_poses;
        for (int i = 0; i <= 6; ++i) {
            double x_position = x_targets1 + pallet_height - (i / 2 + 1) * (piece_diameter / 2.0) - (i / 2) * (piece_diameter / 2.0) - (i / 2) * dist_btw_objects;
            double y_position = y_targets1 - (i % 2 + 1) * (piece_diameter / 2.0) - (i % 2) * (piece_diameter / 2.0) - (i % 2) * dist_btw_objects;
            geometry_msgs::msg::PoseStamped pose = createTargetPose(x_position, y_position, pallet_offset + dif_btw_tip_tcp + 0.1, q1); // 10cm above the target
            target_mid_poses.push_back(pose);
        }

        // start tasks
        for (int i = 0; i < num_objects; ++i) {
            std::string object_name = "cylinder_object" + std::to_string(i+1);
            RCLCPP_INFO(this->get_logger(), "Adding the %s", object_name.c_str());
            if (!tryAction([&]() { 
                return addCollisionObject(
                    object_name, 
                    shape_msgs::msg::SolidPrimitive::CYLINDER, 
                    {0.015, 0.025 / 2.0}, 
                    {x_objects1, y_objects1 + (i + 1) * (piece_diameter / 2.0) + i * (piece_diameter / 2.0) + i * dist_btw_objects, piece_height / 2.0}, 
                    createColor(0.8, 0.2, 0.2, 1.0)
                );
            }, "addCollisionObject(" + object_name + ")")) return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        /* RCLCPP_INFO(this->get_logger(), "moving robot to objects position with shoulder constrain");
        if (!tryAction([&]() { return moveTo(object_mid_poses[1], add_constrain = true, precise_motion = true); }, "moveTo")) return;
        //std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "moving robot to objects position with shoulder constrain");
        if (!tryAction([&]() { return moveTo(target_mid_poses[1], add_constrain = true, precise_motion = true); }, "moveTo")) return;
        //std::this_thread::sleep_for(std::chrono::seconds(1)); */

        
        for (int i = 0; i < 6; ++i) {
            RCLCPP_INFO(this->get_logger(), "moving robot to objects position with shoulder constrain");
            if (!tryAction([&]() { return moveTo(object_mid_poses[i], add_constrain = true, motion_speed = 0.1); }, "moveTo")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "moving to objects linearly");
            if (!tryAction([&]() { return moveLinear(object_mid_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "allowing collisions with the cylinder object");
            std::string object_name = "cylinder_object" + std::to_string(i+1);
            if (!tryAction([&]() { return allowCollision("wrist_3_link", object_name); }, "allowCollision(wrist_3_link, " + object_name + ")")) return;
            if (!tryAction([&]() { return allowCollision("surface", object_name); }, "allowCollision(surface, " + object_name + ")")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            if (!tryAction([&]() { return allowCollision("wrist_3_link", "surface"); }, "allowCollision(wrist_3_link, surface)")) return;

            RCLCPP_INFO(this->get_logger(), "moving to object linearly");
            if (!tryAction([&]() { return moveLinear(object_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "attaching the cylinder object to the gripper");
            if (!tryAction([&]() { return attachObject(object_name); }, "attachObject(" + object_name + ")")) return;

            RCLCPP_INFO(this->get_logger(), "back to above object linearly");
            if (!tryAction([&]() { return moveLinear(object_mid_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            if (!tryAction([&]() { return reenableCollision("wrist_3_link", "surface"); }, "reenableCollision(wrist_3_link, surface)")) return;

            RCLCPP_INFO(this->get_logger(), "moving robot to targets position with shoulder constrain");
            if (!tryAction([&]() { return moveTo(target_mid_poses[i], add_constrain = true, motion_speed = 0.2); }, "moveTo")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "moving to targets linearly");
            if (!tryAction([&]() { return moveLinear(target_mid_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            if (!tryAction([&]() { return allowCollision("wrist_3_link", "surface"); }, "allowCollision(wrist_3_link, surface)")) return;

            RCLCPP_INFO(this->get_logger(), "moving to target linearly");
            if (!tryAction([&]() { return moveLinear(target_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "detaching the cylinder object from the gripper");
            if (!tryAction([&]() { return detachObject(object_name); }, "detachObject(" + object_name + ")")) return;

            RCLCPP_INFO(this->get_logger(), "finished target");
            if (!tryAction([&]() { return moveLinear(target_mid_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            if (!tryAction([&]() { return reenableCollision("wrist_3_link", "surface"); }, "reenableCollision(wrist_3_link, surface)")) return;
        }

        RCLCPP_INFO(this->get_logger(), "All actions completed successfully.");
    }

    std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a) {
        std_msgs::msg::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }
    geometry_msgs::msg::PoseStamped createTargetPose(double px, double py, double pz, const tf2::Quaternion& q) {
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.stamp = this->now();
        target_pose.header.frame_id = "world";
        target_pose.pose.position.x = px;
        target_pose.pose.position.y = py;
        target_pose.pose.position.z = pz;
        target_pose.pose.orientation = tf2::toMsg(q);
        return target_pose;
    }

    //////////////////////////////////////////////////////////////////////////////////
    ///////// Actions ////////////////////////////////////////////////////////////////
    ///////// vvvvvvv ////////////////////////////////////////////////////////////////

    bool addCollisionObject(const std::string &name, uint8_t shape_type, const std::vector<double> &dimensions, const std::vector<double> &position, const std_msgs::msg::ColorRGBA &color) {
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_type;
        primitive.dimensions.assign(dimensions.begin(), dimensions.end());

        geometry_msgs::msg::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.w = 1.0;

        return client_actions_->addCollisionObject(name, primitive, pose, color);
    }

    bool deleteObject(const std::string &name) {
        return client_actions_->deleteObject(name);
    }
    bool attachObject(const std::string &name) {
        return client_actions_->attachObject(name);
    }

    bool detachObject(const std::string &name) {
        return client_actions_->detachObject(name);
    }

    bool moveTo(const geometry_msgs::msg::PoseStamped& target_pose, bool constrain, bool motion_speed) {
        return client_actions_->moveTo(target_pose, constrain, motion_speed);
    }

    bool moveLinear(const geometry_msgs::msg::PoseStamped& target_pose) {
        return client_actions_->moveLinear(target_pose);
    }

    bool allowCollision(const std::string &object1, const std::string &object2) {
        return client_actions_->allowCollision(object1, object2);
    }

    bool reenableCollision(const std::string &object1, const std::string &object2) {
        return client_actions_->reenableCollision(object1, object2);
    }

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<ClientNode>();
    executor->add_node(node);
    executor->spin();
    
    rclcpp::shutdown();
    return 0;
}

// control
// 1- void disconnect()
// 2- bool reconnect()
// 3- bool isConnected()
// 4- bool sendCustomScriptFunction(const std::string &function_name, const std::string &script)
// 5- 

