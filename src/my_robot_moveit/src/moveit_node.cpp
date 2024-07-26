#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

class MoveItNode : public rclcpp::Node
{
public:
  MoveItNode() : Node("moveit_node"), current_pose_index_(0), is_executing_(false)
  {
    // Initialize the target poses
    initialize_target_poses();

    // Schedule a timer to delay the execution of the setup
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MoveItNode::on_activate, this));
  }

private:
  void initialize_target_poses()
  {
    // Define first target pose
    geometry_msgs::msg::Pose pose1;
    pose1.orientation.w = 1.0;
    pose1.position.x = 0.5;
    pose1.position.y = 0.1;
    pose1.position.z = 0.4;
    target_poses_.push_back(pose1);

    // Define second target pose
    geometry_msgs::msg::Pose pose2;
    pose2.orientation.w = 1.0;
    pose2.position.x = -0.3;
    pose2.position.y = 0.5;
    pose2.position.z = 0.7;
    target_poses_.push_back(pose2);
  }

  void on_activate()
  {
    // Initialize the MoveIt! components
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "ur_manipulator");

    // Set velocity and acceleration scaling factors
    move_group_->setMaxVelocityScalingFactor(1.0);  // 1.0 means maximum possible speed
    move_group_->setMaxAccelerationScalingFactor(1.0);  // 1.0 means maximum possible acceleration

    // Add the surface to the planning scene
    setupPlanningScene();

    // Plan and execute the first motion
    plan_and_execute();
  }

  void setupPlanningScene()
  {
    // Create the surface collision object
    shape_msgs::msg::SolidPrimitive primitive_surface;
    primitive_surface.type = primitive_surface.BOX;
    primitive_surface.dimensions = {0.75, 0.8, 0.1};
    geometry_msgs::msg::Pose surface_pose;
    surface_pose.orientation.w = 1.0;
    surface_pose.position.x = 0.16;
    surface_pose.position.y = 0.25;
    surface_pose.position.z = -0.05;
    moveit_msgs::msg::CollisionObject collision_object_surface = createCollisionObject("surface", primitive_surface, surface_pose);

    // Create object color for the surface
    moveit_msgs::msg::ObjectColor surface_color = createObjectColor("surface", 0.5, 0.5, 0.5, 1.0);

    // Create a PlanningScene message and add the collision object and its color
    moveit_msgs::msg::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;
    planning_scene_msg.world.collision_objects = {collision_object_surface};
    planning_scene_msg.object_colors = {surface_color};

    // Apply the planning scene
    planning_scene_interface_.applyCollisionObjects(planning_scene_msg.world.collision_objects);
    
    RCLCPP_INFO(this->get_logger(), "Planning scene setup complete");
  }

  moveit_msgs::msg::CollisionObject createCollisionObject(
      const std::string &id,
      const shape_msgs::msg::SolidPrimitive &primitive,
      const geometry_msgs::msg::Pose &pose)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_->getPlanningFrame();
    collision_object.id = id;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;
    return collision_object;
  }

  moveit_msgs::msg::ObjectColor createObjectColor(
      const std::string &id,
      float r, float g, float b, float a)
  {
    moveit_msgs::msg::ObjectColor color;
    color.id = id;
    color.color.r = r;
    color.color.g = g;
    color.color.b = b;
    color.color.a = a;
    return color;
  }

  void plan_and_execute()
  {
    if (is_executing_) return;

    // Set the executing flag
    is_executing_ = true;

    // Get the current target pose
    const auto &target_pose = target_poses_[current_pose_index_];

    // Set the target pose
    move_group_->setPoseTarget(target_pose);

    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      // Execute the plan
      auto result = move_group_->execute(my_plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Motion executed successfully to target pose %ld.", current_pose_index_);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Motion execution failed to target pose %ld.", current_pose_index_);
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Motion plan failed to target pose %ld.", current_pose_index_);
    }

    // Update the current pose index to alternate between the poses
    current_pose_index_ = (current_pose_index_ + 1) % target_poses_.size();

    // Reset the executing flag
    is_executing_ = false;

    // Plan and execute the next motion
    plan_and_execute();
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_; 
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::Pose> target_poses_;
  size_t current_pose_index_;
  bool is_executing_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
