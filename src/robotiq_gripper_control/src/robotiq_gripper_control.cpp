#include "RobotiqCModelURCap.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("gripper_control_node");
    
    RobotiqCModelURCap gripper("10.130.1.100");
    gripper.activate(true);
    gripper.move_and_wait_for_pos(100);
    gripper.move_and_wait_for_pos(255);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
