from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Build the MoveIt! configuration
    moveit_config = MoveItConfigsBuilder("full_robot").to_moveit_configs()

    # Define the node
    action_server_node = Node(
        package='full_drive',
        executable='action_server_node',
        name='action_server_node',
        namespace='',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]
    )

    return LaunchDescription([
        action_server_node
    ])
