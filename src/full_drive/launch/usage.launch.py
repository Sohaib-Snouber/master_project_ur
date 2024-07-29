from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    usage_node = Node(
        package='full_drive',
        executable='usage_node',
        name='usage_node',
        output='screen',
    )

    return LaunchDescription([
        usage_node,
    ])
