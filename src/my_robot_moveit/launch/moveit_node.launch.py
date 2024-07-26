import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type", default="ur5e")
    description_package = LaunchConfiguration("description_package", default="ur_description")
    description_file = LaunchConfiguration("description_file", default="ur.urdf.xacro")
    moveit_config_package = LaunchConfiguration("moveit_config_package", default="ur_moveit_config")
    moveit_config_file = LaunchConfiguration("moveit_config_file", default="ur.srdf.xacro")
    prefix = LaunchConfiguration("prefix", default="")
    safety_limits = LaunchConfiguration("safety_limits", default="true")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin", default="0.15")
    safety_k_position = LaunchConfiguration("safety_k_position", default="20")

    # Paths to parameters
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "joint_limit_params:=", joint_limit_params,
            " ",
            "kinematics_params:=", kinematics_params,
            " ",
            "physical_params:=", physical_params,
            " ",
            "visual_params:=", visual_params,
            " ",
            "safety_limits:=", safety_limits,
            " ",
            "safety_pos_margin:=", safety_pos_margin,
            " ",
            "safety_k_position:=", safety_k_position,
            " ",
            "name:=", "ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "prefix:=", prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]),
            " ",
            "name:=", "ur",
            " ",
            "prefix:=", prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='Type/series of used UR robot.'
        ),
        DeclareLaunchArgument(
            'description_package',
            default_value='ur_description',
            description='Description package with robot URDF/XACRO files.'
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='ur.urdf.xacro',
            description='URDF/XACRO description file with the robot.'
        ),
        DeclareLaunchArgument(
            'moveit_config_package',
            default_value='ur_moveit_config',
            description='MoveIt config package with robot SRDF/XACRO files.'
        ),
        DeclareLaunchArgument(
            'moveit_config_file',
            default_value='ur.srdf.xacro',
            description='MoveIt SRDF/XACRO description file with the robot.'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix of the joint names, useful for multi-robot setup.'
        ),
        DeclareLaunchArgument(
            'safety_limits',
            default_value='true',
            description='Enables the safety limits controller if true.'
        ),
        DeclareLaunchArgument(
            'safety_pos_margin',
            default_value='0.15',
            description='The margin to lower and upper limits in the safety controller.'
        ),
        DeclareLaunchArgument(
            'safety_k_position',
            default_value='20',
            description='k-position factor in the safety controller.'
        ),
        Node(
            package='my_robot_moveit',
            executable='moveit_node',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': robot_description_kinematics}
            ]
        )
    ])

