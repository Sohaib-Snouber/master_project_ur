from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Resolve the paths to the necessary files
    robot_description_path = get_package_share_directory('full_robot_description') + '/urdf/full_robot.urdf.xacro'
    srdf_path = get_package_share_directory('full_robot_moveit_config') + '/config/full_robot.srdf'
    controllers_path = get_package_share_directory('full_robot_moveit_config') + '/config/moveit_controllers.yaml'

    # Build the MoveIt configuration using the modular builder pattern
    moveit_config = (
        MoveItConfigsBuilder("full_robot", package_name="full_robot_moveit_config")
        .robot_description(file_path=robot_description_path)
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(file_path=controllers_path)
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
