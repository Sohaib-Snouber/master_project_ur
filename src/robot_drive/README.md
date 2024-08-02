# FullDrive Package

## Overview

The `FullDrive` package is designed to control a robotic system using ROS 2, integrating motion planning, collision management, and gripper control. This package leverages MoveIt Task Constructor (MTC) for complex motion planning tasks and integrates with a UR robot through the RTDE interface. 

## Dependencies

- MoveIt
- ROS 2
- RTDE

## Package Contents

1. **Nodes**:
    - **sync_rviz_node.cpp**: Synchronizes the RViz environment with the current state of the real robot using the rtde library.
    - **planning_scene_node.cpp**: Manages the planning scene, including the addition and removal of collision objects.
    - **action_server_node.cpp**: Handles various actions requested by clients, including motion planning, collision management, and gripper control.

2. **Client Node**:
    - **usage_node.cpp**: Allows users to select and execute specific actions. Utilizes `client.h` for compact and efficient action requests.

3. **Header Files**:
    - **client.h**: Provides a streamlined interface for the client node to request actions from the action server.
    - **mtc.h**: Manages MTC stages and execution for motion planning tasks.
    - **update_planning_scene.h**: Interfaces with the MoveIt planning scene to add, remove, and modify collision objects.

## Action Definitions

The `FullDrive.action` file defines the structure of the actions that can be requested by the client. The available actions are:

- `add_collision_object`
- `delete_collision_object`
- `attach_object`
- `detach_object`
- `move_to`
- `move_linear`
- `check_robot_status`
- `allow_collision`
- `reenable_collision`
- `current_state`
- `set_gripper_position`

For more details on the `action_interfaces` package, visit the [action_interfaces](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/action_interfaces).

## How to Run

### Launching the Nodes

1. **Running RViz**: 
    ```bash
    ros2 launch mtc_tutorial mtc_demo.launch.py
    ```
    - This node must run first to run the RViz environment with the needed tools.
    - For more details on the `mtc_tutorial` package, visit the [mtc_tutorial](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/mtc_tutorial).

2. **Launch the Main Nodes**:
    ```bash
    ros2 launch full_drive full_drive.launch.py
    ```
    - This launch file will start fisrt sync_rviz_node then the `planning_scene_node` and `action_server_node`.

### Running the Client Node

The client node can be run separately, allowing the user to request specific actions:

```bash
ros2 run full_drive usage_node
```

## Functionality

### Motion Planning

The action server uses MTC for motion planning. The following tasks can be planned and executed:

- **Move to a specific pose**: `move_to`
- **Move linearly to a specific pose**: `move_linear`

### Gripper Control

The action server can control the gripper by setting its position:

- **Set gripper position**: `set_gripper_position`

### Collision Management

The action server can manage collision objects in the planning scene:

- **Add collision object**: `add_collision_object`
- **Delete collision object**: `delete_collision_object`
- **Attach object to a link**: `attach_object`
- **Detach object from a link**: `detach_object`
- **Allow collision between objects**: `allow_collision`
- **Re-enable collision between objects**: `reenable_collision`

### Checking Robot Status

The action server can check if the robot is currently moving:

- **Check robot status**: `check_robot_status`

## License
This project is licensed under the Apache 2.0 License. See the [LICENSE](/LICENSE) file for details.

## Acknowledgments

Thanks to the MoveIt and ROS 2 community for their continuous support and development of these powerful tools. I would like to specially thank the AMRL team at HAWK University for providing me the opportunity to work on this project.
