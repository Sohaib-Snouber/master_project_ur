# Action Interfaces Package

## Overview

The `action_interfaces` package defines the action interfaces used within the master project. These action interfaces facilitate communication for long-running tasks, providing goal, feedback, and result messages. The package includes two main action definitions: `ProcessTask` and `FullDrive`.

## Action Definitions

### ProcessTask Action

The `ProcessTask.action` file defines the action interface for processing tasks. This interface allows for initiating a task with various requests, receiving feedback during the task's execution, and obtaining the result once the task is complete.

#### Action Definition

The `ProcessTask.action` file is structured as follows:

```plaintext
# Goal
bool request_open_gripper
bool request_close_gripper
bool request_greeting
bool move_robot
bool check_robot_status
---
# Result
string greeting_message
int32 goal_id
---
# Feedback
bool robot_moving
```

#### Usage

This action interface can be used to define servers and clients for processing tasks. The clients can send a goal (with various requests) to the server, which will process the task, provide feedback during execution, and return the result upon completion.

- **Goal**: Contains various task requests.
- **Result**: Contains a greeting message and the goal ID.
- **Feedback**: Provides robot movement status during task execution.

### FullDrive Action

The `FullDrive.action` file defines the action interface for a wide range of robotic actions including motion planning, collision management, and gripper control. This interface provides detailed control over the robot's actions and its interactions with the environment.

#### Action Definition

The `FullDrive.action` file is structured as follows:

```plaintext
# Goal definition
geometry_msgs/PoseStamped target_pose
shape_msgs/SolidPrimitive object_primitive
geometry_msgs/Pose object_pose
std_msgs/ColorRGBA color
bool add_collision_object
bool delete_collision_object
bool attach_object
bool detach_object
bool move_to
bool move_linear
bool check_robot_status
bool allow_collision
bool reenable_collision
bool current_state
bool set_gripper_position
float64 gripper_position
string object_name
string target_name
string task
string id
string link
---
# Result definition
bool success
string message
---
# Feedback definition
float32 progress
string status
```

#### Usage

This action interface can be used to define servers and clients for a wide range of robotic actions. The clients can send a goal with specific requests to the server, which will execute the actions, provide feedback during execution, and return the result upon completion.

- **Goal**: Contains various action requests and parameters for executing robotic actions.
- **Result**: Indicates the success or failure of the requested action and provides a message.
- **Feedback**: Provides progress and status updates during action execution.

## License
This project is licensed under the Apache 2.0 License. See the [LICENSE](/LICENSE) file for details.

