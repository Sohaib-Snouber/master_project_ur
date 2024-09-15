# Master Project UR5E-Husky Integration

This workspace contains the ROS2 package for testing the ROS2 Universal Robots Driver. It is also integrating the UR5E robotic arm, 2F-140 gripper, and Husky mobile robot using the ROS2 framework.

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Setup](#setup)
- [Usage](#usage)
- [License](#license)

## Introduction

This project demonstrates how to use the ROS2 Universal Robot Driver to control a Universal Robots e-Series robot. The project includes:
- Example launch files to start the robot driver.
- A MoveIt! node to plan and execute motions for the robot.

## Installation

### Prerequisites

- Ubuntu 22.04
- ROS2 Iron
- A Universal Robots e-Series robot
- Universal Robots ROS2 Driver
- Robotiq URCap for the 2F-140 gripper

### Install the Universal Robots Driver and ros2controlcli

```bash
sudo apt-get install ros-iron-ur
```
and also the ros2controlcli
```bash
sudo apt-get install ros-iron-ros2controlcli
```

### Clone the Repository
clone the following workspace
```bash
git clone https://github.com/Sohaib-Snouber/master_project_ur.git
```

### Build the Package

```bash
cd master_project_ur
colcon build
source install/setup.bash
```

## Setup

### Robot Setup

1. **Install the externalcontrolURCap**:
   - Follow the instructions in the [Universal Robots manual](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/install_urcap_e_series.html) to install the `externalcontrol-1.0.5.urcap`.

## Usage

1. **Start the Universal Robots Driver:**
   ```bash
   ros2 launch full_husky_control rst.launch.py
   ```
   This will start the Universal Robots driver necessary for communication with the UR5E robotic arm.

2. **Run RQT:**
   - Open RQT to run the necessary services for starting the robot without using the teach pendant(your teach pendant should be closed last time on remote mode).
   - Use the following service calls in RQT to make the robot ready:
     - Call `/dashboard_client/power_on` (Service: `std_srvs/srv/Trigger`)
     - Call `/dashboard_client/brake_release` (Service: `std_srvs/srv/Trigger`)
     - Call `/dashboard_client/play` (Service: `ur_dashboard_msgs/srv/Load`) to load the program `master-projekt.urp`. (this will include the external control node, with gripper activation node).

3. **Start Gripper Control Node:**
   ```bash
   ros2 run robotiq_gripper_control robotiq_gripper_control
   ```
   This will start the Gripper driver that contain the gripper services server.

4. **Launch MoveIt Configuration:**
   ```bash
   ros2 launch full_husky_moveit_config move_group.launch.py
   ```
   This will start the move_group node, to allow planning and execution on MoveIt.

5. **Launch RViz for Visualization:**
   ```bash
   ros2 launch full_husky_moveit_config moveit_rviz.launch.py
   ```
   This will start the RViz including the full Husky robot, to allow planning and execution using the interactive marker (just for testing the arm).

Now the system is ready to go with custom nodes to implment any task the robot arm and gripper can do, the next two nodes are my custom developed nodes to test the system.

6. **Start the Full Driver:**
   ```bash
   ros2 launch full_driver full_robot_driver.launch.py
   ```
   This is the actions server to implement all posible action by the arm, moving to target with high speed, low spped, linearly, with constrain, adding objects to planning scene, deleting objects from planning scene, allowing or disallowing collision between links or objects.

7. **Launch the Usage Node:**
   ```bash
   ros2 launch full_driver usage.launch.py
   ```
   This will execute some actions requested in the `usage` node, that will call actions from the full_robot_driver node, to implement robot arm action, and it will call the robotiq_gripper_control node to exceute some gripper services.


## License
This project is licensed under the Apache 2.0 License. See the [LICENSE](LICENSE) file for details.
