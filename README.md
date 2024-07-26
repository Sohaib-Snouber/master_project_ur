# Master Project UR

This workspace contains the ROS2 package for testing the ROS2 Universal Robots Driver. It includes the necessary launch files, and example nodes to control the robot.

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

### Clone the ROS2 Universal Robots Driver

```bash
cd master_project_ur/src
git clone -b iron https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
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

### Running the Example MoveIt Node

1. **Launch the Driver**:
   Change the parameter according to your robot
   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=10.130.1.100 launch_rviz:=true
   ```
2. **Launch the MoveIt**:
   Change the parameter according to your robot
   ```bash
   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true^C
   ```
3. **Launch the my_robot_moveit**:
   ```bash
   ros2 launch my_robot_moveit moveit_node
   ```


## License
This project is licensed under the Apache 2.0 License. See the [LICENSE](LICENSE) file for details.
