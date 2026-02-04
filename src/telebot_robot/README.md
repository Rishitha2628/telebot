# Level 1: ROS2 Navigation telebot - Mandadi Rishitha

## Overview


## Repository Structure

```
ros_nav2_telebot/
├── telebot_description/
│   ├── launch/            # Launch the full base simulation
│   ├── meshes/
│   ├── rviz/            # RVIZ configuration files
│   └── urdf/            # URDF files for telebot-T1.0.0
├── telebot_gazebo/
│   ├── worlds/            # Simulation world files
│   ├── launch/            # Launch files for Gazebo
│   └── models/            # Misc. Gazebo model files
├── telebot_bringup/
│   ├── launch/            # Launch file for bringing up the robot
│   └── maps/              # Predefined map of the test environment
├── help.md                # Guidelines and FAQs
└── README.md              # Instructions for the telebot
```


## Requirements

To get started, you’ll need:
- ROS2 Humble installed. (Install from [Humble Installation](https://docs.ros.org/en/humble/Installation.html)) (You will need Ubuntu 22.04/Windows 10 for this. More in the help section.)
- Gazebo simulator (version 11.10.2 is compatible with ROS2 Humble) (Install from [Gazebo Installation](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)) and Rviz simulator.
- Basic to intermediate knowledge of ROS2 navigation concepts.
- Familiarity with creating and managing ROS2 packages, actions, and parameter files.
- The reference documentation for the `nav2` stack is going to be your best friend for this telebot: [Nav2 Documentation](https://navigation.ros.org/).

## Instructions

### 1. Setting Up the Repository
1. Create your workspace:
    ```bash
    mkdir -p ~/telebot_ws/src
    ```
2. Clone this repository:
   ```bash
   cd ~/telebot_ws/src
   git clone <repository-url>
   ```
2. Build the workspace:
   ```bash
   cd ~/telebot_ws/
   colcon build
   source install/setup.bash
   ```

### 2. Launching the Simulation Environment
1. Start the full simulation using:
   ```bash
   ros2 launch telebot_bringup telebot_full_bringup.launch.py
   ```
   This brings up the telebot environment in Gazebo and Rviz.




