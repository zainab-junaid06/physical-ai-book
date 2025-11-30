---
id: ros-basics
title: ROS2 Basics
sidebar_position: 1
---

# ROS2 Basics

ROS2 is the operating system for robots.  
It brings **communication**, **interoperability**, and **modularity** to robot software.

If you break it down, ROS2 is basically:

- Nodes = programs  
- Topics = messaging system  
- Services = request/response  
- Actions = long tasks  
- Parameters = config  
- Packages = folders of code  

## Why ROS2 is used in 99% of robotics projects

- Real-time communication  
- Supports microcontrollers → supercomputers  
- Modular: plug and play packages  
- Amazing community (Nav2, MoveIt, SLAM Toolbox)  
- Works on Linux, Windows, macOS  

## ROS2 Workspace structure

```bash
ros2_ws/
 ├── src/
 ├── build/
 ├── install/
 └── log/


You only edit the src folder.

ROS2 Workflow (Simple View)
flowchart LR
    A[Create Package] --> B[Write Nodes]
    B --> C[Connect via Topics/Services]
    C --> D[Build colcon]
    D --> E[Run Nodes]

Core ROS2 commands you’ll use 24/7
ros2 run <package> <node>
ros2 topic list
ros2 topic echo /camera
ros2 param get <node> <param>
ros2 service call <srv> ...

Real Example

A mobile robot running ROS2 uses:

a LiDAR node

a camera node

a mapping node

a navigation stack node

a controller node

ROS2 makes them talk to each other smoothly.
