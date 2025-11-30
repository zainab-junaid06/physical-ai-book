# Isaac ROS and Nav2 for Navigation

## 🛰️ Isaac ROS: Hardware-Accelerated Robotics
**Isaac ROS** is a collection of hardware-accelerated ROS 2 packages that utilize the NVIDIA GPU and Jetson platforms to boost performance for computationally intensive tasks like perception and navigation.

## Visual SLAM (VSLAM)
**Simultaneous Localization and Mapping (SLAM)** is a core navigation challenge where the robot builds a map of an unknown environment while simultaneously keeping track of its own location within that map.

* **Visual SLAM (VSLAM):** Uses camera data (RGB and Depth) instead of LiDAR for mapping and localization.
* **Isaac ROS VSLAM Nodes:** Provides highly optimized packages for VSLAM that run efficiently on NVIDIA hardware. This is essential for the high data throughput of modern depth cameras.

## The Nav2 Stack
**Nav2** is the standard ROS 2 navigation framework. It handles the robot's high-level movement from a starting point to a goal, involving:
1.  **Localization:** Determining the robot's position on the map (using VSLAM/AMCL).
2.  **Global Planning:** Calculating the overall, safe path to the goal.
3.  **Local Planning:** Dynamically avoiding unforeseen obstacles while following the global path.

In a humanoid context, Nav2 is adapted for **bipedal locomotion**, taking into account the complex kinematics and dynamics of walking.