# SLAM Basics: Simultaneous Localization and Mapping

## 🗺️ The SLAM Problem
**SLAM (Simultaneous Localization and Mapping)** is a fundamental problem in robotics: How can a mobile robot build a map of an unknown environment while simultaneously determining its own location within that map?

It's a "chicken-and-egg" problem: a good map requires knowing your location, and a good location estimate requires a good map.

## Core SLAM Components
1.  **Odometry:** Estimates the robot's motion from wheel encoders or IMUs. This is prone to drift (accumulating error over time).
2.  **Sensing:** Uses sensors (LiDAR, Cameras) to gather external environment data.
3.  **Data Association:** Matching current sensor readings to features on the existing map.
4.  **State Estimation (Filtering/Optimization):** Using algorithms like **Extended Kalman Filter (EKF)** or **Graph SLAM** to combine noisy odometry and sensor data into a single, optimized estimate of the robot's pose and the map structure.

## Types of SLAM
* **LiDAR SLAM:** Uses 2D or 3D laser scans for mapping. Highly accurate for distance measurements.
* **Visual SLAM (VSLAM):** Uses camera images (monocular, stereo, or RGB-D) for feature extraction and mapping.