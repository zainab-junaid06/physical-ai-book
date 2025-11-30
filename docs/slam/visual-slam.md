# Visual SLAM (VSLAM)

## 📸 Introduction to VSLAM
**Visual SLAM (VSLAM)** is a subset of SLAM that relies solely on one or more cameras to perform the localization and mapping tasks. This is a common choice for humanoids and small robots where weight and cost constraints may preclude large LiDAR units.

## Key VSLAM Concepts
1.  **Feature Extraction:** Identifying unique, stable points or landmarks in the camera image (e.g., using algorithms like SIFT, ORB).
2.  **Tracking:** Matching features between consecutive frames to estimate the camera's movement (pose change).
3.  **Triangulation and Depth:** Using the known movement and matched features to calculate the 3D position (depth) of the landmarks, building the map.
4.  **Bundle Adjustment:** A powerful optimization technique that simultaneously refines the 3D coordinates of the landmarks and the pose of the camera to minimize the total reprojection error.

## VSLAM in Isaac ROS
NVIDIA's **Isaac ROS** provides GPU-accelerated VSLAM packages that are optimized for high-resolution, high-speed camera data streams, which is necessary for smooth, real-time humanoid navigation. These algorithms often integrate data from an IMU (Visual-Inertial Odometry or VIO) to further improve robustness and accuracy.