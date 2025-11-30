# Gazebo and ROS 2 Integration

## 🌐 The Digital Twin: Gazebo Simulation Environment
**Gazebo** is a powerful 3D robotics simulator that accurately simulates rigid-body dynamics, gravity, and collisions. It is essential for developing and testing robot control algorithms before deployment to physical hardware.

## Bridging Gazebo and ROS 2
Gazebo and ROS 2 communicate primarily through **Gazebo ROS Plugins**. These plugins are shared libraries that load into the simulator and act as the bridge between Gazebo's internal physics engine and ROS 2's communication system (Topics, Services, Actions).

### Key ROS 2 Plugins
* **`gazebo_ros_state`:** Publishes the robot's state (joint positions, link poses) to ROS 2 topics.
* **`ros2_control` Plugins:** Connects Gazebo's physics engine directly to the `ros2_control` framework, allowing standard ROS 2 controllers (like Joint Position Controller) to move the simulated robot.
* **Sensor Plugins:** Simulate outputs from sensors (e.g., LiDAR, cameras) and publish the data to standard ROS 2 topics (e.g., `/scan`, `/camera/depth`).

## Running a Simulation
1.  **URDF/SDF Model:** A description of the robot (including mass, inertia, joints) is loaded into Gazebo.
2.  **Plugin Activation:** The ROS 2 plugins defined within the robot's model are activated.
3.  **Controller Launch:** ROS 2 controller nodes (e.g., PID controllers) are launched, subscribing to command topics and publishing to the robot's joints.