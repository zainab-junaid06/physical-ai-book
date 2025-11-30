# Gazebo ROS2 Integration

Gazebo works seamlessly with ROS2 for robot simulation.

---

## 🔹 ROS2 Plugins

Gazebo uses plugins to interact with ROS2 topics:

Examples:
- differential drive plugin  
- camera plugin  
- lidar plugin  
- joint controller plugin  

---

## 🔹 Launching Gazebo with ROS2

```bash
ros2 launch my_robot gazebo.launch.py


Plugins will auto-load in simulation.

🔹 Common Topics
Purpose	Topic
Command robot	/cmd_vel
Read odometry	/odom
View camera	/camera/image
Joint data	/joint_states
🔹 When to use Gazebo?

Use for:

ROS2 projects

real robot testing

URDF debugging

Not ideal for:

photorealistic rendering

deep AI training

Isaac Sim is better there.
