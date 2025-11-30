# Navigation2 Basics

Navigation2 (Nav2) is the ROS 2 navigation stack.

It handles:
- mapping  
- localization  
- path planning  
- obstacle avoidance  
- goal execution  

---

# 1. Nav2 Architecture



slam_toolbox → map
AMCL → localization
planner → global path
controller → local path
bt_navigator → behavior tree (navigate)


---

# 2. Nav2 Launch

```bash
ros2 launch nav2_bringup navigation_launch.py

3. Required Sensors

LIDAR

IMU

odometry

TF tree

map server

4. Sending a Goal
ros2 action send_goal \
 /navigate_to_pose nav2_msgs/action/NavigateToPose \
 '{pose: { pose: { position: {x: 2.0, y: 1.0 }}}}'

5. Nav2 Config Structure
amcl:
  ros__parameters:
    min_particles: 500

controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

6. Debugging Nav2

robot rotates endlessly → bad localization

path is weird → invalid map

stuck → footprint wrong
