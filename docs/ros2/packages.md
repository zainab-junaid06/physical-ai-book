# ROS 2 Packages

A **package** is the basic building block in ROS 2.  
Everything lives inside packages:
- nodes  
- launch files  
- URDF  
- actions  
- services  
- configs  

---

# 1. Creating a Package

Python:
```bash
ros2 pkg create my_robot --build-type ament_python


C++:

ros2 pkg create my_robot --build-type ament_cmake

2. Package Structure
my_robot/
 ├── package.xml
 ├── setup.py / CMakeLists.txt
 ├── src/
 ├── launch/
 ├── config/
 ├── urdf/
 └── msg/ srv/ action/

3. Key Files
🔹 package.xml

Dependencies + metadata.

🔹 CMakeLists.txt

For C++ builds.

🔹 setup.py

For Python builds.

4. Listing Packages
ros2 pkg list


Check package path:

ros2 pkg prefix my_robot

5. Declaring Dependencies

Python setup.py:

install_requires=['rclpy', 'std_msgs'],


XML:

<depend>rclpy</depend>
<depend>geometry_msgs</depend>

TL;DR

packages = containers for code, configs, msgs, and everything ROS 2.
