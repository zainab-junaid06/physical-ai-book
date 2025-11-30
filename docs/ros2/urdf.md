# URDF

**URDF** is the **blueprint of your robot**. It describes the robot’s structure, motion, visuals, and sensors, allowing simulators and ROS tools to understand it. 🏗️

---

## 📝 What URDF Describes

- 🔗 **Links** – Rigid body parts of the robot (base, arms, sensors)  
- 🔄 **Joints** – How links move relative to each other (rotational, prismatic, fixed)  
- 📐 **Geometry** – Shape of each link (box, cylinder, mesh)  
- ⚖️ **Mass & inertia** – For realistic physics simulation  
- 📡 **Sensors** – Cameras, LiDAR, IMUs  
- 👀 **Visuals** – How the robot appears in **RViz** or **Gazebo**

---

## 🏗️ Example Robot Structure (ASCII Diagram)

less
Copy code
    [ Camera Link ]
           |
       Revolute Joint
           |
       [ Arm Link ]
           |
       Revolute Joint
           |
       [ Base Link ]
php-template
Copy code

- Each `[Link]` represents a physical part of the robot.  
- Each `Joint` shows how two links are connected and how they move. 🔄  

---

## 🛠️ Example URDF Snippet

```xml
<robot name="my_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1.57" effort="5.0" velocity="1.0"/>
  </joint>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
  </link>
</robot>
```

## ⚡ Why URDF is Important
 &ensp; &ensp; &ensp;  Provides a single source of truth for robot structure.

&ensp; &ensp; &ensp;  Makes robots compatible with Gazebo, RViz, and ROS controllers.

&ensp; &ensp; &ensp;  Supports physics-based simulation, kinematics, and visualization.

&ensp; &ensp; &ensp;  URDF lets you see, simulate, and control your robot in a way that mirrors the real world. 🌐🤖