# ROS 2 Launch Files

## 🚀 The Purpose of Launch Files
In a complex robot system, you may need to run dozens of executable nodes, set parameters, and start external tools (like Rviz or Gazebo) simultaneously. A **Launch File** is an XML or Python script that automates the process of starting and managing multiple components of a ROS 2 system.

## Launch File Structure (Python)
Modern ROS 2 uses **Python** for its launch system, offering greater flexibility and programmatic control.

A typical launch file uses the `LaunchDescription` object and includes:
* **Node Declarations:** Specifying the executable name, package, and assigned name for each ROS 2 node.
* **Parameter Loading:** Loading YAML configuration files to set node parameters.
* **Group Actions:** Organizing nodes into groups with common settings (e.g., namespacing).
* **Conditional Execution:** Using `IfCondition` or `UnlessCondition` to run nodes only under specific circumstances (e.g., debug mode).

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='control_node',
            name='robot_controller',
            output='screen'
        ),
        # ... other nodes and actions
    ])