# ROS 2 Parameter Management

## ⚙️ What are ROS 2 Parameters?
**Parameters** are configuration values that allow you to customize the behavior of ROS 2 nodes at runtime without modifying and recompiling the source code. They function like global variables within a node.

## Setting and Getting Parameters
1.  **Initialization:** Parameters are typically defined and assigned default values within the node's Python or C++ code.
2.  **External Configuration (YAML):** The most common way to configure complex systems is by loading parameters from external **YAML files** using a Launch File.

```yaml
# controller_params.yaml
robot_controller:
  ros__parameters:
    Kp: 0.5    # Proportional gain for PID
    robot_namespace: 'humanoid_01'
    max_speed: 1.5