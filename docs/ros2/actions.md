# ROS 2 Actions

Actions are used for **long-running tasks** where:
- the robot needs time to finish  
- feedback is useful  
- the task might need to be canceled  

Examples:
- Navigation (move to pose)
- Manipulation (pick/place)
- Drone missions
- Arm motions

---

# 1. Action Structure

Actions have **three message types**:


Goal
Feedback
Result

Example: `NavigateToPose.action`


Goal

geometry_msgs/PoseStamped pose

Result

bool success

Feedback

float32 progress


---

# 2. Action Communication Pattern



Client → sends goal
Server → executes goal
Server → sends feedback
Client → cancel / wait
Server → sends result


---

# 3. Creating an Action

Add `.action` file in:


my_robot_interfaces/action/


Then add to CMakeLists:
```cmake
rosidl_generate_interfaces(...)

4. Action Client Example (Python)
import rclpy
from rclpy.action import ActionClient
from my_robot_interfaces.action import Move

class MoveClient:
    def __init__(self):
        self.client = ActionClient(node, Move, "move")

    def send_goal(self, x):
        goal_msg = Move.Goal()
        goal_msg.distance = x
        self.client.send_goal_async(goal_msg)

5. Action Use Cases
Task	Why Action?
Move robot base	Long-running, needs feedback
Pick up object	Might fail mid-way
Drone flight	Requires cancel option
Mapping	Continuous progress
TL;DR

Actions = async tasks with feedback + cancel control.
