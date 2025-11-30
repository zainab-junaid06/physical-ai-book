---
id: nodes-topics
title: ROS2 Nodes & Topics
sidebar_position: 2
---

# Nodes & Topics

ROS2 nodes are the “apps” running inside your robot.  
Topics are how these apps talk to each other **asynchronously**.

## What is a Node?

A node is simply a running ROS2 program.

Examples:

- `camera_node`  
- `lidar_node`  
- `navigation_node`  

Each node does one job only. That’s the ROS philosophy.

## What is a Topic?

A topic is a **data stream**.

Camera → publishes images  
Navigation → subscribes to laser data  

Example topics:



/camera/image_raw
/scan
/cmd_vel


## Publisher & Subscriber

```mermaid
flowchart LR
    C[Camera Node] -- publishes --> T((/image_raw))
    T -- subscribes --> V[Vision Node]

Publishing example (Python)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.send_msg)

    def send_msg(self):
        msg = String()
        msg.data = "Hello ROS2"
        self.pub.publish(msg)

rclpy.init()
node = Talker()
rclpy.spin(node)

Subscribe example
class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'chatter', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f"Got: {msg.data}")

Check topics
ros2 topic list
ros2 topic echo /chatter
ros2 topic info /scan
