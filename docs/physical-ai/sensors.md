---
id: sensors
title: Robot Sensors
sidebar_position: 2
---

# Sensors

Robots can’t do anything blind — sensors are their “superpowers”.

## Main sensor categories

### 🧠 1. Perception Sensors
Used to understand surroundings.

- **Cameras** (RGB)
- **Depth Cameras**
- **LiDAR**
- **Thermal Cameras**
- **Event Cameras**

### 🔊 2. Range & Proximity Sensors

- Ultrasonic
- Infrared
- Time-of-flight

### 🏋️ 3. Force & Tactile Sensors

- Force-Torque sensors
- Soft tactile pads
- Joint torque sensors

### 🧭 4. Motion Sensors

- IMU (Accelerometer + Gyro + Magnetometer)
- Encoders

## How robots fuse sensor data

Robots combine multiple sensors to reduce noise:

- Kalman Filter  
- Extended Kalman Filter  
- Sensor fusion (LiDAR + Camera)  

## Real example  
Spot, Boston Dynamics' robot dog:

- LiDAR for mapping  
- Cameras for depth  
- IMU for balance  
- Joint encoders for movement  
