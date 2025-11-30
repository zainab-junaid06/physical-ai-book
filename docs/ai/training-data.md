# Training Data for Robotics AI

Robotics AI needs high-quality data for:
- vision models  
- manipulation  
- navigation  
- imitation learning  
- reinforcement learning  

---

# 1. Types of Data Robots Use

## 🔹 1. Image Data
Used for:
- object detection  
- segmentation  
- depth estimation  

Sources:
- cameras (RGB)
- stereo cameras
- Isaac Sim synthetic datasets  

---

## 🔹 2. Sensor Data
From:
- LiDAR  
- IMU  
- force-torque sensors  
- joint encoders  

Essential for control + navigation.

---

## 🔹 3. Demonstration Data (IL)
Collected via:
- VR  
- teleoperation  
- Mocap  
- kinesthetic teaching  

Used for imitation learning.

---

## 🔹 4. RL Experience Data
Millions of transitions:


(s, a, r, s')

stored in replay buffers.

---

# 2. Synthetic Data

Simulators like Isaac + Gazebo help generate:
- perfect labels  
- endless variety  
- safe training  

Types:
- segmentation masks  
- bounding boxes  
- depth maps  
- 3D keypoints  

---

# 3. Data Pipelines

Data goes through:
1. Collection  
2. Cleaning  
3. Annotation  
4. Normalization  
5. Augmentation  
6. Training  

---

# 4. Common Robotics Datasets

| Dataset | Use |
|---------|-----|
| COCO | Vision |
| ImageNet | Pretraining |
| RoboNet | Manipulation |
| Isaac Dataset | Synthetic |
| KITTI | SLAM & driving |

---

# TL;DR

Data is EVERYTHING in robotics.  
More data → better policy → safer robots.
