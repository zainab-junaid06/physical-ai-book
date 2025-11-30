# Isaac Sim: Simulation Basics

Isaac Sim is NVIDIA’s robotics simulator built for:
- photorealistic rendering  
- accurate physics  
- AI training  

---

## 🔹 Features

- Omniverse RTX rendering  
- GPU physics (PhysX)  
- ROS2 support  
- Synthetic data generation  
- Digital twins  

---

## 🔹 Creating a Scene

1. Add robot (URDF / USD)
2. Add environment
3. Add sensors (RGB, depth, LiDAR)
4. Add physics materials
5. Simulate

---

## 🔹 Synthetic Data

Isaac can generate:
- bounding boxes  
- segmentation masks  
- depth maps  
- keypoints  

Useful for training perception models.

---

## 🔹 ROS2 Integration

You can publish:
- `/cmd_vel`
- `/joint_states`
- `/camera/image`

Great for testing robotics code before using hardware.
