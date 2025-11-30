# Reinforcement Learning for Robotics

Reinforcement Learning (RL) trains robots through **trial + reward**.

Robot learns a policy π(s) → a  
that maximizes cumulative reward.

---

# 1. Key RL Concepts

- **State (s):** robot position, sensor data  
- **Action (a):** motor commands  
- **Reward (r):** goal signal  
- **Policy (π):** brain of the robot  
- **Episode:** one full run  

---

# 2. Why RL in Robotics?

Robots face:
- complex physics  
- high-dimensional movement  
- uncertain environments  

RL handles:
- balance  
- locomotion  
- manipulation  
- navigation  

---

# 3. Popular RL Algorithms

## 🔹 DQN (Deep Q-Network)
Used for discrete actions.  
Not ideal for continuous robot control.

---

## 🔹 PPO (Proximal Policy Optimization)
🔥 most used in robotics  
Stable + efficient.

Perfect for:
- arms  
- quadrupeds  
- humanoids  
- drones  

---

## 🔹 SAC (Soft Actor-Critic)
Great for:
- smooth continuous control  
- noisy environments  

Industry favorite.

---

# 4. RL + Simulation

Robots are trained in:
- Gazebo  
- Isaac Sim  
- Mujoco  
- PyBullet  

Using:
- domain randomization  
- synthetic environments  

Then transferred to real robot (sim2real).

---

# 5. RL Challenges in Robotics

- sample inefficient  
- slow real-world training  
- safety risk  
- sim2real gap  
- reward engineering issues  

Solutions:
- imitation learning warm start  
- curriculum learning  
- domain randomization  

---

# 6. RL Applications in Robotics

| Robot | RL Use |
|-------|--------|
| Quadruped | Walking / running |
| Drone | Stable flight |
| Manipulator | Push, pull, lift |
| Autonomous Car | Lane following |
| Humanoid | Balance |

---

# TL;DR

RL = robots learning by doing.  
PPO + SAC are the kings of robotics control.

