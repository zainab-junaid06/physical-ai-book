# Kinematics

Kinematics is all about describing how a robot moves—without worrying about forces or mass.

---

## 🔹 Forward Kinematics (FK)

FK = “I know the joint angles → tell me where the robot’s end-effector is.”

Example:
- Joint angles (θ1, θ2, θ3)
- Robot geometry (link lengths)
- Output → (x, y, z) position + orientation

FK is always **unique**.

---

## 🔹 Inverse Kinematics (IK)

IK = “I know the target position → tell me what joint angles achieve it.”

IK can be:
- multiple solutions  
- or NO solution  
- or infinite solutions  

Industrial robot arms run IK solvers 1000 Hz for real-time motion.

---

## 🔹 Jacobians

The Jacobian connects:
- joint velocities  
- end-effector velocities  

Used for:
- smooth motion  
- avoiding singularities  
- force control  

---

## 🔹 Singularity

A robot becomes singular when it loses a degree of freedom (DOF).  
Example:
- arm fully stretched  
- wrist aligned with elbow  

Results:
- IK unstable  
- movements unpredictable  

---

## 🔹 TL;DR

Kinematics = geometry of motion  
FK = predictable  
IK = tricky  
Jacobian = movement + forces math
