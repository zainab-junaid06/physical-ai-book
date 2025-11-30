# PID Control

## Introduction
PID (Proportional-Integral-Derivative) control is a widely used feedback control technique in engineering and robotics. It combines three distinct control actions—proportional, integral, and derivative—to provide precise and stable system control.

PID controllers are essential in systems where accuracy and stability are critical, such as motor control, temperature regulation, and robotics.

---

## 1. Basic Concept

A PID controller continuously calculates an error value as the difference between a desired setpoint and a measured process variable. The controller then applies a correction based on proportional, integral, and derivative terms.

**Error calculation:**

error(t) = setpoint - process_variable

scss
Copy code

The control output `u(t)` is:

u(t) = Kp * e(t) + Ki * ∫ e(t) dt + Kd * (de(t)/dt)

yaml
Copy code

Where:  
- `Kp` = Proportional gain  
- `Ki` = Integral gain  
- `Kd` = Derivative gain  

---

## 2. Components of PID

### 2.1 Proportional (P) Control
- Provides control output proportional to the current error.
- Large `Kp` reduces steady-state error but may cause overshoot.
- Formula:  
P_out = Kp * e(t)

markdown
Copy code

### 2.2 Integral (I) Control
- Accounts for past errors to eliminate steady-state offset.
- Large `Ki` reduces steady-state error but can cause slow response or instability.
- Formula:  
I_out = Ki * ∫ e(t) dt

markdown
Copy code

### 2.3 Derivative (D) Control
- Predicts future error based on the rate of change.
- Helps reduce overshoot and improve stability.
- Formula:  
D_out = Kd * (de(t)/dt)

yaml
Copy code

---

## 3. Tuning PID Controllers

PID tuning involves selecting `Kp`, `Ki`, and `Kd` values to achieve desired system response.

### Common Methods:
1. **Ziegler-Nichols Method** – Based on system oscillation response.  
2. **Cohen-Coon Method** – Used for first-order systems with dead time.  
3. **Trial and Error** – Iterative adjustment in real systems.  

**Tips:**
- Start with `Kp` only, then add `Ki`, finally `Kd`.  
- Observe system response (rise time, overshoot, settling time).  
- Adjust gains gradually to avoid instability.  

---

## 4. Applications

- Motor speed and position control  
- Temperature and pressure control systems  
- Robotics and drone flight stabilization  
- Process control in chemical plants  

---

## 5. Advantages and Limitations

### Advantages
- Simple and robust control strategy.  
- Works for linear and moderately nonlinear systems.  
- Easy to implement in software or hardware.

### Limitations
- Requires tuning for each system.  
- May not perform well in highly nonlinear or time-varying systems.  
- Derivative term sensitive to noise.

---

## 6. Summary

PID control is a cornerstone of modern control systems. Its combination of proportional, integral, and derivative actions allows engineers to achieve precise, stable, and responsive system behavior. Proper tuning and understanding of each component are crucial for optimal performance.

---

## References
1. Ogata, Katsuhiko. *Modern Control Engineering*, 5th Edition, 2010.  
2. Franklin, G. F., Powell, J. D., & Emami-Naeini, A. *Feedback Control of Dynamic Systems*, 7th Edition, 2014.  
3. Dorf, R. C., & Bishop, R. H. *Modern Control Systems*, 13th Edition, 2016.