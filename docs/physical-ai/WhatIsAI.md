---
id: what-is-ai
title: What Is AI?
sidebar_position: 1
---

# What Is AI?

Artificial Intelligence is all about making computers *act smart* — seeing, thinking, learning, and making decisions like humans (but faster).

## Why AI matters in robotics  
Robots need AI to:

- understand the world (vision, sensors)  
- make decisions (planning + reasoning)  
- move safely (control + motion planning)  
- adapt to changes  
- learn from mistakes  

Without AI, robots are basically remote-controlled toys.  
With AI, they’re *autonomous teammates*.

## AI vs ML vs Deep Learning — quick vibe

| Term | Meaning |
|------|---------|
| **AI** | The big umbrella: making machines smart |
| **ML** | Teaching machines from data |
| **Deep Learning** | Using neural networks to learn complex stuff |

## AI pipeline for physical robots

```mermaid
flowchart LR
    S[Sensors] --> P[Perception]
    P --> R[Reasoning]
    R --> C[Control]
    C --> A[Actuators]


Clean. Simple. Real-world usable.
