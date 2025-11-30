# VLA for Robot Control

## 🗣️ Voice-to-Action Pipeline
The capstone project requires a voice-controlled humanoid. This pipeline utilizes VLA principles:

1.  **Speech Recognition (OpenAI Whisper):** The human's voice command is captured via a microphone (e.g., ReSpeaker array) and transcribed into text.
2.  **Cognitive Planning (LLM):** The transcribed text is fed to a powerful LLM, which uses its understanding of the world and the robot's capabilities to generate a plan.
3.  **ROS 2 Action Translation:** The LLM's plan (e.g., "Go to the red block and pick it up") is translated into a sequence of standard ROS 2 calls (e.g., calling the `MapsToGoal` service, publishing to the `/arm_controller/command` topic).
4.  **Execution:** The robot's controllers execute the ROS 2 commands in the simulated (or real) environment.

## Multi-Modal Interaction
The true power of VLA lies in multi-modal input:
* **Language:** "Pick up *that* object."
* **Vision:** The robot looks at the scene and identifies which object "that" refers to (grounding the language to the visual input).
* **Action:** Executing the final, grounded action.

This is the cutting edge of Physical AI, enabling a natural, intuitive interaction between humans and humanoids.