# Real-Time PID Controller and Graphical Interface

This repository contains a real-time control system for a PID (Proportional, Integral, Derivative) controller with a graphical user interface (GUI). The system leverages an Arduino-based hardware setup for process control and a Python application using PyQt5 and pyqtgraph for visualization and interaction.

## Key Features:

- PID Control: Fine-tune your system with real-time PID parameter adjustments.
- Serial Communication: Communicates with Arduino over a serial connection to apply control commands and read sensor data.
- Graphical Visualization: Real-time plotting of system variables such as target vs. actual position, error values, and PID contributions.
- User Interaction: Update target values and PID parameters on-the-fly through an intuitive GUI.
- Anti-Windup Handling: Toggle anti-windup feature from the interface to prevent integral term saturation.

Ideal for educational purposes, hobbyists, or as a starting point for more complex control systems.

## Scheme
The scheme of the project is shown below:

![Scheme](/media/scheme.png )

in this case a L298N is utilized but any motor controler is enough.

## Materials

For this project in specific, the materials utilized are shown below:

- Arduino 1
- L298N
- USB B cable
- DC motor with encoder (model unknown)
- Breadboard
  
