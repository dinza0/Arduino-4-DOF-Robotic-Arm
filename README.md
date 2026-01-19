# 4-DOF Robotic Arm Control System

This repository contains the Arduino firmware for a custom-designed **4-degree-of-freedom robotic arm** actuated using **Dynamixel smart servo motors**. The project integrates mechanical design, kinematic modeling, and embedded control to enable precise joint and end-effector motion.

## Project Overview
The robotic arm was fully designed and fabricated by the author, with all structural components **CAD-modeled and 3D printed**. The control system is built around an **Arduino microcontroller**, interfacing with Dynamixel motors to execute commanded joint positions derived from forward and inverse kinematics.

## Features
- Control of a **4-DOF robotic arm** using Dynamixel actuators  
- Implementation of **forward and inverse kinematics** for end-effector positioning  
- Serial command interface for sending target positions  
- Modular joint control architecture for easy expansion  
- Real-time feedback of joint positions  

## Hardware
- Arduino microcontroller  
- Dynamixel smart servo motors  
- Custom 3D-printed arm structure  
- External power supply for actuators  

## Software
- Arduino (C/C++) firmware for motor control and kinematics execution  
- Python used for higher-level command generation and testing  

## Kinematics
The system computes:
- **Forward kinematics** to determine end-effector pose from joint angles  
- **Inverse kinematics** to calculate joint angles required to reach a desired Cartesian position  

These models were derived analytically and validated through physical testing.

## Future Improvements
- Trajectory planning and smoothing  
- Closed-loop force or torque control  
- End-effector tooling integration  
- Expanded DOF or payload capability  

## Author
Aman Dhindsa
# Arduino-4-DOF-Robotic-Arm
