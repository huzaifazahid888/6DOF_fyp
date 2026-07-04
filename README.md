# Supervised Teleoperation Control System for a 6 DOF Serial Chain Manipulator

Final year BS thesis project (Electrical Engineering, PIEAS, 2024) on designing and building the embedded control system for a 6 degree of freedom robotic arm, with a focus on precise motor control and a master-slave communication architecture over CAN bus.

Supervisor: Dr. Nasir Rehman Jadoon

## What this project does

The arm has 6 joints, and each joint is driven by its own DC motor controlled by a dedicated STM32F103C8T6 microcontroller. A separate STM32 acts as the master controller, computes the inverse kinematics for a given target position, and sends the required joint angles (as encoder counts) to each joint controller over CAN bus. Each slave controller then closes its own position control loop using encoder feedback and drives the motor through an IBT-2 driver until it reaches the target.

Limit switches (mechanical on the prismatic joint, optical U-shaped sensors on the revolute joints) cut motor power if a joint reaches the edge of its safe range, so the arm doesn't over-extend or damage itself.

## Architecture

- 1x STM32 as master controller (Nucleo-F767ZI used for kinematics-heavy development/testing)
- 6x STM32F103C8T6 as slave controllers, one per joint
- CAN bus (via MCP2551 transceivers) for master-slave communication
- IBT-2 motor drivers + DC motors + rotary encoders per joint
- Limit switches for joint safety cutoffs
- FreeRTOS on the master controller for task scheduling (mutexes/semaphores for safe shared-resource access during real-time computation)

## Kinematics

Forward and inverse kinematics were derived using standard Denavit-Hartenberg parameters for the arm's link geometry, then implemented in MATLAB and verified against a CoppeliaSim (V-REP) simulation before being ported into the embedded firmware. Given a set of joint angles, forward kinematics gave the same end-effector pose in both MATLAB and simulation, and inverse kinematics was checked the same way, feeding target poses in and confirming the computed joint angles matched.

## Motor control

Each joint uses a closed-loop PID position controller based on encoder feedback. Motion profiles (trapezoidal velocity and cubic polynomial with time slicing) were used to keep joint motion smooth instead of commanding abrupt position jumps.

## Tools used

- STM32CubeMX + Keil uVision for firmware
- FreeRTOS
- MATLAB for kinematics derivation and verification
- SolidWorks for the mechanical CAD model
- CoppeliaSim (V-REP) for simulation and validation before hardware testing

##  Thesis Report  

You can read the complete report here:  [**6DOF_Control – Full PDF Report**](./6DOF_Robotic_Manipulator_Thesis.pdf)

## Notes

This was a team project completed as a BS thesis at PIEAS.
