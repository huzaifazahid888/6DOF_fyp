# Ongoing Work: Intelligent Robotics in Simulation

I am extending my bachelor's robotic arm project into a platform for studying perception, motion planning and autonomous manipulation. The original project involved simulation and embedded control on the physical arm. The ongoing work uses the same arm model in **ROS 2, Gazebo and MoveIt 2** to develop and test new applications.

My objective is to connect what the robot observes with how it moves and how it checks the result of a task. The work has progressed from basic joint control to conveyor sorting and, more recently, apple harvesting. These extensions are simulation experiments; they have not yet been deployed on the physical arm.

[Simulation workspace](robotics_lab/README.md) · [Code reading guide](robotics_lab/docs/CODE_GUIDE.md) · [Original FYP and thesis](#original-fyp)

## Development objectives and progress

### 1. Build a working simulation of the original arm

**Objective:** bring the existing robot into a repeatable environment for control and planning experiments.

I recovered the geometry and joint structure from the original CoppeliaSim model and integrated the robot with Gazebo and ROS 2 controllers. The first experiments sent timed joint commands and compared the requested positions with joint feedback. This established the connection between the robot description, controller and simulated motion.

The model has one linear rail joint and five rotary joints.

[Robot model and Gazebo setup](robotics_lab/src/pieas_arm_description/README.md) · [Joint control experiments](robotics_lab/src/pieas_arm_control/README.md)

### 2. Move from joint commands to collision-aware tool motion

**Objective:** specify where the tool should move and find a feasible path through the robot's workspace.

I added MoveIt 2 for inverse kinematics, collision checking, path planning and trajectory timing. RViz provides a view of the planning scene and proposed motion, while Gazebo runs the trajectory through the robot controller. The experiments include joint targets, tool-position sequences and planning around obstacles.

Measured joint feedback is used to check whether the tool reached the requested position. This connects kinematics and planning with the robot's actual simulated response.

[Motion planning and tool-position experiments](robotics_lab/src/pieas_arm_moveit_config/README.md)

### 3. Connect perception and manipulation through conveyor sorting

**Objective:** identify an arriving object, estimate its position and place it in the correct bin.

The factory scenario includes an indexed conveyor, an RGB-D camera, LiDAR and three types of parts: cubes, rectangular blocks and cylinders. Image geometry determines the shape class, depth estimates position and size, and LiDAR provides an additional height and presence check. A task sequence connects detection, motion planning, pickup, release and placement verification.

A six-part batch completed with the simulation suction tool. Work on using the original gripper is still ongoing: the current factory launch displays its geometry, but finger actuation and physical gripping have not been implemented. The factory documentation records this configuration difference.

[Conveyor sorting application](robotics_lab/src/pieas_sorting/README.md) · [Results and current limitations](robotics_lab/src/pieas_sorting/docs/VALIDATION.md)

### 4. Extend the robot to fruit harvesting and appearance-based sorting

**Objective:** use sensor observations to locate fruit near foliage, plan its removal and select a basket from its observed appearance.

The separate orchard application uses a scaled apple trellis within the arm's reach. An RGB-D camera and LiDAR are mounted on a visible mast attached to the robot's fixed base. A small trained neural network classifies image pixels, while depth-based sphere fitting estimates fruit position and size. LiDAR checks the observed surface, and repeated observations help reject unstable detections.

The robot approaches a detected apple, attaches the simulation suction tool, extracts the fruit toward the open side of the trellis and lifts it clear. MoveIt includes the carried fruit in collision checking. IK candidates are ranked by joint travel to avoid unnecessary arm flips, and feedback checks confirm that motion has settled before the next step.

**The three-fruit scenario has completed in both an automated Gazebo test and a local run.** Ripe, unripe and visibly damaged fruit reached their matching baskets. The automated batch reported no contact with the protected tree geometry.

This is the first working harvesting baseline. The classifier is trained on synthetic appearance samples, and the demonstrated scene exposes most of each apple. Partial-occlusion behaviour is also tested with analytic fixtures. These results do not establish recognition in dense real foliage or biological damage prevention.

[Apple harvesting application](robotics_lab/src/pieas_orchard/README.md) · [Implementation walkthrough](robotics_lab/src/pieas_orchard/docs/WALKTHROUGH.md) · [Verified results](robotics_lab/src/pieas_orchard/docs/VALIDATION.md) · [Appearance model](robotics_lab/src/pieas_orchard/models/MODEL_CARD.md)

## Direction of the ongoing work

The main focus is intelligent manipulation: combining learned perception with geometric reasoning, motion planning and feedback-based decisions. Conveyor sorting provides a controlled starting point. Harvesting adds appearance variation, foliage and constraints on how the fruit can be removed.

Next, I plan to test varied fruit positions, stronger occlusion and changing lighting, then explore better camera viewpoints and recovery from failed detections or grasps. These experiments will help evaluate reliability beyond a single configured scene.

Fruit ripeness and health assessment are further directions. The current system uses synthetic red/green appearance and visible blemishes. Real ripeness and health assessment will require labelled images and suitable reference measurements; internal quality and disease are not measured by this implementation.

Mechanical work includes actuated fingers for the original gripper and better-calibrated grasp, contact and detachment models. These are needed to study hardware transfer and the risk of damaging fruit or branches.

## Explore the ongoing work

The simulation workspace targets Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. Each application has its own documentation, configuration and launch entry point.

[Workspace overview](robotics_lab/README.md) · [Installation and launch guide](robotics_lab/START_HERE.md) · [Code reading guide](robotics_lab/docs/CODE_GUIDE.md) · [Development roadmap](robotics_lab/docs/ROADMAP.md)

---

<a id="original-fyp"></a>

# Supervised Teleoperation Control System for a 6 DOF Serial Chain Manipulator

Final year BS thesis project (Electrical Engineering, PIEAS, 2024) on designing and building the embedded control system for a 6 degree of freedom robotic arm, with a focus on precise motor control and a master-slave communication architecture over CAN bus.

Supervisor: Dr. Nasir Rehman Jadoon

<p align="left">
<img src="6_DOF.png" width="400">
</p>

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
