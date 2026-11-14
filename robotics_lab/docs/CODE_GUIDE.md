# Code reading guide

[Workspace overview](../README.md) · [Setup](../START_HERE.md) · [Roadmap](ROADMAP.md)

The repository is organised by responsibility. I keep the robot model, basic control experiments, planning integration and factory application in separate packages so later scenarios can reuse the earlier work.

## File types

| Type | Meaning in this project |
| --- | --- |
| `.urdf` | Robot links, joints, mesh references, collision geometry and inertias |
| `.urdf.in` | A URDF template resolved during launch preparation |
| `.stl` | A surface mesh; it defines shape, not joints or actuation |
| `.sdf` | Gazebo world, models, physics and sensor definitions |
| `.srdf` | MoveIt planning groups, named states and collision exclusions |
| `.yaml` | Parameters such as controller settings, limits and cell dimensions |
| `.launch.py` | Python code that prepares configuration and starts ROS/Gazebo processes |
| `.py` | Python application or supporting logic |
| `.cpp` | Compiled C++ source, including the factory Gazebo plugin |
| `.rviz` | Saved RViz views and display settings |
| `package.xml` | ROS package identity and dependencies |
| `CMakeLists.txt`, `setup.py` | Build and installation rules |
| Extensionless files in `scripts/` | Executable entry points that call Python application code |

A mesh can contain the visible shape of several mechanical parts. Independent movement still requires separate links and joints. This is why the original gripper can look complete while its fingers remain rigid.

## 1. Start with the robot and its interfaces

[pieas_control.urdf.in](../src/pieas_arm_description/urdf/pieas_control.urdf.in) defines the model used by the control launch. The important chain is one prismatic rail joint followed by five rotary joints. `tool0` is a reference frame on the end effector, not a separate motor.

[controllers.yaml](../src/pieas_arm_description/config/controllers.yaml) assigns the joints to the trajectory controller. [lab.launch.py](../src/pieas_arm_description/launch/lab.launch.py) prepares the URDF/SDF, starts Gazebo and connects the controller and sensor bridges. Its `prepare_world()` function is also reused by the factory.

## 2. Follow a direct joint command

[joint_sequence.py](../src/pieas_arm_control/pieas_arm_control/joint_sequence.py) contains the action-client structure and feedback handling. [sequence.py](../src/pieas_arm_control/pieas_arm_control/sequence.py) builds trajectory points. The current executable blocks the retired sweep; this source is useful for understanding the earlier control stage.

The key distinction is between a target, a timed trajectory and measured feedback. A commanded position is not proof that the robot reached it, and a successful controller result is not a collision check.

## 3. Follow a planned movement

[planning.launch.py](../src/pieas_arm_moveit_config/launch/planning.launch.py) loads the robot, MoveIt settings and RViz. [plan_joint_goal](../src/pieas_arm_moveit_config/scripts/plan_joint_goal) defines the reusable planning client. Its flow is node creation, measured-state checks, request construction, planning and optional execution.

[tool_sequence](../src/pieas_arm_moveit_config/scripts/tool_sequence) builds on that client. It reads the initial tool position through FK, constructs XYZ targets, obtains IK solutions and plans all three legs before motion. It then compares the requested endpoints with FK from measured joint states.

The [MoveIt README](../src/pieas_arm_moveit_config/README.md) maps each configuration file to its purpose. The SRDF describes which joints form the planning groups; the YAML files select solvers, planners, limits and controller connections.

## 4. Follow a factory cycle

The factory separates application decisions from Gazebo mechanics:

| Reading order | File | What it contributes |
| --- | --- | --- |
| 1 | [cell.yaml](../src/pieas_sorting/config/cell.yaml) | Layout, object dimensions, sensor poses and operating tolerances |
| 2 | [factory.launch.py](../src/pieas_sorting/launch/factory.launch.py) | Generates the scene and starts the connected processes |
| 3 | [world.py](../src/pieas_sorting/pieas_sorting/world.py) | Shared cell geometry and optional suction model generation |
| 4 | [perception_core.py](../src/pieas_sorting/pieas_sorting/perception_core.py) | Pixel projection, shape classification and LiDAR consistency checks |
| 5 | [perception.py](../src/pieas_sorting/pieas_sorting/perception.py) | ROS inputs, timestamps, stable detections and debug output |
| 6 | [kinematics.py](../src/pieas_sorting/pieas_sorting/kinematics.py) | Forward kinematics and bounded numerical upright-tool IK |
| 7 | [sorter.py](../src/pieas_sorting/pieas_sorting/sorter.py) | Task phases, planning, execution, attachments and result logging |
| 8 | [task_logic.py](../src/pieas_sorting/pieas_sorting/task_logic.py) | Detection validation and placement verification logic |
| 9 | [factory_system.cpp](../src/pieas_sorting/src/factory_system.cpp) | Simulated conveyor traction, indexing, attachment and machine status |

The task entry point is `scripts/sort_batch`, which calls `sorter.main()`. That creates `Sorter` and runs `Sorter.run()`. Each arm movement goes through `Sorter.move()`: measured start, IK candidates, collision validation, path planning, execution and endpoint verification.

The [factory walkthrough](../src/pieas_sorting/docs/WALKTHROUGH.md) explains the projection equations, IK residuals, state transitions and collision-object lifecycle in detail.

## Adapting the system to another scenario

Changing bin positions or belt dimensions begins in `cell.yaml`, but reachability, collision and sensor coverage must be retested. Changing the object family starts in `perception_core.py` and its tests. A new task sequence belongs in the application logic rather than the sensor bridge.

Changing a tool affects its geometry, grasp frame, actuation, pickup targets and permitted contact links. Replacing only a mesh or launch line is not enough. Moving a sensor onto the robot also changes its transform over time; the current fixed-camera projection assumptions must be revised.

Source lives in `src/`, installed packages in `install/`, and generated world files in temporary launch directories. Persistent changes belong in source, followed by a successful build and fresh launch. The generated files are useful for diagnosis, not as the main editing location.
