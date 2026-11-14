# Stage 4: Perception and conveyor sorting

[Workspace overview](../../README.md) · [Previous: MoveIt](../pieas_arm_moveit_config/README.md) · [Code walkthrough](docs/WALKTHROUGH.md)

I added this cell to connect motion planning with a complete task. A part arrives on the conveyor, the sensors estimate its shape and position, the application selects a bin, and the robot performs a pick-and-place sequence. The result is checked after release rather than treating a successful movement command as a successful sort.

The batch contains six parts: two cubes, two rectangular blocks and two cylinders. All share the same green colour. Image geometry determines the class; depth estimates position and height; LiDAR provides a separate height and presence check.

## Development status

A six-part batch completed locally with the simulation suction tool. The current factory launch has since been changed to load the original gripper geometry, but the sorting logic still expects suction. That configuration is an unfinished gripper adaptation and must not be presented as a working finger-grasping demo.

The original gripper is one rigid link without actuated fingers. Selectable suction/original modes, movable fingers and physical grasp verification are planned. No `gripper_type` launch argument exists yet. The original arm lab and MoveIt experiments remain separate entry points.

[Recorded results and limits](docs/VALIDATION.md) · [Current roadmap](../../docs/ROADMAP.md)

## What the baseline includes

| Component | Implementation |
| --- | --- |
| Conveyor | Indexed motion from bounded traction forces; stops for picking |
| RGB-D camera | Fixed overhead view, segmentation and geometric shape classification |
| LiDAR | Fixed overhead cloud used to corroborate object height and presence |
| IK | Bounded numerical solution for tool position and upright endpoint axis |
| Planning | MoveIt state validity, collision-checked paths and trajectory timing |
| Grasp | Suction approximation using a checked detachable fixed joint |
| Placement | Release under gravity and simulator-based bin verification |
| Task control | Explicit phases, timeouts, stale-data checks and batch CSV output |

The camera and LiDAR are world-mounted sensor definitions, without visible sensor housings. They are not attached to the robot. Robot mounting needs new geometry and transforms, plus changes to the perception assumptions.

## Run and inspect

The [operation guide](docs/RUNNING.md) contains installation, idle launch, sensor checks and the conditional suction-batch commands. The [troubleshooting guide](docs/TROUBLESHOOTING.md) covers faults by stage. The factory starts idle by default.

The code walkthrough explains each part in execution order, including the difference between sensor estimates and simulator ground truth. The scope is a controlled shape-sorting experiment, not general object recognition or a calibrated industrial cell.

## File map

| Read order | File | Purpose |
|---|---|---|
| 1 | [docs/WALKTHROUGH.md](docs/WALKTHROUGH.md) | Full explanation and call order |
| 2 | [config/cell.yaml](config/cell.yaml) | Belt, objects, bins, sensors, tolerances and operating envelope |
| 3 | [pieas_sorting/world.py](pieas_sorting/world.py) | Generate Gazebo world and temporary suction robot variant |
| 4 | [config/bridge.yaml](config/bridge.yaml) | Translate Gazebo sensor/control messages to/from ROS |
| 5 | [pieas_sorting/perception_core.py](pieas_sorting/perception_core.py) | Projection, silhouette classification and LiDAR corroboration |
| 6 | [pieas_sorting/perception.py](pieas_sorting/perception.py) | ROS subscriptions, freshness, stability and debug output |
| 7 | [pieas_sorting/kinematics.py](pieas_sorting/kinematics.py) | Bounded upright-tool numerical IK; yaw is free |
| 8 | [pieas_sorting/sorter.py](pieas_sorting/sorter.py) | Task states, MoveIt scene changes, planning, execution and verification |
| 9 | [pieas_sorting/task_logic.py](pieas_sorting/task_logic.py) | Validate sensor goals and require a stationary drop inside the correct bin |
| 10 | [src/factory_system.cpp](src/factory_system.cpp) | Gazebo traction, station indexing, watchdog and suction physics joint |
| 11 | [launch/factory.launch.py](launch/factory.launch.py) | Connect the complete system |
| 12 | [test/test_core.py](test/test_core.py) | CPU tests of geometry, perception and IK |
| 13 | [test/runtime_smoke.py](test/runtime_smoke.py) | Explicit real ROS/Gazebo sensor + pick/place test |

All positions in configuration are world metres; angles are radians. Bin names map perception classes to destinations. `cell.yaml` is not a general robot calibration file: changing camera pose requires matching real-world calibration if this is later deployed on hardware.
