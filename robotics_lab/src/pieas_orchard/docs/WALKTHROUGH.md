# From sensor image to basket

[Package overview](../README.md)

I use this package to study how perception, motion planning and physical simulation work together in a harvesting task. The sorting experiment remains separate. The first version uses a small trellis, three apples and the simulation suction tool.

## 1. Scene and robot

`config/orchard.yaml` describes dimensions in metres, sensor poses, basket locations and acceptance thresholds. `world.py` builds the Gazebo scene. The `boxes()` function supplies the same branch and basket geometry to Gazebo and MoveIt, so the planner knows which rigid structures occupy the workspace. Decorative foliage and background trees provide visual context; the nearby canopy has a conservative collision guard.

`robot_description()` creates a temporary suction variant and adds a visible mast, camera and LiDAR to the fixed robot base. It does not overwrite the original URDF. The arm joint origins and axes stay unchanged. The original gripper remains a rigid mesh, so this experiment cannot demonstrate finger opening or closing.

## 2. Observations

`perception.py` subscribes to RGB, depth, camera calibration and LiDAR. It rejects stale or poorly synchronized data before calling `vision.detect()`. Camera calibration converts each valid depth pixel into a metric camera point. The known sensor transform converts that point into world coordinates.

`vision.py` runs a small trained neural network on candidate image pixels. Its classes are background, leaf, ripe appearance, unripe appearance and visible defect. Connected image regions become candidates. A robust sphere fit estimates the centre and radius from visible depth points. This supports partial leaf occlusion, but cannot recover a completely hidden fruit.

The candidate must pass size, surface-fit, visible-area, class-confidence and LiDAR checks. Several consistent frames are required. The published detection contains measured geometry and appearance scores. It does not contain a Gazebo object name or read the configured fruit class.

`training/train_appearance.py` explains and reproduces model training. This is a synthetic appearance baseline, not a detector trained on real orchard photographs. The model card lists the evidence and limitations.

## 3. Task decisions

`harvester.py` contains the task sequence. Start with `run()`, then follow its calls to `wait_detection()`, `move()`, `command()` and scene helpers. The sequence observes a fruit, approaches from above, descends, attaches, pulls the fruit toward the open side, lifts, transfers, releases and verifies placement.

A visible defect selects the damaged basket. Otherwise the dominant ripe or unripe appearance selects the corresponding basket. Ambiguous detections are rejected instead of assigned a guessed grade. Previously harvested centres are excluded from later observations.

## 4. Motion planning

`move()` asks the shared numerical kinematics helper for joint configurations that put `tool0` at the requested XYZ while keeping the suction axis upright. IK produces candidate joint configurations; MoveIt then checks state validity and searches for a path between configurations. These are different operations.

The rail constraint limits translation. During descent and extraction, a tool position constraint keeps each planned segment inside a narrow corridor. After attachment the tool first moves 25 mm toward the open side of this trellis, then lifts. A straight upward lift would sweep the larger fruit through the overhanging leaf even though the cup fitted beside it. This extraction direction is a configured workcell assumption, not a general tree-understanding algorithm. The orientation constraint keeps the suction axis upright while allowing wrist yaw. Branch geometry has an added clearance margin. MoveIt also includes the held fruit as an attached collision object during transport.

Execution uses the existing trajectory controller. Afterwards, `motion.py` requires fresh stationary joint feedback for 0.2 simulation seconds. This avoids treating an earlier moving sample as the final state. The wait has a timeout and does not relax the velocity limits. Then forward kinematics from measured joint feedback verifies the endpoint. Planning success alone does not count as successful movement. The arm can stop on tracking error, stale feedback, invalid state or a contact fault.

## 5. Simulated grasp and release

`src/orchard_system.cpp` runs inside Gazebo. Each fruit starts attached to a branch through a fixed physics joint. A grasp command is accepted only near the fruit surface with a sufficiently still, upright tool and a live task heartbeat. The backend removes the stem joint and creates a tool-to-fruit joint. It does not teleport the fruit to a planned pose.

This is a detachment surrogate. There is no calibrated stem break force, suction-pressure model, cutting action or fruit deformation. Release removes the tool joint and gravity moves the fruit. Measured simulator position then verifies that the fruit settles inside the selected basket. That position is used for verification, not for detecting where to pick.

## 6. Files that connect the system

`launch/orchard.launch.py` starts Gazebo, ROS bridges, controllers, robot transforms, MoveIt, perception and RViz. The task remains idle unless execution is explicitly requested. `config/bridge.yaml` maps Gazebo sensor and backend messages to ROS topics. `CMakeLists.txt` builds and installs the C++ plugin, Python code and configuration. `package.xml` declares dependencies.

These installation files explain how the application starts, but the main application logic lives in `harvester.py`, `vision.py`, `world.py` and `orchard_system.cpp`.

## 7. Experiments and failure diagnosis

For an initial run, request one fruit with `-p cycles:=1`. Inspect `/orchard/detections` before starting movement. A missing detection can mean stale sensors, insufficient visible surface, uncertain appearance or missing LiDAR corroboration. It does not automatically mean IK failed.

A planning failure occurs later: the target may be unreachable, collide with geometry or violate a corridor constraint. A measured endpoint failure means execution did not reach the expected pose within tolerance. A tree-contact fault is latched and requires a fresh launch. A basket verification failure means the task cannot claim a successful placement even if release was commanded.

`test/test_core.py` checks isolated numerical and perception behaviour. `test/mesh_audit.py` checks sampled configurations against the arm meshes. `test/runtime_smoke.py` checks the actual rendered sensors and complete three-fruit batch. These tests answer different questions; their current results are recorded in `VALIDATION.md`.

Future work includes real image training, camera motion and viewpoint selection, flexible branch interaction, a calibrated detachment mechanism and physical original-gripper actuation. Those extensions require additional models and validation rather than only increasing confidence thresholds.

