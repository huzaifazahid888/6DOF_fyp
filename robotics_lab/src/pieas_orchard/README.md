# Apple harvesting workcell

[Workspace overview](../../README.md) · [Code walkthrough](docs/WALKTHROUGH.md) · [Model card](models/MODEL_CARD.md) · [Validation](docs/VALIDATION.md)

I am extending the arm from conveyor sorting to fruit harvesting. This separate package uses a small trellised apple tree within the arm's reach, partial leaf occlusion, a learned appearance model and three baskets for ripe, unripe and visibly damaged fruit. The existing sorting package is unchanged.

This is a scaled simulation workcell. The apples are 32 mm in diameter, branches are rigid, and suction/stem detachment uses checked physics joints. It is not a full-size orchard deployment or a validated model of fruit bruising and branch fracture.

The configured three-fruit batch has passed the automated Jazzy/Gazebo integration test and a subsequent local run. Each fruit reached its matching basket. [Validation results](docs/VALIDATION.md) describe the tested scenario and its limits.

## Current implementation

RGB and depth come from a camera on a visible mast attached to the robot's fixed base. A second sensor on the mast supplies LiDAR points. A small trained neural network labels image pixels; connected regions and robust sphere fitting estimate fruit centres and radii. LiDAR corroborates the surface. A visible blemish takes priority over the ripeness class when selecting a basket.

The task plans around shared branch geometry, constrains the tool's approach corridor and includes the carried fruit in MoveIt's scene. Gazebo monitors robot and picked-fruit contact with protected branches and leaves. Contact, stale data, uncertain detection or a failed grasp stops the task. Completely hidden fruit is not inferred from simulator coordinates.

The orchard explicitly uses the simulation suction tool. It does not silently change the original gripper or the sorting launch. Finger actuation remains a separate project milestone.

## Setup

Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic:

```bash
cd ~/6DOF_fyp/robotics_lab
bash scripts/setup_ubuntu.sh --with-moveit --yes --build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pieas_orchard orchard.launch.py
```

The launch starts idle. In a second sourced terminal:

```bash
ros2 run pieas_orchard check_cell --ros-args -p use_sim_time:=true
ros2 topic echo /orchard/detections --once
```

The complete harvesting command is:

```bash
ros2 run pieas_orchard harvest --ros-args -p use_sim_time:=true -p execute:=true
```

`cycles:=1` limits a commissioning run to one fruit. A fresh launch is required before another batch. No old factory or lab launch should run alongside this one. Ctrl+C requests cancellation; a held fruit remains attached on fault. Reset during a task requires a fresh launch.

The current acceptance status and any known runtime failures are recorded in [VALIDATION.md](docs/VALIDATION.md). An implemented command is not evidence that the integration test passed.

## Files and reading order

| File | Responsibility |
| --- | --- |
| [orchard.yaml](config/orchard.yaml) | Scene dimensions, fruit appearance, basket locations and thresholds |
| [harvester.py](pieas_orchard/harvester.py) | Observe, approach, detach, carry, grade and verify sequence |
| [motion.py](pieas_orchard/motion.py) | Nearby IK candidate ranking and fresh stationary feedback checks |
| [vision.py](pieas_orchard/vision.py) | Learned segmentation, sphere fitting, grading and occlusion rejection |
| [perception.py](pieas_orchard/perception.py) | ROS sensor inputs, timing and stable observations |
| [orchard_system.cpp](src/orchard_system.cpp) | Stem attachment, suction interlocks and tree-contact status |
| [world.py](pieas_orchard/world.py) | Tree, foliage, apples, baskets and robot-mounted sensor geometry |
| [orchard.launch.py](launch/orchard.launch.py) | Starts the connected system |
| [train_appearance.py](training/train_appearance.py) | Reproducible synthetic appearance-model training |
| [appearance.json](models/appearance.json) | Learned weights, class names and held-out synthetic results |
| [test_core.py](test/test_core.py) | Numerical, perception and model checks |
| [runtime_smoke.py](test/runtime_smoke.py) | Actual rendered sensor and three-placement acceptance test |

## Interpretation of results

Ripeness means a synthetic red/green appearance class. Quality means a visible simulated surface blemish, not taste, firmness, disease diagnosis or internal quality. The classifier has not been trained on real orchard photographs. Partial occlusion is supported only when enough visible depth surface remains; difficult observations are rejected.

Zero reported tree contact is a useful experiment metric, not proof of zero biological damage. The stem is detached by a simulation command after a proximity/orientation check, without calibrated cutting, twisting or break force. The limited release height is also not a calibrated bruising model.

