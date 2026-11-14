# Stage 1: Robot model and Gazebo

[Workspace overview](../../README.md)

I started with the original arm geometry so the later planning and perception experiments would use the same mechanism as my bachelor’s project.

Extracted from `final forward at.ttt`. The model contains the arm's eight STL files, original joint transforms, travel limits, and scene mass and inertia data. The neutral pose uses zero for the six arm joint positions. The `base_link` frame uses the source scene's axes, with its origin beneath the slide axis at the bottom of the mounting feet. The `tool0` frame is the scene's Dummy reference on the RG2 and is available for planning. Check its offset against the intended grasp point before using it as a grasp TCP.

| URDF joint | V-REP name | Type | Scene travel |
| --- | --- | --- | --- |
| joint_1 | Prismatic_joint | Prismatic | −0.5 to +0.5 m |
| joint_2 | rev1 | Revolute | −π to +π rad |
| joint_3 | rev2 | Revolute | −π to +π rad |
| joint_4 | rev3 | Revolute | −π to +π rad |
| joint_5 | rev4 | Revolute | −π to +π rad |
| joint_6 | Revolute_joint | Continuous | Cyclic |

The URDF preserves the scene travel settings. Check usable travel and collisions before commanding the extremes. In particular, the V-REP slide range may exceed the physical rail travel. Joints 2–5 used zero target effort in the scene. The URDF uses provisional 10 N·m limits for these joints. Treat the limits and masses as simulation inputs until checked against the physical arm. Visual colors were assigned during conversion.

The RG2 is rigid at the scene's saved finger pose. Its visible and collision shapes are included and its moving bodies are combined into one inertial body. The original gripper CAD is courtesy of On Robot ApS, as credited in the scene. The V-REP scripts, gripper actuation, and scene environment are outside the URDF. Verify rights to the source assets before redistribution.

## Run the model and controllers

After the [workspace setup](../../START_HERE.md):

```bash
source /opt/ros/jazzy/setup.bash
source ~/6DOF_fyp/robotics_lab/install/setup.bash
ros2 launch pieas_arm_description lab.launch.py
```

The launch resolves the control description, converts it to SDF and includes the robot in the startup world. It also starts the controllers and lab sensor bridges. [Reset and home](../../RESET_AND_HOME.md) explains why the model remains part of the world after reset.

## Main files

| File | Purpose |
| --- | --- |
| [pieas_6dof_arm.urdf](urdf/pieas_6dof_arm.urdf) | Converted baseline model |
| [pieas_control.urdf.in](urdf/pieas_control.urdf.in) | Model template with control interfaces |
| [meshes](meshes) | Visual and collision surface geometry |
| [controllers.yaml](config/controllers.yaml) | Joint-state broadcaster and arm trajectory controller |
| [bridge.yaml](config/bridge.yaml) | Lab sensor and clock message mappings |
| [lab.launch.py](launch/lab.launch.py) | Model preparation and simulation startup |
| [lab.sdf](worlds/lab.sdf) | Lab environment and fixed sensor rigs |

A mesh describes shape. A link is a rigid body. A joint defines relative motion between links. The gripper mesh contains visible fingers, but those fingers do not have independent actuator joints in this model.

## Validation and next stage

Offline checks covered mesh references, connectivity and forward kinematics across 101 configurations against the source scene chain. Local Gazebo work then established model loading, joint feedback and basic motion. These checks do not validate all contacts, payloads or physical actuator ratings.

[Next: joint control experiments](../pieas_arm_control/README.md).
