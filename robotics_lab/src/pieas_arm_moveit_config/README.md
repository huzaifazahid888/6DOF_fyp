# MoveIt milestone 1: check geometry, then plan and execute

MoveIt plans collision-checked joint motion. Gazebo simulates contact and dynamics;
ros2_control executes the timed joint trajectory. Adding MoveIt does not calibrate
masses, inertias, actuator torque, collision shapes or mechanical end stops.

The previous full-range sweep produced observed link penetration and is retired.
The launch now explicitly enables model and link self-collision in the generated
SDF. SDFormat excludes directly joint-connected links from self-contact. Existing
mesh collisions remain unchanged; their representation in the physics engine
must still be inspected, particularly around concave rail and arm geometry.

## 1. Install and rebuild

Stop the previous Gazebo launch. In a terminal outside Conda/virtualenv:

```bash
cd ~/6DOF_fyp
git pull --ff-only
cd robotics_lab
bash scripts/setup_ubuntu.sh --with-moveit --yes --build
```

The setup script installs missing packages and uses rosdep for workspace
requirements. If dependencies are already installed, rebuild with:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. Launch Gazebo, MoveIt and RViz together

```bash
source /opt/ros/jazzy/setup.bash
source ~/6DOF_fyp/robotics_lab/install/setup.bash
ros2 launch pieas_arm_moveit_config planning.launch.py
```

Do not also launch the old lab in a second terminal: this launch includes it.
It starts the real Gazebo controllers, not a fake robot. No motion is commanded.
The scene setup node adds the z=0 lab ground to MoveIt's planning scene. Only
base_link-to-ground contact is allowed, because that base is fixed to the ground.
Wait for `Planning scene ready` before using RViz planning controls.

Confirm Self Collide is on in Gazebo. If the robot shakes or jumps immediately,
pause it: this can expose overlapping collision geometry previously hidden by
self-collision being off. Do not solve that by disabling all collision checks.

## 3. Validate current state and preview a small plan

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/6DOF_fyp/robotics_lab/install/setup.bash
ros2 run pieas_arm_moveit_config plan_joint_goal --ros-args -p use_sim_time:=true
```

This checks fresh advancing joint feedback, refuses a moving robot, asks MoveIt
whether the measured pose is valid, and reports colliding link pairs if invalid.
It then plans a small six-joint goal with OMPL RRTConnect. It does NOT move Gazebo.
RViz previews the planned trajectory through the MotionPlanning display. Select
planning group `arm` if needed. The target is rail +10 mm and small rotary angles.
The first-lesson client restricts targets to +/-20 mm rail and +/-0.3 rad angles.
These are teaching restrictions, not validated mechanical limits.

The planner checks the URDF collision meshes and the ground represented in its
scene. It does not automatically know about newly inserted Gazebo objects or
sensor observations. The supplied SRDF excludes only directly adjacent pairs;
no non-adjacent contacts are silently ignored because they occur at home.

If the check fails, send the reported contact pairs and the move_group error.
Fix mesh/frame/joint-limit errors or choose a valid starting pose before executing.
A refusal is useful diagnostic evidence; it is not a reason to bypass checking.

## 4. Execute only after the preview is sensible

```bash
ros2 run pieas_arm_moveit_config plan_joint_goal --ros-args -p use_sim_time:=true -p execute:=true
```

This repeats validation and plans afresh from the current state before execution.
MoveIt sends the result to the existing `/arm_controller/follow_joint_trajectory`
action. Do not run any old trajectory client at the same time.

To plan a return home first, then execute it, use respectively:

```bash
ros2 run pieas_arm_moveit_config plan_joint_goal --ros-args -p use_sim_time:=true -p target:="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
ros2 run pieas_arm_moveit_config plan_joint_goal --ros-args -p use_sim_time:=true -p execute:=true -p target:="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

Ctrl+C requests cancellation. If cancellation is unconfirmed, pause Gazebo.
Do not reset the simulation clock while a motion is active.

## What each component teaches

| Component | Responsibility |
|---|---|
| URDF | Links, joint frames, collision meshes, mass and inertia |
| SRDF | Six-joint planning group, named poses, adjacent collision exclusions |
| OMPL RRTConnect | Search for a joint-space path around modeled collisions |
| Time parameterization | Add timing using velocity and acceleration limits |
| MoveIt controller connection | Send the planned trajectory to arm_controller |
| ros2_control + Gazebo | Execute motion and report simulated joint feedback |
| RViz | Inspect the planning model and proposed trajectory |

`config/joint_limits.yaml` deliberately uses conservative planning speeds and
assumed acceleration limits; the example applies 10% speed/acceleration scaling.
The 30-second requirement is retired for this milestone. Plan duration comes from
the path and limits. Speed does not take priority over finding a valid path.

The first milestone is joint-space planning. No IK solver or interactive tool-pose
marker is configured yet. This robot has one rail plus five rotary joints, so
its attainable tool orientations need to be checked before promising arbitrary
six-dimensional pose goals. Gripper fingers remain fixed.

## Next milestones and acceptance criteria

1. Current-state validation passes at home; collision geometry and rail travel
   are reviewed; the robot holds still with self-collision enabled.
2. Preview, execute and return home succeed without penetrating links or ground.
3. Add the same obstacle in Gazebo and MoveIt's scene; show a detour or rejection.
4. Validate forward kinematics and an IK solver, then command reachable tool poses.
5. Add depth/LiDAR observations to update the planning scene and replan.

Offline checks verify Python/XML/YAML syntax, joint/controller consistency,
conservative planning speeds, and adjacent-only collision exclusions. The
integration still requires a ROS 2 Jazzy/Gazebo run on the target machine.

References:
- https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html
- https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html
- https://moveit.picknik.ai/main/doc/examples/ompl_interface/ompl_interface_tutorial.html
- https://sdformat.org/spec/1.7/model/
