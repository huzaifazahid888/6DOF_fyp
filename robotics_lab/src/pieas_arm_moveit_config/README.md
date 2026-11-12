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

The first milestone uses joint-space planning. The XYZ milestone below adds
position-only IK. Arbitrary tool orientation is not constrained. Gripper fingers
remain fixed.

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


# Milestone 2: program the tool in XYZ coordinates

`tool_sequence` requests three world-coordinate targets for the saved `tool0`
frame: 30 mm up, then 30 mm sideways at the raised height, then the original
joint configuration. This is a task expressed with tool coordinates; IK chooses
the joints and OMPL chooses collision-checked joint-space paths between them.
The connecting paths are not constrained to be Cartesian straight lines.

## Update and run

Stop the integrated launch, then:

```bash
cd ~/6DOF_fyp
git pull --ff-only
cd robotics_lab
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-select pieas_arm_moveit_config
source install/setup.bash
ros2 launch pieas_arm_moveit_config planning.launch.py
```

In a second terminal, preview the complete task:

```bash
source /opt/ros/jazzy/setup.bash
source ~/6DOF_fyp/robotics_lab/install/setup.bash
ros2 run pieas_arm_moveit_config tool_sequence --ros-args -p use_sim_time:=true
```

After the three plans pass and the preview is sensible, execute:

```bash
ros2 run pieas_arm_moveit_config tool_sequence --ros-args -p use_sim_time:=true -p execute:=true
```

Each invocation plans afresh. Within the execution invocation, all three legs
must plan successfully BEFORE any motion. Those exact trajectories are then
executed in order, not independently replanned after each movement. Changed
starting joints cause refusal. Keep the scene static and use only one motion
client during the task. Execution uses the existing arm_controller. Ctrl+C
requests cancellation; pause Gazebo if cancellation is not confirmed.

The console prints starting XYZ, each target, planned endpoint error, and the
FK position error calculated from measured joint feedback after each execution.
Default maximum tool-position error is 3 mm. That error measures consistency
with the same URDF; it is not independent calibration of physical tool placement.

If the existing cube obstructs a goal or path, the task may refuse. IK failure
is distinct from path-planning failure. Read the reported leg and error code;
do not disable collision checks. Restore the cube separately in both scenes
if you restart a world where it was inserted only at runtime.

## Change the task coordinates

Offsets are in WORLD axes from the original measured tool position. Values are
metres. Both lists contain X, Y, Z; the second is not added to the first.
For example, lift 40 mm, then move 20 mm along world Y at that height:

```bash
ros2 run pieas_arm_moveit_config tool_sequence --ros-args -p use_sim_time:=true -p lift:="[0.0, 0.0, 0.04]" -p across:="[0.0, 0.02, 0.04]"
```

Preview is the default. Add `-p execute:=true` to plan and execute the changed
task. Offsets are initially limited to 100 mm in Euclidean distance from the
starting position. Reachability and collision checking can further restrict them.

## Files involved

- `config/pieas_arm.srdf`: defines the same six-joint arm as a serial chain from
  base_link to tool0, so a chain IK plugin can operate on it.
- `config/kinematics.yaml`: loads KDL with `position_only_ik: true`. Tool orientation
  may change. It is not a fixed-downward gripper task.
- `launch/planning.launch.py`: passes kinematics settings to move_group and RViz.
- `scripts/tool_sequence`: commented application logic using FK, collision-aware
  IK, planning and execution services/actions. The saved tool0 location is inherited
  from the original conversion, not claimed to be physically calibrated.
- `scripts/plan_joint_goal`: exposes its existing planning connection for reuse;
  the original small joint-goal command remains available.

Target markers are published on `/tool_targets`; a MarkerArray display is added
to the RViz preset. Green is the lifted target, blue the sideways target, orange
the original position. The actual commanded working frame can be inspected in
RViz by adding a TF display and locating `tool0` relative to the gripper fingers.
The existing converted frame is used without changing mesh or tool offsets.

Offline validation checked chain continuity and joint membership, configuration
syntax, returned-IK joint bounds, Python client initialization, and numerical XYZ
reachability for the default targets at home and small_test. Actual KDL/plugin
loading, collision-valid IK, visualization and execution need the target Jazzy
machine. Neither arbitrary reachable orientation nor collision-free success from
all starting poses is claimed.
