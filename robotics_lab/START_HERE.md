# Your PIEAS arm robotics lab

Prepared for Muhammad Huzaifa, 10 November 2026.

The goal is an arm that estimates object poses, plans collision-free motion, executes timed trajectories, checks the outcome, and recovers from failures. This is a multiweek project. At roughly one hour per session, allow 30–50 sessions for the core stages and additional time for learning algorithms. These are planning estimates.

## What is ready

The package in src contains your extracted arm, six position command interfaces, a joint trajectory controller, a launch file, and a Gazebo world with an overhead RGB-D camera and a separate 3D LiDAR. Sensor mounts are experimental placements. They are defined in the world with matching ROS static transforms. Later wrist-mounted sensors belong in the arm URDF with fixed joints.

Offline checks cover XML/YAML syntax, Python syntax, joint names, file references, and the original model's forward kinematics. ROS 2 and Gazebo are unavailable in this environment, so the integrated launch, plugins, sensor topics, TF conventions, and controller behavior require runtime validation on your Ubuntu computer. Treat this package as the starting implementation to test.

Your arm has one prismatic and five rotary joints. RG2 geometry is fixed in the saved configuration. Four arm effort limits are provisional. The source's ±0.5 m slide travel must be narrowed after checking the actual rail. The scene's masses and inertias are carried forward as simulation data. Current position control is suitable for planning integration. Controller robustness, payload response, and PID studies need realistic effort control and actuator dynamics later.

## Tomorrow morning, first hour

1. Spend 10 minutes checking Ubuntu, ROS, and Gazebo.
2. Spend 15 minutes installing dependencies and building.
3. Spend 15 minutes launching and checking the model, controllers, and clock.
4. Spend 10 minutes inspecting sensor messages and TF.
5. Spend 10 minutes commanding a small motion and recording observations.

If dependency setup takes longer, finish that stage first. Move through the completion checks below in order.

### Install and build

Use Ubuntu 24.04 on amd64 or arm64. The script can install ROS 2 Jazzy on a fresh Ubuntu machine.

```bash
git clone https://github.com/huzaifazahid888/6DOF_fyp.git ~/6DOF_fyp
cd ~/6DOF_fyp/robotics_lab
bash scripts/setup_ubuntu.sh --yes --build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pieas_arm_description lab.launch.py
```

For an existing clone, enter its robotics_lab directory and run `git pull --ff-only` before setup. The script locates the workspace from its own location, so the repository can live elsewhere too. Substitute that workspace path in the terminal commands below.

| Option | Behavior |
| --- | --- |
| No options | Install missing prerequisites with package-manager prompts |
| --check | Inspect installed packages, Gazebo version, and cached rosdep dependencies without downloads or sudo |
| --yes | Accept installation prompts |
| --build | Build the workspace after dependencies pass |
| --with-moveit | Also install MoveIt and its Setup Assistant |
| --help | Show usage |

Run as your regular user. Internet access and sudo are needed when installing missing software. Repeat runs skip installed apt packages and refresh rosdep's dependency index. The script configures the official ROS apt source only when Jazzy packages are unavailable, enables Ubuntu Universe as needed during installation, and uses the ROS-packaged Harmonic libraries.

The script sets a UTF-8 locale for its process. It prints environment commands for your terminal. With --check, a missing package or failed prerequisite check returns a nonzero exit status.

If apt fails, inspect the reported package conflict or repository error, correct it, and rerun. Ubuntu's Noble updates repositories may be needed to satisfy dependencies. If graphics fail after setup, inspect your GPU/VM rendering configuration separately.

Validation of the script includes Bash syntax, mocked install and repeat-run paths, read-only mode, dependency failures, and unsupported-system rejection. Installation on a fresh Ubuntu 24.04 machine and Gazebo GUI execution remain to be tested.

### Check before sending a motion

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/6DOF_fyp/robotics_lab
source install/setup.bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
ros2 topic echo /clock --once
gz topic -l
ros2 topic list -t
```

Both controllers should be active. Six position command interfaces should be claimed. The simulation should be running, the base should stay fixed, and all joints should start at zero. If the arm collapses, jumps, or separates, stop this stage and inspect controller/plugin errors and the model's inertial properties.

### First motion

Visually check room for a 1 cm slide displacement. Then command only that displacement while explicitly holding the rotary joints at zero.

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 5}}]}}" \
  --feedback
```

Repeat with all positions zero to return. This direct controller command has no motion planner collision check. Later test each rotary joint individually by 0.02 rad over five seconds, checking clearance first. Stop issuing direct goals once autonomous planning starts.

### Sensor checks

```bash
ros2 topic hz /rgbd/image
ros2 topic hz /rgbd/depth_image
ros2 topic hz /rgbd/points
ros2 topic hz /lidar/points
ros2 run tf2_ros tf2_echo world camera_optical_frame
ros2 run tf2_ros tf2_echo world lidar_link
rviz2 --ros-args -p use_sim_time:=true
```

In RViz set Fixed Frame to world. Add RobotModel using /robot_description, TF, Image, and PointCloud2 displays. Use Best Effort reliability if a sensor publisher requires it. The configured topics are expected Gazebo RGB-D/LiDAR suffixes. Compare gz topic -l with config/bridge.yaml and correct any differences in the installed release.

Verify that clouds align with the visible arm and floor. The camera optical convention is x right, y down, z forward. If a cloud is rotated, inspect the actual message frame and coordinate convention before editing transforms. A frame label alone cannot rotate data. All processing nodes should use simulation time.

## How the system will work

Gazebo produces joint states and sensor measurements. ROS bridges carry sensor data. TF supplies transformations at the measurement timestamp. Perception estimates object pose and scene geometry. MoveIt's planning scene maintains robot geometry and obstacles. A planner finds a path in joint space. Time parameterization adds velocity, acceleration, and timing. The trajectory controller executes it. A task state machine checks success and chooses the next action.

Each experiment should change one factor, use a fixed list of scene seeds, and save configuration, software versions, input poses, rosbag data, and results. Record failed runs as well as successful runs. Keep training/tuning scenes separate from evaluation scenes.

## Stage 1 — Model and joint verification, 2–3 sessions

Read the supplied URDF and identify each origin, axis, limit, visual mesh, collision mesh, and inertial element. Compare the simulated directions with your bachelor project. Check tool0 against the grasp point you actually want.

Test one joint at a time with small motions. Measure actual slide travel and define usable joint limits. Inspect self-collision at multiple configurations. Replace unnecessarily detailed collision meshes with validated convex parts or primitives if simulation is slow. Preserve visual geometry.

Finish when all six joints move independently, remain stable for 60 simulated seconds at rest, and return consistently. Suggested initial thresholds are 1 mm slide error and 0.5 degree rotary error after settling. These are experiment criteria to revise from the measured baseline.

## Stage 2 — MoveIt configuration and inverse kinematics, 3–4 sessions

Install ros-jazzy-moveit and ros-jazzy-moveit-setup-assistant. Import urdf/pieas_6dof_arm.urdf into MoveIt Setup Assistant. Create planning group arm as the chain base_link to tool0. Use the explicit world joint already in the model. Review the generated self-collision matrix, particularly contacts between neighboring meshes. Define a named zero pose only after confirming it is collision-free.

Configure arm_controller as a FollowJointTrajectory controller with all six joints and action namespace follow_joint_trajectory. Generate an independent pieas_arm_moveit_config package. Run one robot_state_publisher and one Gazebo controller manager. Generated demo launch files may start mock hardware, so adapt the execution launch to connect to the existing Gazebo controller.

Start with KDL numerical IK if available for the chain. First generate targets using forward kinematics from collision-free joint states. Ask IK to recover a solution for each target. Then test unreachable poses and joint-limit cases. Six DOF does not imply every Cartesian orientation is reachable.

Finish with 20 known-reachable poses, documented failures, and pose errors below chosen tolerances, initially 5 mm and 2 degrees.

## Stage 3 — Path planning, 3–4 sessions

A path defines where the arm moves. Start with two joint configurations and a static box between them. Add the obstacle both to Gazebo and to the MoveIt planning scene. An object visible in Gazebo is not automatically known to MoveIt.

Run RRTConnect as the baseline. Compare PRM and RRTstar using the same 30 start-goal pairs, equal planning time limits, identical collision checking settings, and repeat trials for stochastic variability. Report success rate, planning time, minimum clearance, and a normalized joint-space path length. Scale prismatic and rotary distances by chosen joint ranges before combining them.

For a focused graph-search exercise, lock four joints and discretize two rotary joints. Mark configurations occupied using whole-arm collision checks. Implement Dijkstra and A* with a valid heuristic and compare expanded nodes and path cost. Then return to the full six-joint problem. This connects your graph-search study to configuration-space obstacles.

Finish when the planned swept arm clears the obstacle and goal execution agrees with the planned result.

## Stage 4 — Trajectory generation, 3–4 sessions

A trajectory defines when the robot reaches positions along a path. Compare your bachelor project's cubic interpolation against quintic interpolation on the same simple collision-free move. Use analytical position, velocity, acceleration, and jerk expressions where possible.

For cubic rest-to-rest interpolation, s=3u²−2u³. For quintic rest-to-rest interpolation, s=10u³−15u⁴+6u⁵, with u=t/T. Set q=q_start+s(q_goal−q_start). Pick a shared T that satisfies every joint's limits. Check sampled swept geometry. Smooth joint interpolation does not by itself avoid obstacles.

Then examine the installed MoveIt time parameterization and Ruckig smoothing. Define explicit acceleration and jerk limits. This arm mixes linear and angular joints, so inspect mixed-joint compatibility and tolerance semantics in the installed algorithm. Record failures instead of silently accepting an unparameterized path. Recheck collisions if postprocessing changes the path.

Measure duration, peak velocity/acceleration/jerk, and tracking error for each joint separately. Position-interface simulation does not establish realistic motor torque or payload performance.

## Stage 5 — RGB-D perception, 3–5 sessions

Start with a colored cube at a verified reachable position. Add it through a reproducible scene setup. Use simulator pose only as evaluation ground truth. Implement HSV segmentation, reject tiny regions, take a robust depth estimate from valid object pixels, and project using CameraInfo intrinsics.

For pixel (u,v) and optical-axis depth Z, compute X=(u−cx)Z/fx and Y=(v−cy)Z/fy. Transform the resulting point from camera_optical_frame to world using TF at the image timestamp. Confirm the depth encoding and units first.

Compare estimated and true center at 20 locations. The visible surface centroid differs from the cube center, so include the object's geometric offset or fit a box to its point cloud. Repeat with changed lighting, background colors, and partial occlusion.

Finish with a quantified pose-error distribution and explicit rejection of invalid or missing depth. Suggested easy-scene target is median translation error below 10 mm.

## Stage 6 — 3D obstacle mapping and LiDAR, 3–4 sessions

Feed the depth-camera point cloud to MoveIt's PointCloudOctomapUpdater. Set its planning frame consistently and inspect the filtered cloud. Tune self-filter padding so the moving robot is removed without erasing nearby objects. Use measured sensor geometry, timestamps, and suitable range limits.

Compare voxel sizes such as 5, 10, and 20 mm. Record map update latency, missed thin obstacles, false occupancy, and planning time. Add objects that were never manually entered into MoveIt's planning scene.

Repeat using 3D LiDAR. Its 32 vertical layers and range noise are initial experimental settings. Test thin obstacles, near-range gaps, and objects outside its field of view. A 2D laser scan can support a restricted-plane experiment but cannot cover the arm's full 3D workspace.

Compare depth only, LiDAR only, and combined observations on the same scenes. Transform observations into one frame, handle time alignment, crop the workspace, and avoid treating correlated observations as independent certainty. Investigate stale occupied voxels when objects move.

Finish when unseen obstacles are represented and cause a planned detour, with missing observations detected.

## Stage 7 — Gripper actuation and physical grasping, 4–7 sessions

This is a required implementation stage. The existing RG2 is a rigid snapshot. Its source model contains linked joints and loop closures. URDF represents a tree, so a faithful RG2 requires additional Gazebo constraints or a validated mechanism model.

For the first grasp study, use an explicitly documented simplified parallel-jaw model with two opposing sliding fingers and a controllable opening. Preserve the original RG2 assets for later mechanical reconstruction. Define collision geometry, finger mass, friction, travel, and a gripper controller. Measure object contact, lift, and slip before adding perception.

Run known-pose pick-and-place on a cube. Use MoveIt Task Constructor or a small explicit state machine for approach, open, descend, close, lift, transfer, place, open, and retreat. In MoveIt, attach the grasped collision object for planning and allow only intended finger contact. That planning attachment alone creates no physical bond in Gazebo.

If a simulator attachment joint is temporarily used, label those experiments as assisted grasping. Separate them from friction/contact-based success rates. A physical success requires lifting and holding the object without a helper attachment.

Finish with repeated known-pose picks and a failure breakdown by approach, contact, lift, transfer, and release.

## Stage 8 — Vision-guided grasping, 3–5 sessions

Use perceived object pose to generate multiple feasible grasp candidates. Reject candidates using IK, joint limits, collision checks, and available finger opening. Rank by clearance and approach length. Start with a cube of known dimensions.

Compare perfect ground truth, noisy ground truth, and camera-derived pose on the same 30 placements. Log perception failures separately from planning and grasp failures. Add checks for object motion during lift and empty closure. Retry with a new observation and an alternative grasp, with a bounded retry count.

Finish with end-to-end results and clear success/failure detection.

## Stage 9 — Dynamic obstacles and recovery, 3–5 sessions

Introduce a moving box on a repeatable trajectory. Begin with stop, update scene, and replan. Define maximum acceptable sensor age and planning-scene age. Cancel motion when observations become stale or the monitored collision margin is violated.

A global plan does not automatically react to a moving obstacle. Develop a monitored execution loop, then evaluate MoveIt Servo or a local method for restricted tasks. Check controller cancellation, deceleration, delay, and replanning behavior in simulation.

Experiment with 5/10/20 Hz updates, sensor delays, and short dropouts. Measure detection-to-stop time, minimum clearance, false stops, completion time, and failed recovery. Add a retreat strategy only after verifying that the retreat itself is feasible.

## Stage 10 — Additional sensors and estimation, optional

| Sensor | Experiment | Integration needed |
| --- | --- | --- |
| Wrist force/torque | Detect unexpected contact and compare payloads | Force-torque joint sensor, bridge, timestamped frame transform, bias removal |
| Finger contacts | Confirm grasp and detect asymmetric contact | Contact sensors on finger collisions, contact system, decision logic |
| Wrist RGB-D | Observe regions hidden from the overhead camera | URDF fixed mount, optical frame, dynamic TF and self-filtering |
| IMU | Compare predicted wrist acceleration with measurements | Sensor link, IMU system, gravity/frame treatment and a defined estimator |
| Joint noise/delay | Estimate state under imperfect measurements | Explicit measurement corruption, separate true and measured state channels |

Add a sensor when an experiment needs its measurement. For a fixed arm, joint states and vision are the first priorities. LiDAR contributes obstacle geometry. An IMU becomes useful when studying vibration, acceleration, or estimation errors.

## Stage 11 — Learning-based methods, optional

Establish a classical baseline first. Replace color segmentation with a detector or segmentation model and measure generalization on held-out shapes, lighting, and backgrounds. Then consider learning grasp rankings from successful and failed candidates.

For reinforcement learning, start with reaching in an obstacle-free simulation. Define observations, bounded actions, reset conditions, reward, and termination. Split training and evaluation scenes. Compare against the classical planner on matched reachable targets. Domain randomization can test variation in appearance, noise, friction, and mass. A learning method must improve a measured outcome to justify added complexity.

## Stage 12 — Final demonstration and research report

Build a repeatable cluttered sorting task. The arm should identify a target, plan an approach, execute a timed motion, grasp, verify lift, place, and recover from a failed attempt. Evaluate at least 50 held-out trials across easy, occluded, cluttered, and delayed-sensor cases.

Save success rate, task time, placement error, minimum obstacle clearance, recovery success, and compute load. Report performance by scene difficulty and show failure examples. A short video, reproducible launch commands, and a results table will make a useful portfolio project.

## Troubleshooting order

| Symptom | First check |
| --- | --- |
| Missing meshes | Build/source workspace and inspect GZ_SIM_RESOURCE_PATH |
| Controller never appears | gz_ros2_control installation, plugin load log, YAML path |
| Controller timeout | Simulation unpaused, /clock advancing, controller manager service present |
| RGB-D/LiDAR topics absent | Gazebo Sensors system, rendering support, actual gz topic names |
| Cloud rotated or displaced | Message frame, optical transform, static mount pose |
| Robot appears in obstacle map | Robot state timestamp, collision geometry, self-filter padding |
| Planner finds no path | Start-state collisions, IK reachability, usable limits, scene geometry |
| Slow execution | Joint velocity limits, especially the original 2 deg/s limits on joints 2–5 |
| Object falls after planned attach | Physical gripper contact or Gazebo attachment implementation |
| Frequent stale-state errors | Simulation time, bridge direction, TF timestamps, update rate |

## Reading, in order

Read only the section needed for the current stage, then implement one experiment.

1. Gazebo and ROS version pairing — https://gazebosim.org/docs/harmonic/ros_installation/
2. ROS 2 control in Gazebo — https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
3. Joint trajectory controller — https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
4. MoveIt Setup Assistant — https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html
5. Time parameterization — https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html
6. Gazebo sensors — https://gazebosim.org/docs/harmonic/sensors/
7. MoveIt perception pipeline — https://moveit.picknik.ai/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.html
8. MoveIt Task Constructor — https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html

The linked MoveIt main documentation tracks Rolling. Use its explanations, then verify API names and available packages against your installed Jazzy release. Pin versions when a working milestone is reached.
