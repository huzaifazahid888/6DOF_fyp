# Six-joint simulation demonstration

This package sends one `FollowJointTrajectory` action goal to the existing
`arm_controller`. The controller executes it in Gazebo and returns feedback.
The same `joint_sequence` executable now exercises all six joints.

## Run

Start the lab normally, leave Gazebo running and the robot at its zero/home pose.
In another sourced terminal:

```bash
cd ~/6DOF_fyp/robotics_lab
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pieas_arm_control
source install/setup.bash
ros2 run pieas_arm_control joint_sequence --ros-args -p use_sim_time:=true
```

The demonstration takes **176 simulation seconds**. A slower simulation takes
longer in real time. The node prints each stage and target/actual/error for all
six joints. Joint 1 is measured in metres; joints 2–6 are measured in radians.

## What moves and why

The first 120 seconds test each joint separately, returning home between tests.
Each test has 8 seconds outward motion, 2 seconds holding, 8 seconds returning,
and 2 seconds holding at home. This makes each axis easy to identify.

| Joint | Test displacement | Unit |
|---|---:|---|
| joint_1 (rail) | +0.02 | metres (20 mm) |
| joint_2 | +0.10 | radians |
| joint_3 | -0.10 | radians |
| joint_4 | -0.10 | radians |
| joint_5 | -0.10 | radians |
| joint_6 | +0.15 | radians |

The remaining 56 seconds move all joints together between two arm poses while
the rail moves between +20 mm and -20 mm, then return to zero/home. The tool tip
moves with the links; these are joint-space trajectories, not a prescribed
Cartesian line. The RG2 fingers remain rigid because the model does not yet
provide independently controlled finger joints.

These small excursions do not represent full mechanical travel. The converted
model still needs collision and physical-limit validation before larger motions.
This demo does not run a collision planner. Observe the first run in Gazebo.

## Understand the code

`joint_sequence.py` is the ROS action client. Its comments explain the execution
order: `main()` creates the node, `__init__()` establishes communications, and
`run()` checks feedback and home, builds and sends a goal, then waits for its
result. A goal being accepted is different from successful execution.

`sequence.py` is plain Python data and helpers. `build_waypoints()` converts
segment durations into cumulative arrival times. Each waypoint contains all six
absolute positions and a stage description. Changing this file changes the
experiment; it does not add a new ROS controller.

`make_goal()` gives every waypoint zero endpoint velocity. The controller can
therefore interpolate a smooth rest-to-rest cubic segment between positions.
Repeated positions with a later timestamp create a hold. All six joints share
the waypoint time, so they arrive together even when they travel different distances.
This is trajectory execution with manually chosen waypoints, not automatic path planning.

Feedback prints desired minus actual position. Final success checks the action
status and controller result, with final position tolerances of 2 mm for the
rail and 0.01 rad for rotary joints. Final success alone does not certify every
intermediate position or collision clearance.

## Starting, stopping and failures

The client requires fresh advancing joint feedback, finite values, and all joints
near home and stopped. It refuses to start otherwise; it does not automatically
move an unknown pose home. Ctrl+C requests cancellation of this goal and waits
briefly for confirmation. Cancellation does not return home. If cancellation is
not confirmed, pause Gazebo. Do not reset the world during an active goal.

There is a wall-time timeout of five times the planned duration plus 30 seconds.
It remains effective when Gazebo is paused. After interruption, restore a valid
home state before rerunning.

## Validation

`test/test_sequence.py` checks feedback validation, individual-joint isolation,
coverage of all six joints, final home, and cubic peak speeds against the URDF
velocity limits. These are offline checks, not evidence of a completed Gazebo run.
Run with `colcon test --packages-select pieas_arm_control` in the ROS workspace.
