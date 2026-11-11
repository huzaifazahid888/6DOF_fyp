# Thirty-second configured-range simulation test

The `joint_sequence` action client now commands all six joints simultaneously.
This replaces the longer individual-joint demonstration.

| Simulation time | Target |
|---|---|
| 0 s | Start at zero/home |
| 4 s | Halfway to upper targets |
| 8 s | Rail +0.5 m; rotary joints +180 degrees |
| 11 s | Halfway back |
| 15 s | Home |
| 19 s | Halfway to lower targets |
| 22 s | Rail -0.5 m; rotary joints -180 degrees |
| 26 s | Halfway back |
| 30 s | Home |

Joint 6 is continuous: +/-180 degrees are chosen demonstration targets, not
its maximum limits. The tool follows the arm. Gripper fingers remain fixed.
The trajectory uses joint coordinates, not a straight Cartesian tool path.

## Changed simulation speeds

Both description URDF files now use 0.15 m/s for the rail and 1.0 rad/s
(about 57.3 degrees/s) for every rotary joint. Effort limits are unchanged.
These speeds are simulation assumptions, not measured actuator capabilities.
The largest planned rest-to-rest cubic peak speeds are 0.125 m/s and about
0.7854 rad/s, below the new limits. Each waypoint has zero endpoint velocity.

The configured position limits have not been mechanically validated. In
particular, +/-0.5 m rail travel may exceed the visible rail geometry, and
large arm rotations can contact the base or other links. This is a range test,
not a validated collision-free demonstration. Do not disable collisions to
interpret a completed sweep as proof of real reachability.

## Pull, build and run

Stop the running lab before rebuilding, then:

```bash
cd ~/6DOF_fyp
git pull --ff-only
cd robotics_lab
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pieas_arm_description pieas_arm_control
source install/setup.bash
```

Restart the lab with your normal launch command so Gazebo loads the changed
URDF speed limits. In another terminal, with Gazebo running and the arm home:

```bash
source /opt/ros/jazzy/setup.bash
source ~/6DOF_fyp/robotics_lab/install/setup.bash
ros2 run pieas_arm_control joint_sequence --ros-args -p use_sim_time:=true
```

Thirty seconds means simulation time; real elapsed time depends on simulation
speed. Allow up to three additional simulation seconds for final settling.
The client has a 180-second wall-time timeout for paused or slow simulation.

## Understand and observe

`sequence.py` defines absolute positions and cumulative arrival times.
`joint_sequence.py` creates the ROS action client, checks fresh feedback and
home, sends the trajectory, prints each stage and target/actual/error for all
joints, and checks the controller result. Both files contain explanatory comments.

Path position-error tolerances are 0.03 m for the rail and 0.15 rad for rotary
joints. Excessive tracking error aborts the goal instead of continuing the
remaining sweep. These tolerances are not a collision detector. Final tolerances
remain 0.002 m and 0.01 rad. If it aborts, inspect the feedback and contact before
changing tolerances. Ctrl+C requests cancellation; pause Gazebo if cancellation
is not confirmed. Restore home before rerunning; do not reset during a goal.

Offline tests check feedback validation, finite positions, URDF position and
cubic peak velocity limits, both extreme targets, wrist segment direction, and
final home at 30 seconds. Gazebo execution and collision clearance still require
observation on the simulation machine.
