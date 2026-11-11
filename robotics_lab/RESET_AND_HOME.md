# World startup and reset

The arm is now assembled into the startup world before Gazebo starts. Previously,
`ros_gz_sim create` inserted it after the empty lab had loaded. A full reset could
therefore restore the initial world without the arm.

`lab.launch.py` now resolves the control URDF, converts it with `gz sdf -p`, and
adds the resulting `pieas_arm` model to a temporary copy of `worlds/lab.sdf`.
Gazebo loads that combined world. ROS receives the same resolved URDF. The source
world and URDF are not rewritten during launch, and temporary files are cleaned
up at shutdown. Always launch through ROS; opening the source `lab.sdf` alone
still opens the environment without the generated arm.

The moving joints remain dynamic. Do not set the entire robot static to keep it
visible. The existing fixed base mount anchors it to the world.

## Working baseline

Joints 3 and 4 use effort 10 Nm and velocity 0.5 rad/s. The user observed holding
at zero under gravity with these values. Raising effort alone from 10 to 100 Nm
with the original 0.034906585 rad/s velocity ceiling did not fix sagging. These
observations do not validate all poses, payloads, or physical motor ratings.

## Verification on Ubuntu

1. Stop the previous ROS launch with Ctrl+C.
2. Build and source the workspace, then run
   `ros2 launch pieas_arm_description lab.launch.py`.
3. Confirm one `pieas_arm` appears and both controllers become active.
4. Pause and press the GUI Reset button. Confirm `pieas_arm` remains in the tree.
5. Resume. Check `ros2 control list_controllers` and one message from
   `/arm_controller/controller_state`. Confirm the simulation clock advances
   and feedback continues. Model presence alone does not prove control works.

Full reset rewinds simulation time and can recreate plugin state. Upstream
reports describe reset problems with gz_ros2_control, including issue #636
(https://github.com/ros-controls/gz_ros2_control/issues/636), reported on a
Humble/Harmonic combination. This does not establish the same failure on this
project's Jazzy installation, but controller recovery must be tested separately.
If Reset crashes or leaves controllers unavailable, retain the log; this change
only addresses initial-world membership, not an upstream plugin lifecycle bug.

Returning home should ultimately be a trajectory to the home joint positions,
without destroying the world or rewinding time. It is distinct from full Reset.

## Validation status

Python syntax and isolated world-assembly checks passed. Tests mocked `gz sdf`
and checked resource paths, preserved environment models, joint limits, and
failure cleanup. No ROS/Gazebo runtime is installed in the editing environment;
actual URDF conversion, startup, and GUI Reset require the Ubuntu test above.
