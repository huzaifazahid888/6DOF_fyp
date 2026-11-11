# Lesson 1: program the tested joint-2 sequence

This package sends the same trajectory already tested through the terminal:
+0.1 rad at 8 s, hold until 11 s, -0.1 rad at 27 s, hold until 30 s,
and zero at 38 s. Only joint 2 changes its target. It is a simulation lesson,
not a collision-checked planner or a hardware-ready control system.

## Get and build

From the repository root, use `git pull --ff-only`. If Git reports conflicting
local edits, inspect them; do not discard them to force an update.

```bash
cd ~/6DOF_fyp/robotics_lab
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pieas_arm_control
source install/setup.bash
```

Keep the existing Gazebo lab launch running in another terminal. If it is not
running, start `ros2 launch pieas_arm_description lab.launch.py` separately.
Resume simulation and start from home with no other motion sender active.
The following command sends a motion goal immediately after its checks pass:

```bash
ros2 run pieas_arm_control joint_sequence --ros-args -p use_sim_time:=true
```

It runs once and exits. Do not press world Reset while it executes. Ctrl+C
requests cancellation of this client's goal; it does not reset or return home.
If cancellation cannot be confirmed, pause Gazebo. The client waits at most
180 wall-clock seconds for execution, even though the trajectory is 38 seconds
of simulation time. A very slow or paused simulation can hit that timeout.

## Files and responsibilities

- `package.xml`: ROS dependencies and package metadata.
- `setup.py`: Python installation and the `joint_sequence` executable entry point.
- `setup.cfg`: puts the executable where `ros2 run` searches.
- `resource/pieas_arm_control`: marker used by the ROS package index.
- `pieas_arm_control/__init__.py`: identifies the Python package.
- `sequence.py`: joint order, waypoint values and a home-state check.
- `joint_sequence.py`: ROS node, messages, action communication and feedback.

## Read the program in execution order

### 1. Imports: where the types come from

`time` is Python's standard library; `monotonic()` measures elapsed wall time
without jumping when simulation time resets. `rclpy` is the ROS 2 Python client
library. Its `Node` supplies subscriptions and logging, `ActionClient` handles
the goal protocol, and `spin_once` dispatches callbacks. The sensor-data QoS
accepts best-effort feedback publishers. `SignalHandlerOptions.NO` leaves
Ctrl+C as a Python interruption so cancellation can run before ROS shuts down.

`FollowJointTrajectory` and `JointTolerance` come from `control_msgs`.
`JointTrajectoryPoint` comes from `trajectory_msgs`; `JointState` comes from
`sensor_msgs`; `Duration` comes from `builtin_interfaces`; `GoalStatus` comes
from `action_msgs`. These are generated ROS interface classes, not classes
we invented. `JOINTS`, `WAYPOINTS` and `home_error` are our own definitions.

### 2. main creates the ROS participant

`rclpy.init` initializes ROS communication. `JointSequence()` constructs our
node, and `node.run()` performs one experiment. `self` in class methods refers
to that particular node object, much like `this` in C++.

The constructor calls the parent `Node` constructor with the name
`pieas_joint_sequence`. It creates an action client for
`/arm_controller/follow_joint_trajectory` and a subscription to `/joint_states`.
The action name and type must match the existing controller's server.
Constructing the client does not move the robot.

Members such as `self.goal_handle` retain information across method calls;
local variables such as `point` exist within a method call. Python manages
object references here rather than requiring C-style address operators.

### 3. run waits for readiness

The action server must be discoverable within ten seconds. Then `spin_once`
allows incoming joint messages to call `read_state`. That callback stores the
message, compares its timestamp with the previous timestamp, and records when
it arrived. Two advancing timestamps help distinguish a live simulation from
an old or paused sample.

`home_error` checks all six joints by name, not assumed incoming array order.
It rejects missing, duplicate, non-finite or incomplete data. Home tolerances
are 0.002 m for joint 1 and 0.01 rad for rotary joints; stopped-speed checks use
0.002 m/s and 0.01 rad/s respectively. These are lesson checks, not hardware
safety limits. The program refuses rather than automatically moving home from
an unknown pose.

### 4. make_goal constructs the message

`FollowJointTrajectory.Goal()` creates the request object. Its `trajectory`
contains joint names and a list of points. Each iteration creates a NEW point,
sets all six positions, zero endpoint velocities, and a cumulative duration,
then appends it to the list. Reusing and mutating one point object would be a
mistake because Python lists retain references to objects.

A zero default header timestamp means start when the controller receives the
trajectory. Zero endpoint velocities mean stop at those waypoints, not stay
stationary for the whole motion. The controller generates intermediate
references. Joint 1 positions are metres; joints 2-6 are radians.

The final goal tolerances make endpoint success meaningful: 0.002 m for the
slide and 0.01 rad for each rotary joint, with three extra simulation seconds
allowed to settle. We do not set custom path tolerances in this lesson;
tracking throughout motion is observed, not fully certified by this result.

### 5. send_goal_async sends, but does not wait for completion

The returned future represents the pending acceptance/rejection response.
A future is an object whose result becomes available later. `wait_for` keeps
calling `spin_once`, so ROS can process subscription, action-response and
feedback callbacks while we wait. It is called outside callbacks, avoiding a
callback that blocks its own executor.

Once acceptance arrives, `goal_handle.accepted` tells us whether execution was
accepted. It does not mean the arm finished moving. `get_result_async()` returns
a second future for eventual completion.

### 6. feedback reports progress

Passing `feedback_callback=self.feedback` registers the function to call for
action feedback. `self.feedback` passes the bound method; `self.feedback()`
would call it immediately, which is not what registration needs.

The callback reads the message's `feedback`, finds joint 2 by name, and prints
its desired position, actual position and their difference. Output is limited
to once per wall-clock second so the terminal remains readable. Feedback fields
here are `desired` and `actual`; the separate controller-state topic uses
`reference` and `feedback`. Message field names depend on the interface type.

### 7. completion and cleanup

The program requires BOTH action status SUCCEEDED and controller error code
SUCCESSFUL. Rejection, abort and timeout produce a failure exit code rather
than a success message. Ctrl+C or an error while a goal is pending triggers a
bounded cancellation attempt. Cancellation confirmation is separate from
assuming the robot stopped; inspect it or pause Gazebo when uncertain.

Finally, `destroy_node` releases the node and `rclpy.shutdown` ends ROS
communication for this process. Gazebo and the controller remain running.
The successful trajectory ends at home, so it leaves the target at zero.

## What to observe

Watch both pauses and the direction change. Read the printed desired versus
actual angles during movement. At completion, inspect one `/joint_states`
message if needed. The action result confirms only the configured criteria;
it does not demonstrate collision avoidance or hardware realism.

## Validation and reference

Python syntax, package metadata and offline tests of home checks and the
trajectory's rest-to-rest cubic peak speed were checked. The peak speed of the
moving segments is 0.01875 rad/s, below joint 2's 0.034906585 rad/s ceiling.
This environment has no ROS/Gazebo, so the new Python client has NOT yet been
executed against a live controller. The terminal version of the sequence was
confirmed by the user.

ROS 2 Jazzy action-client tutorial source:
https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.rst

Trajectory controller:
https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
