# Stage 2: Joint control and feedback

[Workspace overview](../../README.md) · [Previous: robot model](../pieas_arm_description/README.md) · [Next: MoveIt planning](../pieas_arm_moveit_config/README.md)

I used this stage to connect a ROS action client to the simulated arm and understand how a timed joint command becomes measured movement. The early experiment moved joint 2 by +0.1 rad, held it, moved to -0.1 rad and returned home. Feedback showed the reference changing during motion and the actual position settling at each target.

## What this established

The client sends a `FollowJointTrajectory` goal to `/arm_controller/follow_joint_trajectory`. The trajectory controller produces commands for Gazebo through `ros2_control`. Joint feedback returns through `/joint_states`, while the action reports progress and completion. This is the control connection reused by MoveIt later.

Joint 1 is a rail displacement in metres. Joints 2–6 use radians. A trajectory contains joint positions and arrival times; it does not by itself describe a collision-free route for the tool.

## Retired full-range sweep

The later experiment moved all six joints through a configured range in 30 simulation seconds. It exposed link penetration and unverified travel limits. That sweep is retired: the current `joint_sequence` executable refuses motion. Its source remains as a record of the action-client work, not a recommended motion command.

The earlier single-joint sequence is project history, not the behaviour of the current executable. Active movement experiments now use [MoveIt](../pieas_arm_moveit_config/README.md), which checks the modelled scene before execution.

## Files

| File | Role |
| --- | --- |
| [joint_sequence.py](pieas_arm_control/joint_sequence.py) | ROS node, trajectory action client, feedback checks and retired-demo guard |
| [sequence.py](pieas_arm_control/sequence.py) | Trajectory construction and timing logic |
| [test_sequence.py](test/test_sequence.py) | Checks for sequence generation and feedback handling |
| [setup.py](setup.py) | Installs the Python package and executable entry point |

## Result and lesson

Small joint motion and return to home were observed successfully. The full-range experiment showed why trajectory timing and joint limits are not enough for manipulation. Collision checking belongs in the planning stage, and source joint ranges still need comparison with the actual mechanism.
