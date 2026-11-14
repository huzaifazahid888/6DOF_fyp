# Stage 3: MoveIt planning and tool movement

[Workspace overview](../../README.md) · [Previous: joint control](../pieas_arm_control/README.md) · [Next: factory sorting](../pieas_sorting/README.md)

I added MoveIt after the direct joint sweep exposed collisions. The aim was to express a target, check whether the robot could reach it without intersecting the modelled scene, and execute a timed trajectory through the existing arm controller.

Gazebo handles simulated dynamics and contact. MoveIt handles the planning model and motion planning. RViz displays measured state, target poses and trajectory previews. The same six joints and trajectory controller connect these parts.

## Experiments in this package

| Experiment | Purpose | Implementation |
| --- | --- | --- |
| Small joint goal | Validate the measured start state, plan a modest joint change, preview and execute | [plan_joint_goal](scripts/plan_joint_goal) |
| Obstacle avoidance | Represent a box in the planning scene and observe a detour or planning rejection | RViz MotionPlanning panel and the planning scene |
| Tool XYZ sequence | Request a lift, sideways move and return; let IK find the joints | [tool_sequence](scripts/tool_sequence) |
| Orientation constraint | Preserve the initial tool orientation while planning the task | `keep_orientation` option in `tool_sequence` |

A box added in RViz is not automatically created in Gazebo. Likewise, an object visible in Gazebo is not automatically included in MoveIt's scene. The factory stage later generates shared static geometry to reduce this mismatch.

## Launch and first preview

After the [workspace setup](../../START_HERE.md):

```bash
ros2 launch pieas_arm_moveit_config planning.launch.py
```

This includes Gazebo, MoveIt, RViz and the controllers. In another sourced terminal:

```bash
ros2 run pieas_arm_moveit_config plan_joint_goal --ros-args -p use_sim_time:=true
```

The default is planning only. Execution uses the same command with `-p execute:=true` and plans again from the current state. The [experiment guide](docs/EXPERIMENTS.md) contains the full commands, home target, XYZ offsets, RViz checks and orientation mode.

## How the tool task works

`tool_sequence` reads the current joints, asks for forward kinematics, and constructs tool targets relative to the starting position. KDL inverse kinematics provides candidate joint positions. MoveIt checks validity and plans paths; time parameterization sets the trajectory timing. The controller executes the accepted trajectories, then measured joint feedback is converted through FK to check the final tool position.

The default task plans all three legs before executing them. Its connecting paths are joint-space plans, not guaranteed straight Cartesian lines. Position-only IK leaves tool orientation free. The optional orientation mode adds a path constraint and has a more restricted reachable set.

## Configuration map

| File | Purpose |
| --- | --- |
| [pieas_arm.srdf](config/pieas_arm.srdf) | Planning groups, named states and adjacent collision exclusions |
| [kinematics.yaml](config/kinematics.yaml) | KDL solver settings for position and full-pose groups |
| [ompl_planning.yaml](config/ompl_planning.yaml) | Planner and request settings |
| [joint_limits.yaml](config/joint_limits.yaml) | Planning velocity and acceleration limits |
| [moveit_controllers.yaml](config/moveit_controllers.yaml) | Connection to the existing trajectory action |
| [planning.rviz](config/planning.rviz) | Saved displays and planning panel |
| [planning.launch.py](launch/planning.launch.py) | Starts the integrated system |
| [test_configuration.py](test/test_configuration.py) | Configuration and application checks |

## Observed results and limits

Small joint goals, obstacle-scene experiments and the default XYZ task were exercised locally. One executed XYZ sequence reported endpoint errors of 0.36, 0.76 and 0.74 mm. These values come from measured joints and the same robot model used for planning; they are not independent physical accuracy measurements.

Orientation-mode checks recorded so far are offline configuration and numerical checks. That mode still needs its own runtime results. The original gripper fingers remain fixed throughout these arm-motion experiments.

[Detailed experiments](docs/EXPERIMENTS.md) · [Project results](../../EXPERIMENTS.md).
