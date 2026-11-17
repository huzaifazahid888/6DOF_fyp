# Development roadmap

[Workspace overview](../README.md) · [Recorded experiments](../EXPERIMENTS.md)

The first four stages established the model, controller connection, motion planning and a sensor-driven sorting baseline. My current focus is understanding and improving that implementation before extending it to other tasks. The separate [orchard package](../src/pieas_orchard/README.md) now explores harvesting; its validation notes track acceptance. The items below remain planned work unless explicitly described as an existing baseline.

## Current priority: two tool modes

The suction implementation needs to remain available while the original gripper gains its own behaviour. The intended interface is one launch selection for suction or original-gripper mode, applied consistently to Gazebo, MoveIt and the sorting application. That selector is not implemented yet.

The original gripper currently has one rigid mesh link. The next steps are to inspect the source mechanism, separate moving components, define the actual finger joints and add control. Pickup targets must then place the object between the fingers rather than put a cup at its top surface. Finger opening, closing force, contact, slip and release need separate tests.

A physical finger-grasp test must lift and hold the object through contact and friction. Any temporary helper attachment should be recorded as assisted grasping, with separate results from physical grasping.

Completion evidence: both tool modes load consistently, opening and closing works for the original gripper, and repeated known-pose picks report grasp and release outcomes before perception is reintroduced.

## Robot-mounted sensing

The factory camera and LiDAR are fixed overhead. The orchard implements a visible mast on the anchored robot base. A moving wrist camera remains future work. Robot-mounted sensing needs mounting geometry, sensor frames and timestamped transforms that follow the arm. The perception code currently uses fixed sensor poses, so mounting changes require processing changes as well as URDF edits.

Completion evidence: projected points agree with the scene at multiple arm poses, the robot does not classify its own body as a part, and occlusion or missing data causes an explicit rejection.

## Broader perception experiments

The current classifier uses a known family of isolated green shapes. Future digital image processing experiments can compare segmentation methods, contour features, depth fitting and robustness to background, lighting and partial occlusion. Different object families need measured classification and pose-error results before use in the task.

Completion evidence: held-out scenes, a confusion matrix, position-error distributions and separate records for rejected observations. The orchard adds a neural pixel classifier trained on synthetic appearance samples. Real orchard detection and grading require a representative image dataset and separate validation.

## Planning and trajectory comparisons

RRTConnect is the baseline. Other planners can be compared using identical start/goal states, collision geometry and time budgets. Planning time, success rate, clearance and path length should be recorded over repeated trials.

Trajectory studies can compare cubic and quintic interpolation, time parameterization and smoothing on collision-checked paths. Rail and rotary tracking errors need separate units. Faster motion is useful only after the limits and model response are understood.

## Mapping, recovery and hardware

Depth/LiDAR obstacle mapping, moving-obstacle response and bounded recovery are later stages. The current LiDAR check is not a full environmental map. The current factory stops on faults and requires a fresh scene; it does not resume a partial batch.

Hardware integration would replace Gazebo's actuator and machine interfaces with motor/encoder, conveyor and gripper drivers. The original STM32/CAN experience applies at that interface. Real calibration, timing, usable travel, payload limits and independent placement verification are still required.

## Recording progress

Each new experiment should identify its revision, model/tool configuration, starting conditions, parameter changes and measured result. The [experiment log](../EXPERIMENTS.md) separates completed observations from proposed work. A demonstration video or successful single run is useful evidence, but does not replace repeatability measurements.

