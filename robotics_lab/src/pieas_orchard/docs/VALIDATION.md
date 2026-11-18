# Orchard validation

## Offline checks

Nineteen CPU tests passed during initial development. They cover three appearance grades, metric sphere fitting, partial and heavy occlusion, missing LiDAR, invalid/flat depth, temporal stability, shared scene geometry, sensor mounts, unchanged arm joint transforms, numerical reachability, unobstructed LiDAR mounting sightlines and a recorded Gazebo RGB-D/LiDAR regression frame, padded twig clearance and fruit extraction clearance.

A separate actual-mesh FCL audit found valid sampled home, park, fruit approach/pick/extraction and basket endpoint configurations. That check does not prove a continuous collision-free path or successful Gazebo execution.

The learned model's held-out synthetic pixel accuracy is recorded in the model card. Analytic RGB-D fixtures exercise perception separately. Neither replaces evaluation on actual Gazebo sensor output.

## Runtime acceptance

The dedicated Orchard Jazzy integration workflow builds the plugin and package, runs CPU tests, launches the rendered RGB-D/LiDAR simulation, requires all three fruit grades and executes three verified placements. It also requires no reported robot-tree contact.

Run 44108116903 passed build and rendered sensor grading. The arm reached the first fruit with 0.31 mm measured-state FK endpoint error and accepted the grasp. Withdrawal was refused because the detected payload overlapped a padded twig in MoveIt. The twig endpoint now leaves space for branch padding and fruit pose-estimation tolerance. A separate geometry check identified an overhanging leaf on a straight vertical withdrawal, so the task now extracts 25 mm toward the open side before lifting. Both segments keep the payload in collision checking.

Run 44257288248 completed verified ripe and unripe placements, then extracted and transferred the damaged fruit. It stopped after the last basket descent because the immediate feedback check still reported joint motion. The task now waits for fresh stationary feedback over 0.2 simulation seconds, with a bounded timeout, before checking the measured endpoint. Stale feedback, contact and payload-loss checks remain active. Run 44360954610 repeated two placements but reached the settling timeout at the damaged-basket descent. Its IK had selected a distant flipped arm configuration. Candidate selection now ranks joint travel before collision checking, and settling failures include actual position/velocity values. Full three-fruit harvesting acceptance remains pending. A successful build, sensor test or sampled geometric check is not a completed orchard demonstration.

Logs are written to `/tmp/pieas_orchard_launch.log`, `/tmp/pieas_orchard_task.log` and `/tmp/pieas_orchard_batch` in CI. Local task results are written under `~/.ros/pieas_orchard/`. The task phase and detections are visible on `/orchard/task_state` and `/orchard/detections`.

## Model boundaries

The tree is a scaled rigid trellis with conservative collision geometry. Leaves are partially occluding geometry; no leaf elasticity is modelled. The scene is intentionally reachable by the existing small arm. The original arm's actuator limits and inertias have not been calibrated to real harvesting loads.

A physics fixed joint holds each fruit on a branch until a checked suction command replaces that stem attachment with a tool attachment. This preserves fruit pose and mass but does not simulate biological detachment mechanics. A released fruit falls under gravity. Simulator position is used only for proximity interlocks and final basket verification, never fruit class or pickup estimation.

The camera and LiDAR are fixed to the robot base mast. This is robot mounting, not a moving wrist camera. Their static world transforms are valid because the robot base is anchored; a mobile base would need timestamped dynamic transforms in perception.

Protected branch/leaf contact is a fault signal. It is not a calibrated force or damage measurement. Hidden internal fruit quality and real ripeness are not observable from this synthetic model. Rejected fruit remains on the tree; the task must not report a completed requested batch when observations are missing.

