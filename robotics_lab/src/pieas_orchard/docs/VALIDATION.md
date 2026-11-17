# Orchard validation

## Offline checks

Fourteen CPU tests passed during initial development. They cover three appearance grades, metric sphere fitting, partial and heavy occlusion, missing LiDAR, invalid/flat depth, temporal stability, shared scene geometry, sensor mounts, unchanged arm joint transforms, numerical reachability unobstructed LiDAR mounting sightlines and a recorded Gazebo RGB-D/LiDAR regression frame.

A separate actual-mesh FCL audit found valid sampled home, park, fruit approach/pick and basket endpoint configurations. That check does not prove a continuous collision-free path or successful Gazebo execution.

The learned model's held-out synthetic pixel accuracy is recorded in the model card. Analytic RGB-D fixtures exercise perception separately. Neither replaces evaluation on actual Gazebo sensor output.

## Runtime acceptance

The dedicated Orchard Jazzy integration workflow builds the plugin and package, runs CPU tests, launches the rendered RGB-D/LiDAR simulation, requires all three fruit grades and executes three verified placements. It also requires no reported robot-tree contact. The initial integration build found an incorrect Gazebo contact-message accessor. That build error was corrected and the next CI build and CPU tests passed. The rendered readiness test then received advancing RGB, depth, LiDAR, joint feedback and backend status, but no accepted fruit detections. That run did not attempt harvesting. Sensor mounting and timestamp synchronization were revised next. Full harvesting acceptance remains pending; a successful build is not a completed orchard demonstration.

Logs are written to `/tmp/pieas_orchard_launch.log`, `/tmp/pieas_orchard_task.log` and `/tmp/pieas_orchard_batch` in CI. Local task results are written under `~/.ros/pieas_orchard/`. The task phase and detections are visible on `/orchard/task_state` and `/orchard/detections`.

## Model boundaries

The tree is a scaled rigid trellis with conservative collision geometry. Leaves are partially occluding geometry; no leaf elasticity is modelled. The scene is intentionally reachable by the existing small arm. The original arm's actuator limits and inertias have not been calibrated to real harvesting loads.

A physics fixed joint holds each fruit on a branch until a checked suction command replaces that stem attachment with a tool attachment. This preserves fruit pose and mass but does not simulate biological detachment mechanics. A released fruit falls under gravity. Simulator position is used only for proximity interlocks and final basket verification, never fruit class or pickup estimation.

The camera and LiDAR are fixed to the robot base mast. This is robot mounting, not a moving wrist camera. Their static world transforms are valid because the robot base is anchored; a mobile base would need timestamped dynamic transforms in perception.

Protected branch/leaf contact is a fault signal. It is not a calibrated force or damage measurement. Hidden internal fruit quality and real ripeness are not observable from this synthetic model. Rejected fruit remains on the tree; the task must not report a completed requested batch when observations are missing.


The second rendered attempt localized apples but missed the damaged class. Gamma-aware appearance training fixes that recorded-frame regression. Synchronization now waits for matching frames without treating normal callback gaps as a stability failure. Full task execution is still subject to the dedicated integration result.
