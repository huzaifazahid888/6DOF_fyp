# Orchard validation

## Offline checks

Nineteen CPU tests passed. They cover three appearance grades, metric sphere fitting, partial and heavy occlusion, missing LiDAR, invalid/flat depth, temporal stability, shared scene geometry, sensor mounts, unchanged arm joint transforms, numerical reachability, unobstructed LiDAR mounting sightlines and a recorded Gazebo RGB-D/LiDAR regression frame, padded twig clearance and fruit extraction clearance, feedback settling and IK candidate ranking.

A separate actual-mesh FCL audit found valid sampled home, park, fruit approach/pick/extraction and basket endpoint configurations. That check does not prove a continuous collision-free path or successful Gazebo execution.

The learned model's held-out synthetic pixel accuracy is recorded in the model card. Analytic RGB-D fixtures exercise perception separately. Neither replaces evaluation on actual Gazebo sensor output.

## Runtime acceptance

The first complete three-fruit baseline passed in [Orchard Jazzy integration run 44363924055](https://github.com/huzaifazahid888/6DOF_fyp/actions/runs/44363924055), using code commit `adfd674d3e4e2848a37470ebaafce6ca3fb8507e`.

| Check | Observed result |
| --- | --- |
| Build and CPU tests | Passed |
| Rendered RGB-D and LiDAR | Advancing sensor data; all three grades detected |
| Ripe fruit | Verified in ripe basket |
| Unripe fruit | Verified in unripe basket |
| Visibly damaged fruit | Verified in damaged basket |
| Protected tree contact | None reported during the batch |
| Final task state | `ORCHARD: COMPLETE` |

This is a successful configured scenario, not a reliability estimate across arbitrary trees or random placements. The initial scene exposes most of each fruit surface. Separate analytic fixtures test partial occlusion and rejection of heavily occluded observations; this run does not establish dense-canopy harvesting performance.

A subsequent local run on 19 November 2026 also reached `ORCHARD: COMPLETE`. Its terminal log recorded 25 measured-state FK endpoint checks, ranging from 0.13 to 0.54 mm with a mean of 0.318 mm, below the configured 3 mm tolerance. Log timestamps span approximately 484 seconds (8 minutes 4 seconds). These are model-based tool-position errors calculated from joint feedback, not independently measured physical fruit-placement accuracy. The local CSV is named `batch_20261119_113614.csv`.

The bins need no geometry change for this three-fruit baseline. Each release is followed by a check that the fruit has settled inside the selected basket. Larger batches, different fruit sizes and stacking need separate capacity and placement validation.

## Changes made during validation

The first grasp exposed an overlap between a detected fruit and padded twig geometry. The twig endpoint now leaves space for branch padding and pose-estimation tolerance. Withdrawal first extracts 25 mm toward the open side, then lifts, keeping the carried fruit in collision checking.

Earlier trials completed two placements but stopped at the damaged-basket descent. Waiting for fresh stationary feedback alone did not resolve that failure. Ranking IK candidates by joint travel before collision checking avoided the distant arm configuration selected by the first-solution approach. The final successful run includes both candidate ranking and the bounded stationary-feedback check. Collision checks and the 3 mm endpoint tolerance remain enabled.

Logs are written to `/tmp/pieas_orchard_launch.log`, `/tmp/pieas_orchard_task.log` and `/tmp/pieas_orchard_batch` in CI. Local task results are written under `~/.ros/pieas_orchard/`. Task phase and detections are visible on `/orchard/task_state` and `/orchard/detections`.

## Model boundaries

The tree is a scaled rigid trellis with conservative collision geometry. Leaves are partially occluding geometry; no leaf elasticity is modelled. The scene is intentionally reachable by the existing small arm. The original arm's actuator limits and inertias have not been calibrated to real harvesting loads.

A physics fixed joint holds each fruit on a branch until a checked suction command replaces that stem attachment with a tool attachment. This preserves fruit pose and mass but does not simulate biological detachment mechanics. A released fruit falls under gravity. Simulator position is used only for proximity interlocks and final basket verification, never fruit class or pickup estimation.

The camera and LiDAR are fixed to the robot base mast. This is robot mounting, not a moving wrist camera. Their static world transforms are valid because the robot base is anchored; a mobile base would need timestamped dynamic transforms in perception.

Protected branch/leaf contact is a fault signal. It is not a calibrated force or damage measurement. Hidden internal fruit quality and real ripeness are not observable from this synthetic model. Rejected fruit remains on the tree; the task must not report a completed requested batch when observations are missing.

