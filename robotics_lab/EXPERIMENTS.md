# Project progress and experiment log

[Workspace overview](README.md) · [Roadmap](docs/ROADMAP.md) · [Factory validation](src/pieas_sorting/docs/VALIDATION.md)

This log records the progression from model import to sorting. Historical local observations were reported during development; where the exact revision or raw artifact was not saved, that limitation is stated. They should not be read as fresh tests of every later commit.

| Stage | Observation | Evidence and limitation |
| --- | --- | --- |
| Model conversion | Geometry and joint chain recovered from V-REP; FK compared across 101 configurations | Offline conversion checks; not a contact-dynamics test |
| Gazebo control | Controllers active, joint feedback advancing, home held and small joint motion completed | Local terminal feedback and visual observation; no complete hardware calibration |
| Joint sequence | Joint 2 moved to +0.1 rad, held, moved to -0.1 rad and returned home | Reported trajectory success; logged moving error around 0.0017 rad at its sampled peak |
| Full-range sweep | Large direct motions produced link penetration | Sweep retired; joint limits alone did not establish collision-free motion |
| MoveIt | Joint planning, preview and execution exercised; obstacle-scene detours observed | Local experiments; no systematic planner benchmark recorded |
| XYZ tool sequence | Lift, sideways move and return executed | Reported measured-state FK endpoint errors: 0.36, 0.76 and 0.74 mm; same-model estimates |
| Orientation-constrained sequence | Configuration and numerical checks implemented | No runtime result recorded here |
| Suction factory, one object | CI completed sensor checks and a verified placement | [Run 43281803114](https://github.com/huzaifazahid888/6DOF_fyp/actions/runs/43281803114), commit `1ac877c` |
| Suction factory, expanded batch | Cube and rectangle placed, then cylinder retract failed on start-state bounds | [Run 43372619445](https://github.com/huzaifazahid888/6DOF_fyp/actions/runs/43372619445); continuous-joint normalization was added afterward |
| Suction factory, six objects | Local run reported six placements and `FACTORY: COMPLETE` on 13 November 2026 | Reported CSV `batch_20261113_105610.csv`; raw CSV and exact local commit are not stored in this repository |
| Original gripper restoration | Factory launch changed from suction conversion to the original rigid mesh | Sorting still uses suction assumptions; a reported descent failed its measured endpoint check |

## Current boundary

The completed suction batch is a baseline for perception, planning and task sequencing. It does not validate the original finger gripper or the current launch configuration. The planned tool selector and finger actuation are described in the roadmap.

The factory validation notes retain the detailed modelling assumptions and the observed MoveIt shutdown issue. A task result before shutdown and a clean process exit are separate checks.

## Record format for new experiments

| Field | Information to save |
| --- | --- |
| Revision and environment | Commit, local changes, Ubuntu/ROS/Gazebo versions |
| Configuration | Tool, sensor mounts, scene, object family and limits |
| Question | What changes and what stays fixed |
| Reproduction | Launch commands, start state, goals and random seeds if applicable |
| Measurements | Sensor quality, planning outcome, tracking, contact and placement results |
| Outcome | Successes, rejections and failures, including their task phase |
| Artifacts | Logs, CSV, configuration snapshot and optional recording |
| Next step | Change motivated by the evidence |

Large recordings belong outside Git with a stable reference. Short diagnostic logs and result tables can accompany a specific experiment when they make its outcome reproducible.
