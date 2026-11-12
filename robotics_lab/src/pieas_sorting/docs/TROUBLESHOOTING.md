# Troubleshooting by stage

| Symptom | Meaning and next check |
|---|---|
| `pieas_factory_system` fails to load | Build the new C++ package and source `install/setup.bash`. Inspect compiler output first; check `GZ_SIM_SYSTEM_PLUGIN_PATH`. |
| `No advancing joint feedback` | Gazebo may be paused, controllers inactive, or a second launch is publishing conflicting state. Use one factory launch and check `ros2 control list_controllers`. |
| `check_cell` lacks camera/depth/LiDAR | Check Gazebo sensor/render-engine errors, bridge topic names and graphics support. The GPU LiDAR needs rendering even with the GUI hidden. |
| Camera is blank on remote/headless host | The Ogre2 renderer needs functioning OpenGL/EGL. CI uses software rendering. A GUI window alone does not prove sensors render. |
| `waiting_for_rgb_depth_camera_info_lidar` | One required sensor topic has not arrived. Inspect the named topics in `config/bridge.yaml`. |
| `sensor_timestamp_mismatch` or stale data | Check that every node uses simulation time and that Gazebo is running. Do not mix topics from another world. |
| `uncertain_or_not_stationary` | Inspect `/factory/perception_mask`; the part may not be at the station, may be occluded, moving, merged, an unknown shape or unsupported by LiDAR. |
| Belt stops and no pick starts | Inspect `/factory/detections` and `/factory/task_state`. The code intentionally waits for accepted sensor evidence. |
| `No bounded, upright, collision-free IK` | Check target XYZ, current pose, bin layout and collision geometry. Do not disable self-collision as a workaround. |
| MoveIt planning failure | Endpoint reachability does not guarantee a path. Inspect the MoveIt log and scene, particularly the payload and queued-parts keepout. |
| Suction tolerance failure | Tool may not be close enough, upright, or the part may still move. Inspect measured FK and detection top height. Do not enlarge attachment radius to conceal a frame error. |
| `released object inside intended bin` timeout | The drop did not satisfy placement verification. Inspect the object in Gazebo; a successful trajectory alone does not count as a sort. |
| A task fails while holding an object | The attachment is retained. Stop the launch and begin with a clean scene; automatic arbitrary release is not performed. |
| Fresh scene looks wrong after Reset | Stop the task and relaunch. Task counters, attached objects and planning scene must agree with the reset world. |

A useful diagnostic bundle is the factory launch terminal output, task terminal output, `ros2 control list_controllers`, one `/factory/state` message, one `/factory/detections` message and the batch CSV. The CI runtime test writes `/tmp/pieas_factory_launch.log` and `/tmp/pieas_factory_sort.log` and uploads them with test artifacts.

Do not run the retired full-range sweep in this scene. Motion limits alone do not prevent self-collision.
