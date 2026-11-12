# Sensor-driven conveyor sorting cell

This package extends the PIEAS arm into an indexed conveyor workcell: RGBD camera + LiDAR → shape classification → bounded IK → MoveIt planning → suction attachment → bin placement. It sorts **six parts, two each of cube, rectangular block and cylinder**, into three bins. There are no hidden class labels in the perception input.

**Read before running:** [the code and concept walkthrough](docs/WALKTHROUGH.md), [validation and limits](docs/VALIDATION.md), and [troubleshooting](docs/TROUBLESHOOTING.md).

## What changes and what stays available

Your existing `planning.launch.py`, original RG2 model and basic movement lessons remain available. The factory has its own launch and configuration. Its generated URDF replaces only the rigid RG2 geometry with a **simulation suction surrogate**. The six arm/rail joints, joint frames, controllers and `tool0` transform remain unchanged. Original source URDF files are not overwritten.

The conveyor stops for picking; this is an indexed factory cell, not interception of a moving target. The camera supplies the shape and pickup position. LiDAR independently corroborates object height/presence. The suction model creates a Gazebo fixed joint only when the tool is close, upright and the part is stationary. That is an explicit simulation approximation, not a validated vacuum or finger-contact model.

## Install and build

Stop the old Gazebo/MoveIt launch first. Use Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic; deactivate Conda/venv.

```bash
cd ~/6DOF_fyp
git pull --ff-only
cd robotics_lab
bash scripts/setup_ubuntu.sh --with-moveit --yes --build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select pieas_sorting pieas_arm_moveit_config
colcon test-result --verbose
```

The setup script installs missing OpenCV, SciPy, ROS sensor bridges and Gazebo plugin development dependencies through apt/rosdep. It is idempotent. The first C++ plugin build can take longer than the earlier Python-only changes.

## Launch the factory, initially idle

```bash
source /opt/ros/jazzy/setup.bash
source ~/6DOF_fyp/robotics_lab/install/setup.bash
ros2 launch pieas_sorting factory.launch.py
```

This opens Gazebo and RViz, loads the suction variant, starts controllers, camera, LiDAR, bridges and perception, and publishes the static cell geometry to MoveIt. No sorting motion starts automatically. The belt is stopped. There is one RGBD camera and one GPU LiDAR; the old lab sensor rigs are not duplicated.

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/6DOF_fyp/robotics_lab/install/setup.bash
ros2 run pieas_sorting check_cell --ros-args -p use_sim_time:=true
```

`READY` means all five required streams advance. It does not mean a batch has already been tested.

## Run the batch

```bash
ros2 run pieas_sorting sort_batch --ros-args \
  -p use_sim_time:=true \
  -p config:="$(ros2 pkg prefix --share pieas_sorting)/config/cell.yaml" \
  -p execute:=true
```

Use `-p cycles:=1` for a single-cycle commissioning run. After a successful commissioning run, restart the factory before the six-part batch so counters, parts and planning scene start consistently. For a fresh scene you can launch and start everything with:

```bash
ros2 launch pieas_sorting factory.launch.py start_sorting:=true
```

Expected phases: `PARK_FOR_INSPECTION`, `INDEX_BELT`, `DETECT_AND_CLASSIFY`, `APPROACH_PICK`, `DESCEND_TO_PICK`, `ATTACH_SUCTION`, `LIFT_PAYLOAD`, `TRANSFER_TO_<SHAPE>`, `LOWER_TO_BIN`, `RELEASE`, `RETRACT`, then repeat. `COMPLETE` follows all requested cycles. Each cycle is only counted after the released object's simulated position is inside the intended bin.

CSV results go to `~/.ros/pieas_sorting/batch_<date_time>.csv`. Speed remains conservative; this is not a 30-second sweep. Planning may take seconds and a batch can take several minutes.

## Observe and stop

In RViz, enable the **LiDAR** display to see the cloud; **Inspection camera** shows RGB; **Detected shapes** labels stable detections. The perception mask is available on `/factory/perception_mask` and can be viewed with an Image display. The camera is intentionally occluded during pickup; detection is required at the inspection stage, not while carrying.

```bash
ros2 topic echo /factory/task_state
ros2 topic echo /factory/detections --once
ros2 topic echo /factory/state --once
```

Ctrl+C in the task terminal requests trajectory cancellation and stops the belt. A held object is retained rather than dropped at an arbitrary location. The plugin's heartbeat watchdog also stops the belt when the task disappears. Gazebo's Pause button freezes physics. **Do not reset during an active task.** After a fault or a partial batch, stop both task and launch and start a fresh factory; this release does not resume partially completed batches.

## File map

| Read order | File | Purpose |
|---|---|---|
| 1 | `docs/WALKTHROUGH.md` | Full explanation and call order |
| 2 | `config/cell.yaml` | Belt, objects, bins, sensors, tolerances and operating envelope |
| 3 | `pieas_sorting/world.py` | Generate Gazebo world and temporary suction robot variant |
| 4 | `config/bridge.yaml` | Translate Gazebo sensor/control messages to/from ROS |
| 5 | `pieas_sorting/perception_core.py` | Projection, silhouette classification and LiDAR corroboration |
| 6 | `pieas_sorting/perception.py` | ROS subscriptions, freshness, stability and debug output |
| 7 | `pieas_sorting/kinematics.py` | Bounded upright-tool numerical IK; yaw is free |
| 8 | `pieas_sorting/sorter.py` | Task states, MoveIt scene changes, planning, execution and verification |
| 9 | `src/factory_system.cpp` | Gazebo traction, station indexing, watchdog and suction physics joint |
| 10 | `launch/factory.launch.py` | Connect the complete system |
| 11 | `test/test_core.py` | CPU tests of geometry, perception and IK |
| 12 | `test/runtime_smoke.py` | Explicit real ROS/Gazebo sensor + pick/place test |

All positions in configuration are world metres; angles are radians. Bin names map perception classes to destinations. `cell.yaml` is not a general robot calibration file: changing camera pose requires matching real-world calibration if this is later deployed on hardware.
