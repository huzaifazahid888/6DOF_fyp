# Factory setup and operation

[Package overview](../README.md) · [Validation](VALIDATION.md) · [Troubleshooting](TROUBLESHOOTING.md)

## Current tool configuration

The current main-branch launch reads the original rigid gripper directly from the description template. The task and Gazebo backend still implement suction attachment. Idle inspection is available, but the batch commands below describe the **suction baseline** and are not acceptance instructions for the restored gripper.

In `launch/factory.launch.py`, the tested suction description was generated with `robot_variant(...)`. The current `.read_text()` line bypasses that conversion. Restoring appearance does not add finger actuation. The planned `gripper_type:=suction/original` interface is not implemented. This documentation update does not change the launch or enable automatic motion.

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

This opens Gazebo and RViz, loads the currently selected description in the launch source, starts controllers, camera, LiDAR, bridges and perception, and publishes the static cell geometry to MoveIt. No sorting motion starts automatically. The belt is stopped. There is one RGBD camera and one GPU LiDAR; the old lab sensor rigs are not duplicated.

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/6DOF_fyp/robotics_lab/install/setup.bash
ros2 run pieas_sorting check_cell --ros-args -p use_sim_time:=true
```

`READY` means all five required streams advance. It does not mean a batch has already been tested.

## Suction-baseline batch commands

```bash
ros2 run pieas_sorting sort_batch --ros-args \
  -p use_sim_time:=true \
  -p config:="$(ros2 pkg prefix --share pieas_sorting)/config/cell.yaml" \
  -p execute:=true
```

Use `-p cycles:=1` for a single-cycle commissioning run. After a successful commissioning run, restart the factory before the six-part batch so counters, parts and planning scene start consistently. For a fresh scene with a compatible suction description, the combined command is:

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
