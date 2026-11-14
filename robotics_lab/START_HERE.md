# Setup and launch guide

[Workspace overview](README.md) · [Code guide](docs/CODE_GUIDE.md) · [Roadmap](docs/ROADMAP.md)

This page contains the common setup. Each package README covers its own experiment. Commands assume the repository is at `~/6DOF_fyp`; a different checkout location needs the corresponding path.

## Install and build

The target system is Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. Installation runs from a normal user account with sudo access. Conda or a Python virtual environment must be deactivated first.

For a new checkout:

```bash
git clone https://github.com/huzaifazahid888/6DOF_fyp.git ~/6DOF_fyp
cd ~/6DOF_fyp/robotics_lab
bash scripts/setup_ubuntu.sh --with-moveit --yes --build
```

For an existing checkout, local edits should be committed or backed up before updating. From the repository root, `git pull --ff-only` updates without creating an automatic merge. The same setup script can then be rerun; it checks for missing dependencies before installation.

| Setup option | Purpose |
| --- | --- |
| `--check` | Check prerequisites without installation |
| `--with-moveit` | Include MoveIt and Setup Assistant |
| `--yes` | Accept package installation prompts |
| `--build` | Build after dependency checks |
| `--help` | Show script usage |

In each new ROS terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/6DOF_fyp/robotics_lab/install/setup.bash
```

## Choose a stage

Only one of these integrated launches should run at a time. Both the planning and factory launches already start Gazebo and their required nodes.

| Stage | Launch | Details |
| --- | --- | --- |
| Model and controller inspection | `ros2 launch pieas_arm_description lab.launch.py` | [Robot model](src/pieas_arm_description/README.md) |
| MoveIt planning and tool movement | `ros2 launch pieas_arm_moveit_config planning.launch.py` | [Planning experiments](src/pieas_arm_moveit_config/README.md) |
| Factory layout and sensor inspection | `ros2 launch pieas_sorting factory.launch.py` | [Factory status and operation](src/pieas_sorting/README.md) |

The factory currently loads the original rigid gripper while its sorter retains suction logic. The idle launch is useful for inspection; automatic sorting requires resolving that tool mismatch. The earlier suction run is a separate baseline, not validation of this configuration.

## Check the running system

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
ros2 topic echo /clock --once
```

`arm_controller` and `joint_state_broadcaster` should be active. The six position command interfaces should be claimed. Joint messages should update as simulation time advances. Very small floating-point values near zero are normal; an effort value of NaN means no usable effort measurement is present in that field.

The [MoveIt guide](src/pieas_arm_moveit_config/README.md) is the active route for commanded motion. The old full-range joint sweep is retired because it did not account for collisions.

## Editing source and loading changes

Source files are under `src/`. ROS resolves installed packages under `install/`; temporary factory URDF/SDF files are generated at launch. Editing source does not update an already running model.

After stopping the launch, rebuild from the workspace root:

```bash
cd ~/6DOF_fyp/robotics_lab
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

A successful build is required before relaunch. The installed location can be checked with `ros2 pkg prefix --share pieas_sorting`. This distinction matters when a source edit appears to have no effect. Gazebo Reset does not reload edited source; [reset and home](RESET_AND_HOME.md) explains the difference.

## Tests and diagnosis

```bash
colcon test --packages-select pieas_arm_control pieas_arm_moveit_config pieas_sorting
colcon test-result --verbose
```

Unit tests check parts of the model and application logic. Actual sensor delivery, contact behaviour and complete task execution need runtime checks as well. [Factory validation](src/pieas_sorting/docs/VALIDATION.md) records that boundary.

| Symptom | First check |
| --- | --- |
| `ros2 control` unavailable | Setup dependencies and ROS environment |
| No advancing feedback | Gazebo pause state, controller status and duplicate launches |
| Missing meshes or old model | Build result, installed package path and fresh launch |
| Missing camera/LiDAR data | Sensor renderer, bridge configuration and message timestamps |
| Invalid start state or no plan | Collision geometry, joint limits and planning scene |
| Factory descent misses target | Measured endpoint, contact and the loaded tool model |

Detailed factory diagnostics are in [TROUBLESHOOTING.md](src/pieas_sorting/docs/TROUBLESHOOTING.md).
