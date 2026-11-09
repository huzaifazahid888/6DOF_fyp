# PIEAS 6-DOF arm description

Extracted from `final forward at.ttt`. The model contains the arm's eight STL files, original joint transforms, travel limits, and scene mass and inertia data. The neutral pose uses zero for the six arm joint positions. The `base_link` frame uses the source scene's axes, with its origin beneath the slide axis at the bottom of the mounting feet. The `tool0` frame is the scene's Dummy reference on the RG2 and is available for planning. Check its offset against your desired grasp point before using it as a grasp TCP.

| URDF joint | V-REP name | Type | Scene travel |
| --- | --- | --- | --- |
| joint_1 | Prismatic_joint | Prismatic | −0.5 to +0.5 m |
| joint_2 | rev1 | Revolute | −π to +π rad |
| joint_3 | rev2 | Revolute | −π to +π rad |
| joint_4 | rev3 | Revolute | −π to +π rad |
| joint_5 | rev4 | Revolute | −π to +π rad |
| joint_6 | Revolute_joint | Continuous | Cyclic |

The URDF preserves the scene travel settings. Check usable travel and collisions before commanding the extremes. In particular, the V-REP slide range may exceed the physical rail travel. Joints 2–5 used zero target effort in the scene. The URDF uses provisional 10 N·m limits for these joints. Treat the limits and masses as simulation inputs until checked against the physical arm. Visual colors were assigned during conversion.

The RG2 is rigid at the scene's saved finger pose. Its visible and collision shapes are included and its moving bodies are combined into one inertial body. The original gripper CAD is courtesy of On Robot ApS, as credited in the scene. The V-REP scripts, gripper actuation, and scene environment are outside the URDF. Verify rights to the source assets before redistribution.

## Preview in Gazebo Harmonic with ROS 2 Jazzy

With ROS 2 Jazzy, `colcon` and `ros_gz_sim` available, extract this package under a workspace's `src` directory. From that workspace root, run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pieas_arm_description
source install/setup.bash
pkg_dir="$(ros2 pkg prefix --share pieas_arm_description)"
export GZ_SIM_RESOURCE_PATH="$(dirname "$pkg_dir"):${GZ_SIM_RESOURCE_PATH:-}"
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf"
```

In another terminal at the same workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
pkg_dir="$(ros2 pkg prefix --share pieas_arm_description)"
ros2 run ros_gz_sim create -world empty -name pieas_arm -file "$pkg_dir/urdf/pieas_6dof_arm.urdf"
```

The base is fixed to the world. Use the paused simulation for the initial visual check. Commanded motion requires a `gz_ros2_control` setup and trajectory controller. MoveIt configuration can use the chain from `base_link` to `tool0`.

## Validation

Mesh references, STL geometry, link connectivity, and forward kinematics were checked offline. The URDF and V-REP joint chains agreed across 101 configurations, including zero. Gazebo runtime loading and contact dynamics have yet to be tested.
