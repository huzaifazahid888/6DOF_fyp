# Validation and explicit limits

## Checks performed during development

- Pure Python tests exercise shape classification on synthetic RGBD images, metric projection, LiDAR disagreement rejection, unknown shape rejection, frame stability, world/collision geometry consistency, preservation of original joint transforms, reachable station/bin IK and unreachable-target rejection.
- Existing MoveIt configuration tests are retained and run alongside the factory tests.
- `test/mesh_audit.py` is a reproducible optional offline mesh collision audit (requires `trimesh` and `python-fcl`) that checks the suction-variant home pose, parking/pick poses and both drop positions for all three bins, using the actual arm collision meshes and cell box geometry. Its sampled endpoints are collision-free. This is not a continuous-path or physics validation; runtime MoveIt planning remains mandatory.
- Python compilation and generated XML/YAML parsing are checked.
- `.github/workflows/factory.yml` builds with ROS 2 Jazzy on Ubuntu 24.04, runs unit tests, then invokes `test/runtime_smoke.py`. The runtime test requires actual advancing RGB, depth, LiDAR and joint streams and requires six physical attach/carry/release cycles and checks the CSV for two verified placements of each shape. Check the Actions result for the exact commit you pull; a workflow being present does not imply it passed.

The development workspace does not itself contain ROS 2 or Gazebo. Offline checks must not be presented as proof that the complete Gazebo batch ran. The runtime workflow and commissioning commands are provided to verify that distinction explicitly.

## Recorded runtime result

[Run 43281803114](https://github.com/huzaifazahid888/6DOF_fyp/actions/runs/43281803114) passed on commit `1ac877c`: Ubuntu/Jazzy build, package tests, actual sensor streams, camera classification of a cube (77 supporting LiDAR points), pickup, transport, release and stationary placement verification. Reported measured-state FK endpoint errors were 0.11–0.58 mm. These are model-based simulation errors, not external metrology. That run tested one object. The workflow has subsequently been expanded to all six parts; check its result separately.

[Run 43372619445](https://github.com/huzaifazahid888/6DOF_fyp/actions/runs/43372619445) verified the cube and rectangular-block placements, then stopped at the cylinder retract with MoveIt's `CheckStartStateBounds` error. Continuous wrist feedback is now normalized to an equivalent angle in [-pi, pi] before constructing planning start states; a regression test verifies unchanged FK across multiple encoder turns. The full six-part batch is **not yet verified** until a subsequent runtime run passes.

## Operating assumptions

The supplied six parts are isolated, green, flat-topped and upright; the classes are cube, rectangular block and cylinder. Their heights are around 30 mm. This classifier is not trained for arbitrary industrial objects, overlapping piles, transparent surfaces or unknown shapes. Green separates foreground; class does not come from colour or object identity.

The fixed overhead RGBD camera supplies aligned RGB and depth; LiDAR confirms height/presence. Both are calibrated from the known SDF sensor mount poses. No SLAM, dynamic-obstacle avoidance, semantic neural network or online camera calibration is included.

The belt stops for each pick. The traction model applies bounded forces to objects on a low-friction deck. It is an approximation of an indexed conveyor, not roller mechanics or moving-target interception.

The RG2 geometry in the factory is replaced by a simulation suction tool. A proximity-gated fixed physics joint approximates attachment. Vacuum pressure, seal leakage, finger contact and a holding-force limit are not modelled. A detached object still falls and collides normally. This tool must not be described as a validated hardware gripper.

Numerical upright IK is separate from MoveIt collision validation. The factory rail envelope is ±0.16 m, an operating restriction selected for this scene, not a measured mechanical rating. Velocity/acceleration and effort settings are inherited simulation settings. Full motor/gearbox thermal and torque feasibility is not established.

Transport may rotate the held object; the endpoint is upright. MoveIt collision sampling is discrete. Good modelling and a successful plan do not prove perfect physical safety, zero contact penetration or hardware reliability.

Simulator ground truth is used for indexing, suction proximity checks and released-object verification only. The perception path does not consume ground-truth class or pickup pose. Placement scene updates use release ground truth; hardware needs an independent placement sensor.

Each bin has two drop slots. Start a fresh launch for a new batch or after a fault. Do not run RViz Execute or another motion client during an automatic cycle. Resetting Gazebo during a task invalidates its scene/state bookkeeping; relaunch both task and world.

## Acceptance procedure

1. Build and pass the unit tests.
2. Launch the idle factory and run `check_cell` until it reports advancing sensors and joint feedback.
3. Run a fresh scene with `cycles:=1`; require `COMPLETE` and `verified_in_bin` in its CSV.
4. Restart and run all six parts; require two placements in each bin, no fault, and six CSV rows.
5. Inspect RGB, the segmentation mask and LiDAR cloud to ensure detections correspond to visible parts.
6. Repeat with LiDAR disabled: perception must reject pickup rather than proceed blindly.
7. Pause during a task: execution must not silently proceed on stale feedback; stop and relaunch afterward.
8. Move a bin or change a camera pose only through the shared config, rebuild/relaunch and repeat acceptance.

## Reference APIs

- [Gazebo Harmonic detachable joints](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DetachableJoint.html)
- [Gazebo Link forces and velocities](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1Link.html)
- [ros_gz_bridge](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/)
- [MoveIt planning scene](https://moveit.picknik.ai/main/doc/examples/planning_scene/planning_scene_tutorial.html)
- [MoveIt controller integration](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html)
- [SciPy bounded least squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)
