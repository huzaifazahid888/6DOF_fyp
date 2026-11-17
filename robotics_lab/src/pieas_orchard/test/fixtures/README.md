# Gazebo sensor regression frame

`gazebo_frame.npz` contains RGB, depth, camera intrinsics and LiDAR from orchard CI run 44105953025, attempt 2, revision 470ea973c4f0781a53203a5717b5512ee802aa09. RGB and depth are subsampled by two; intrinsics are scaled accordingly. Sensor mount positions are recorded in the fixture.

The first learned model treated the rendered blemish as background. This frame exposed the missing gamma variation in the synthetic training distribution. It is now a development regression fixture. It must not be counted as held-out model validation. Fruit labels and positions in the test are expected simulation truth used for scoring, not runtime perception inputs.
