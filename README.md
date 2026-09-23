<!-- Concise English project homepage; detailed operating instructions are in docs/USAGE.md. -->
# dyn_small_obs_avoidance-ros2

### Kinodynamic obstacle avoidance for ROS 2 Jazzy

[中文说明](docs/README-zh.md)

A standalone ROS 2 port of [HKU-MARS dyn_small_obs_avoidance](https://github.com/hku-mars/dyn_small_obs_avoidance),
containing the `path_searching` and `path_planning` modules. It combines time-accumulated
KD-Tree mapping, kinodynamic A* search and a cubic terminal connection to plan trajectories
from registered point clouds and odometry.

The planner publishes polynomial trajectories, sampled position/velocity/acceleration and a
path for visualization. It runs independently of SLAM, sensor drivers and flight controllers.

Original work: *Avoiding dynamic small obstacles with onboard sensing and computing on aerial robots*.
[Paper](https://arxiv.org/abs/2103.00406) · [Video](https://youtu.be/pBHbQ_J1Qhc)

## 1. Prerequisites

- Ubuntu 24.04 and ROS 2 Jazzy
- PCL, Eigen and colcon
- Tested on amd64 and aarch64

See the [usage guide](docs/USAGE.md) for dependency installation and environment setup.

## 2. Build

With ROS 2 and rosdep initialized:

```bash
git clone https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2.git
cd dyn_small_obs_avoidance-ros2
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
rosdep install --from-paths path_searching path_planning --ignore-src -r -y --rosdistro jazzy
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
source install/setup.bash
```

## 3. Run demo

Run a synthetic pole-avoidance example without a sensor or flight controller:

```bash
.venv/bin/python path_planning/test/benchmark.py \
  --seconds 30 --output /tmp/planner-benchmark.json
```

The script starts and stops the planner automatically, and reports planning results and timing.
It tests planning with a stationary starting position; it does not simulate a vehicle following the path.
See the [RViz2 instructions](docs/USAGE.md#rviz2-可视化) to display the obstacle cloud and planned path.

## 4. Run with other sensors or bags

Provide registered `PointCloud2` and `Odometry` inputs in the same world frame:

```bash
ros2 launch path_planning planner.launch.py \
  cloud:=/cloud_registered odom:=/Odometry goal:=/goal
```

In another terminal with the workspace sourced, publish a target:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: camera_init}, pose: {position: {x: 4.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'
```

The default planning frame is `camera_init`. Adjust it and the planning bounds in
[planner.yaml](path_planning/config/planner.yaml); load your copy with `config:=/absolute/path/planner.yaml`.
Results are published on `/plan_result` and `/kino_path`.
ROS 2 bag replay, input conventions and configuration are described in the [usage guide](docs/USAGE.md).

## 5. Documentation

- [Detailed usage, visualization and replay guide (Chinese)](docs/USAGE.md)
- [Porting notes (Chinese)](docs/PORTING.md)
- [Validation results and limitations (Chinese)](docs/VALIDATION.md)

The original paper's flight performance has not been reproduced by this port.
The current implementation does not predict obstacle motion or verify observed free space;
10 Hz planning is not a hard real-time guarantee.

## 6. Acknowledgments and license

Thanks to [HKU-MARS](https://github.com/hku-mars/dyn_small_obs_avoidance) for the original system
and [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) for the planning foundation.

This port is distributed under [GPLv3](LICENSE).
See [third-party notices](THIRD_PARTY_NOTICES.md) for the upstream revision, attribution and scope of changes.
