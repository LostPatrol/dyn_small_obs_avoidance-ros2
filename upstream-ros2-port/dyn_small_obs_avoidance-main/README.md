<!-- Locally prepared ROS 2 interface adaptation of the upstream planner for comparison. -->
# dyn_small_obs_avoidance — local ROS 2 port

This is a locally prepared ROS 2 interface and build adaptation of
[HKU-MARS dyn_small_obs_avoidance](https://github.com/hku-mars/dyn_small_obs_avoidance),
not an official upstream ROS 2 release. It preserves original algorithm behavior,
including the known issues in [BUGS.md](BUGS.md). The maintained modified version
is in the [repository root](../../README.md). Build this reference in a separate
workspace because both versions use the same ROS package names.

### Avoiding dynamic small obstacles with onboard sensing and computating on aerial robots

This repository is used for UAV dynamic small obstacles avoidance. It is a complete system for lidar-based UAV, including FAST-LIO slam, time-accumulated KD-Tree mapping and kinodynamic A* search modules. It is able to avoid dynamic small obstacles (down to 20mm diameter bars) by running at 50Hz.

![RAL_coverfigure6](https://user-images.githubusercontent.com/23183555/109411264-d39ccf00-79db-11eb-85d3-764a6b17235d.png)

![outdoor2_movingobs](https://user-images.githubusercontent.com/23183555/109411282-f202ca80-79db-11eb-8ae0-704bda25ff12.png)

Related paper:
"Avoiding dynamic small obstacles with onboard sensing and computating on aerial robots", 
available on arxiv now https://arxiv.org/abs/2103.00406.

Related video:
https://youtu.be/pBHbQ_J1Qhc

 
## 1. Prerequisites

Ubuntu 24.04 and ROS 2 Jazzy, with PCL and Eigen. This adaptation changes the ROS interfaces and build configuration; the original planning algorithms are preserved. Known original issues are listed in [BUGS.md](BUGS.md).

## 2. Build

Place the adapted `path_planning` and `path_searching` packages under `~/ros2_ws/src`, then build:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to path_planning
source install/setup.bash
```

## 3. Run

Start ROS 2 point-cloud, odometry and local-pose sources, then launch the planner:

```bash
ros2 launch path_planning demo_withNOlidar.launch.py
```

The launch files configure the point-cloud and odometry inputs as `/odin1/cloud_slam_down` (`sensor_msgs/msg/PointCloud2`), `/odin1/odometry_highfreq` (`nav_msgs/msg/Odometry`), `/mavros/local_position/pose` and `/goal` (both `geometry_msgs/msg/PoseStamped`). The trajectory time index input remains `/demo_node/trajectory_time_index` (`std_msgs/msg/Int64`). Input data must use the coordinate conventions expected by the original planner.

To run without RViz:

```bash
ros2 launch path_planning demo_withNOlidar.launch.py rviz:=false
```

The input remappings are configured in `path_planning/launch/demo_withNOlidar.launch.py`; both demo launch files use them. The RViz configuration uses the same sensor topics.

Set a target point:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped '{header: {frame_id: "camera_init"}, pose: {position: {x: 5.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'
```

Search parameters are in `path_planning/config/planner.yaml`; ROS 1 parameter names such as `search/max_vel` become `search.max_vel` in ROS 2. The three original latched trajectory topics use transient-local durability; late subscribers should also request transient-local durability to receive the retained sample.

For a converted ROS 2 bag, launch with `use_sim_time:=true` and play with `ros2 bag play <bag_directory> --clock`. The original ROS 1 `.bag` files require conversion before use with this package.

## 4. External sensors

FAST-LIO, Livox drivers and MAVROS are external dependencies and must have ROS 2 versions. `demo.launch.py` starts the same planner and optionally includes ROS 2 sensor launch files supplied by absolute path:

```bash
ros2 launch path_planning demo.launch.py fast_lio_launch:=/absolute/path/to/fast_lio.launch.py livox_driver_launch:=/absolute/path/to/livox.launch.py
```

If those arguments are omitted, start the sensors separately. The auxiliary clicked-point transformer is available as `ros2 run path_planning goal_transform` and retains its original transform behavior.

## 5.Acknowledgments
Thanks for FAST-PLANNER(Zhou, Boyu and Gao, Fei and Wang, Luqi and Liu, Chuhao and Shen, Shaojie. Robust and efficient quadrotor trajectory generation for fast autonomous flight), [FAST-PLANNER](https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git).
