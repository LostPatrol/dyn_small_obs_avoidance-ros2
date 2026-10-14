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
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths path_searching path_planning --executor sequential \
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
It loads the same installed `planner.yaml` as the regular launch (300 ms search budget).
Use `--config /absolute/path/planner.yaml` or an explicit `--search-budget 0.08` / `0.3` for comparisons.
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
  '{header: {frame_id: odom}, pose: {position: {x: 4.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'
```

The default planning frame is `odom`. Adjust it and the planning bounds in
[planner.yaml](path_planning/config/planner.yaml); load your copy with `config:=/absolute/path/planner.yaml`.
Results are published on `/plan_result` and `/kino_path`.
`PlanResult` is authoritative for current validity; `/kino_path` retains the last successful preview
with its original timestamp when a later attempt fails.
ROS 2 bag replay, input conventions and configuration are described in the [usage guide](docs/USAGE.md).

Both manual and bridge targets use native goal/time/distance/collision replanning and the
same `dual_tree` map. `/set_goal` (`path_planning/srv/SetGoal`) accepts `header`, `goal`,
`target_id`, and `active`; its response acknowledges `planner_session` and `goal_revision`.
An active request needs a nonempty ID and the configured frame. Repeating the same ID and
position is idempotent. `active: false` cancels the target while retaining the map; a nonempty
cancel ID must match the active target. Each manual `/goal` message gets a generated ID.
Native `PlanResult` includes `planner_session`, `target_id`, `sequence`, and `goal_revision`.
`PlanMotion/CHECK_LINE` remains available for current-map line inspection and does not replace
the active target. Idle nodes do not search; a 1 Hz identity-only heartbeat has an empty target ID and no trajectory.
The same heartbeat keeps an active session visible while it waits for a replan, at any distance.
Set `replan_interval` or `replan_distance` to `.inf` to disable that forced trigger;
goal and blocked-path events remain active.
If an input failure revokes that curve, recovery schedules one plan from the actual state.

The shared YAML's selected comparison profile sets horizontal speed/acceleration **norms** to
2.0 m/s and 2.0 m/s², with the same vertical absolute limits. A measured overspeed start keeps its exact initial velocity and
first brakes into the envelope within the acceleration, clearance and position bounds; speed
can exceed the nominal limit during this monotonically decelerating prefix. An infeasible braking
prefix fails. All subsequent primitives and terminal cubic shots satisfy nominal limits in their
original time. Set `search.max_vel_xy`, `search.max_vel_z`, `search.max_acc_xy`,
and `search.max_acc_z` in YAML or as launch arguments. Legacy scalar `search.max_vel` and
`search.max_acc` CLI arguments fill both directions unless a more specific directional CLI
argument is also supplied. Thus explicit CLI values override YAML, and directional CLI values
win for their direction. In direct C++ `SearchConfig`, scalar caps still apply alongside the
four directional limits. Parameters are copied at startup; restart to apply edits.
`search.*` is the single source for avoidance dynamics. Every `PlanResult`, including identity
heartbeats and `PlanMotion` responses, carries `max_velocity_xy`, `max_velocity_z`,
`max_acceleration_xy`, and `max_acceleration_z` from the effective core configuration for execution.
Straight motion uses the control project's independent `control.yaml` limits.
The original acceleration scalar was 2.0 m/s² **per axis**; the new horizontal norm cap of
2.0 m/s² excludes the old simultaneous x/y extreme of 2√2 m/s².
The ordinary primitive duration `search.max_tau` is 0.6 s. When even a half-acceleration axis
candidate exceeds its speed cap from rest, its five controls sample the feasible positive and
negative acceleration intervals using current velocity and duration. Other axes retain the
original lattice. The narrow Z axis also includes exact stopping acceleration `-vz/max_tau` when
it is bounded and distinct, retaining all original 125 controls and using at most 150 candidates.
This avoids carrying small vertical drift through the whole cruise without trajectory retiming.
Measured-state replans select acceleration immediately, without a forced initial coast that
could repeatedly postpone steering. The optional legacy execution-index mode retains its initial coast.
`odin.launch.py` defaults to the SDK's world-frame linear velocity; generic/MAVROS inputs retain body twist.
Override the Odin convention in `odin_launch.ros__parameters.odometry_twist_in_body` in YAML;
an explicit `odometry_twist_in_body:=true/false` launch argument takes precedence.

The shared YAML sets `search.vertical_distance_weight: 0.0` for the selected original-value profile.
A positive weight adds cost per metre of accumulated up/down travel, favouring horizontal detours when other costs are similar.
Primitives and terminal cubic candidates include this cost. The added height-difference
heuristic is temporarily disabled because it caused repeated vertical reversals.
Set it to `0.0` for legacy behavior (also the bare C++/node default); restart after changing it.
This is a soft preference, not a height constraint or a fix for incorrect state coordinates.
The shared YAML uses `lambda_heu=5`, with a single `1.0001` tie factor
(effective heuristic multiplier `5.0005`); the bare C++/node default remains 1.
Explicit YAML and CLI values override these defaults.
The half-speed duration estimate and position-only hash do not guarantee an
admissible heuristic or an optimal path.

## 5. Documentation

- [Detailed usage, visualization and replay guide (Chinese)](docs/USAGE.md)
- [Parameter effects and replanning triggers (Chinese)](docs/planner_parameters.md)
- [Porting notes (Chinese)](docs/PORTING.md)
- [Validation results and limitations (Chinese)](docs/VALIDATION.md)

The original paper's flight performance has not been reproduced by this port.
The current implementation does not predict obstacle motion or verify observed free space;
Standalone searches follow goal/cloud events and refresh every 0.2 s or 1 m of movement by default;
planning_rate controls timer checks and input-loss diagnostics.

## 6. Acknowledgments and license

Thanks to [HKU-MARS](https://github.com/hku-mars/dyn_small_obs_avoidance) for the original system
and [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) for the planning foundation.

This port is distributed under [GPLv3](LICENSE).
See [third-party notices](THIRD_PARTY_NOTICES.md) for the upstream revision, attribution and scope of changes.
