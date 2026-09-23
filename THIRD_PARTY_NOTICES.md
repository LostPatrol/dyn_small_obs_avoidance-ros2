<!-- Provenance and license record for the extracted upstream algorithm. -->
# Third-party notices

- Upstream: [hku-mars/dyn_small_obs_avoidance](https://github.com/hku-mars/dyn_small_obs_avoidance).
- Fixed revision: `9d25dd6974e04518c892e5711c9be9082558e5eb` (2021-04-27).
- Upstream package author/maintainer: kongfz, `kongfz@connect.hku.hk`.
- License: GNU GPL version 3; the upstream `LICENSE` is copied verbatim.
- Upstream acknowledges [HKUST-Aerial-Robotics/Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
  for the kinodynamic planning foundation. This acknowledgement is retained.
- The initial commit of this repository contains the exact upstream search header/source and LICENSE.
  Subsequent commits expose the ROS2 port and defect fixes as a reviewable diff.
- Only `path_searching` and the responsibilities of `path_planning` are ported. FAST-LIO,
  Livox drivers, flight/demo controllers and the unused DenseInput header are not included.
- The ROS1 wrapper is replaced with a Jazzy rclcpp node. Search primitives, heuristic polynomial
  solver, terminal cubic construction, spatial hash and two-bank accumulated KD-Tree design are retained.
- Changes by LostPatrol, 2026, are distributed under the same GPLv3 license.
- Eigen and PCL remain separately installed dependencies, not vendored source.

The original repository and its ROS1 checkout are not modified by this port.
