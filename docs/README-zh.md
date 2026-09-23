<!-- 与英文主页对应的简明中文介绍；详细操作与接口见USAGE.md。 -->
# dyn_small_obs_avoidance-ros2

### 面向 ROS 2 Jazzy 的运动学避障规划

[English](../README.md)

[HKU-MARS dyn_small_obs_avoidance](https://github.com/hku-mars/dyn_small_obs_avoidance)
的独立 ROS 2 移植，包含 `path_searching` 和 `path_planning` 两个模块。
基于时间累积 KD-Tree 地图、运动学 A* 搜索和三次多项式终端连接，
从已配准点云和里程计生成轨迹。

规划器发布多项式轨迹、位置/速度/加速度采样和可视化路径，
独立于 SLAM、传感器驱动和飞控运行。

原论文：*Avoiding dynamic small obstacles with onboard sensing and computing on aerial robots*。
[论文](https://arxiv.org/abs/2103.00406) · [视频](https://youtu.be/pBHbQ_J1Qhc)

## 1. 环境要求

- Ubuntu 24.04、ROS 2 Jazzy
- PCL、Eigen、colcon
- 已在 amd64 和 aarch64 上验证

依赖安装和环境配置见[使用指南](USAGE.md)。

## 2. 编译

安装 ROS 2 并初始化 rosdep 后，在终端运行：

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

## 3. 运行示例

无需传感器或飞控，运行合成竖杆绕障场景：

```bash
.venv/bin/python path_planning/test/benchmark.py \
  --seconds 30 --output /tmp/planner-benchmark.json
```

脚本自动启动和回收规划器，输出规划结果与耗时统计。
示例起点固定，不模拟飞机沿路径运动。
显示障碍点云和规划路径的方法见 [RViz2 可视化](USAGE.md#rviz2-可视化)。

## 4. 接入其他传感器或数据包

提供处于同一世界坐标系的已配准 `PointCloud2` 和 `Odometry`：

```bash
ros2 launch path_planning planner.launch.py \
  cloud:=/cloud_registered odom:=/Odometry goal:=/goal
```

在另一个已加载工作区环境的终端发布目标：

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: camera_init}, pose: {position: {x: 4.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'
```

默认规划坐标系为 `camera_init`。按实际输入调整
[planner.yaml](../path_planning/config/planner.yaml) 中的坐标系和规划边界，
通过 `config:=/absolute/path/planner.yaml` 加载配置副本。
规划结果发布到 `/plan_result` 和 `/kino_path`。
ROS2 bag 回放、输入约定和参数说明见[使用指南](USAGE.md)。

## 5. 配套文档

- [详细使用、可视化与回放指南](USAGE.md)
- [移植与修复说明](PORTING.md)
- [验证结果和限制](VALIDATION.md)

本移植未复现原论文的实飞性能。目前没有障碍运动预测或已观测自由空间判断，
10 Hz 规划不代表硬实时保证。

## 6. 致谢与许可

感谢 [HKU-MARS](https://github.com/hku-mars/dyn_small_obs_avoidance) 提供原始系统，
以及 [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) 提供规划基础。

本移植遵循 [GPLv3](../LICENSE)。
上游版本、来源和变更范围见[第三方声明](../THIRD_PARTY_NOTICES.md)。
