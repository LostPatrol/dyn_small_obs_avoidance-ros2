<!-- Detailed setup, interfaces, visualization, replay and diagnostic guide. -->
# 使用指南

[English README](../README.md) · [中文 README](README-zh.md)

以下命令均在仓库根目录执行。

[HKU-MARS dyn_small_obs_avoidance](https://github.com/hku-mars/dyn_small_obs_avoidance)
的独立 ROS 2 Jazzy 移植，只包含 `path_searching` 和 `path_planning`。
保留运动学 A*、恒加速度运动原语、三次多项式终端连接和交替累积的双 KD-Tree。
上游固定提交：`9d25dd6974e04518c892e5711c9be9082558e5eb`。GPLv3；来源及修复见
[THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md) 和 [移植说明](PORTING.md)。

不依赖 FAST-LIO、Livox、Odin、MAVROS 或任何飞控项目。节点只接收状态和点云、发布规划结果；
没有解锁、起飞、降落或 setpoint 接口。已验证平台为 Ubuntu 24.04 / Jazzy，amd64 和 aarch64。

## 构建与测试

在安装 ROS 2 Jazzy 的 Ubuntu 24.04 上：

```bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions python3-rosdep \
  python3-venv python3-pytest libeigen3-dev libpcl-dev \
  ros-jazzy-pcl-conversions ros-jazzy-rosidl-default-generators \
  ros-jazzy-ament-cmake-gtest ros-jazzy-ament-cmake-pytest ros-jazzy-sensor-msgs-py
git clone https://github.com/LostPatrol/dyn_small_obs_avoidance-ros2.git
cd dyn_small_obs_avoidance-ros2
source /opt/ros/jazzy/setup.bash
# 新环境只用系统 Python 自举；之后显式使用仓库自己的环境。
/usr/bin/python3 -m venv --system-site-packages .venv
rosdep install --from-paths path_searching path_planning --ignore-src -r -y --rosdistro jazzy
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
source install/setup.bash
colcon test
colcon test-result --verbose
```

若 rosdep 尚未初始化，先按 ROS 安装指南完成 `sudo rosdep init` 和 `rosdep update`。
两个包直接位于仓库根目录；也可以将整个仓库放进其他工作区 `src/`。
构建无需任何传感器、GPU或飞控在线。测试使用 localhost/domain 232，不访问飞机 domain 0。
ARM64 推荐限制构建并发，不在运行中的飞行任务期间构建。

## 快速验证

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# 自启动并回收独立节点，30 秒重复规划绕过竖杆；只在 localhost/domain 230 发布合成输入。
.venv/bin/python path_planning/test/benchmark.py --seconds 30 --output /tmp/planner-benchmark.json
```

这是规划计算与 DDS 测试，不是仿真飞机执行路径；最终输出包含状态计数、搜索时间分位数、
结果间隔、内存和 CPU。测试结果与未通过指标见 [验证记录](VALIDATION.md)。

## RViz2 可视化

先按上面的快速验证命令启动合成场景；可将 `--seconds` 改为 `600` 延长观察时间。
脚本已经启动规划器，不要重复启动第二个节点。

在另一个终端、仓库根目录运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=230
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_DISCOVERY_SERVER
rviz2
```

RViz2 的 Fixed Frame 设为 `map`，添加以下显示：

| 显示 | 话题 | 设置 |
| --- | --- | --- |
| PointCloud2 | `/cloud` | Best Effort；Color Transformer=FlatColor；红色；Style=Spheres；Size=0.08 m |
| Path | `/kino_path` | 绿色 |
| Odometry | `/odom` | Best Effort |

合成点云只有 XYZ，没有 intensity 字段，因此不要选择 Intensity 着色。
起点固定为 (0,0,1)，目标为 (4,0,1)，障碍杆在 x=2、y=0；
从斜上方观察可见规划路径绕过点柱。点的显示尺寸不改变规划碰撞半径。
这里只验证规划输出，里程计不会沿规划路径运动。

回放真实 bag 时，Fixed Frame 应改为该点云的实际 frame_id。
点云显示 OK 但看不见时，先检查颜色与背景、点尺寸及视角；
没有路径时查看 `/plan_result` 的 status/detail，而不是只凭画面判断成功。

## 接入外部输入

```bash
source install/setup.bash
ros2 launch path_planning planner.launch.py \
  cloud:=/cloud_registered odom:=/Odometry goal:=/goal
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: camera_init}, pose: {position: {x: 4.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'
ros2 topic echo /plan_result
```

配置模板位于 [planner.yaml](../path_planning/config/planner.yaml)。复制到外部路径修改后使用
`config:=/absolute/path/planner.yaml`。默认 frame 为 `camera_init`，默认高度范围 `[0.2,10.0] m`；
按实际坐标系设置 `search.lower/upper`，不要为让测试通过而修改实测点云或伪造机体高度。
节点不做 TF 转换；三个输入必须已经在同一固定世界系，frame_id 必须精确匹配，不接受带前导 `/` 的别名。

| 话题（节点相对名） | 类型 | 约定 |
| --- | --- | --- |
| `cloud` | `sensor_msgs/msg/PointCloud2` | 已配准的障碍点；little-endian FLOAT32 x/y/z，允许附加字段；非空且全有限 |
| `odom` | `nav_msgs/msg/Odometry` | pose 在规划 frame；默认 twist 在 child_frame_id，由姿态旋转到世界系 |
| `goal` | `geometry_msgs/msg/PoseStamped` | 位置目标，目标速度为零；姿态/yaw 不参与求解；目标持续有效直到被新目标替换 |
| `plan_result` | `path_planning/msg/PlanResult` | 同一消息中的状态、输入时标、序号、目标版本、精确多项式和 p/v/a 采样 |
| `kino_path` | `nav_msgs/msg/Path` | 同一轨迹的位置预览；失败发布空路径；不用于跨话题拼接控制结果 |

点云和里程计 QoS：BEST_EFFORT/VOLATILE/depth 1，可接可靠或非可靠传感器发布者。
目标与输出：RELIABLE/VOLATILE/depth 1，不保留旧规划供晚加入的消费者误用。
参数在启动时读取；修改配置后重启节点。

配置模板逐项标注了单位、约束与主要影响。`search.*` 默认值也在
[`SearchConfig`](../path_searching/include/path_searching/kinodynamic_astar.h) 中提供，
便于直接调用C++库；维护默认值时应同时核对YAML与头文件。
初始固定加速度时长和时间索引分辨率只对核心库的相应搜索模式生效，模板已标明；
它们不会让当前ROS节点自动启用动态障碍预测。

搜索采样比例、终端连接时长重试、数值容差和资源上限等实现常量集中在两个
`src/*.cpp` 文件顶部的匿名命名空间，并说明了用途。
这些常量不是运行时参数；修改时应重新构建并运行回归测试。
多项式求导的2/3/6等数学系数、XYZ维度和双树结构下标保留原公式写法，
不将其误当作可调经验参数。

默认每 100 ms 从最新有效实际里程计的 **位置和三轴速度** 重新搜索。
不订阅原演示器的 `trajectory_time_index`，不从旧参考点伪造当前位置。
Odometry 没有加速度字段，所以从上游的自由加速度原语分支开始，不假定实际加速度连续。
若某数据源明确将 twist 发布在世界系，配置 `odometry_twist_in_body: false`。

## 时间、轨迹与失败语义

- `stamp_clock: ros` 默认同时检查 ROS header 年龄和 steady-clock 接收年龄；接收超时为 0.5 s，
  未来时标最多容许 50 ms。bag 的 ROS 时钟回放应使用 `use_sim_time` 与 `/clock`。
- `stamp_clock: receive` 只判断接收年龄，用于设备时钟输入或保留原时标的离线回放。
  **它不证明传感器内部延迟为零**，不重写输入时标。重复帧不会刷新年龄或重复积累；
  时间倒退先清空/失效，再等待下一个严格递增的输入。
- 每次尝试均发布结果，失败的 segments 和 trajectory 为空；禁止消费者继续执行上一条成功结果。
  `REACH_END=2` 表示曲线抵达目标；`REACH_HORIZON=1` 是局部前缀，不能当作到点，也不保证终点已停车。
  `NO_PATH=3`、`INVALID_INPUT=5`、`NO_MAP=6`、`TIMEOUT=7`、`NODE_LIMIT=8`、
  `WAITING_FOR_INPUT=9`、`STALE_INPUT=10`、`FRAME_MISMATCH=11` 分别表示对应失败。
  保留枚举 `NEAR_END=4` 兼容源代码术语，目前终端连接失败继续搜索，不输出该状态。
- 多项式每轴为 `c0+c1*t+c2*t²+c3*t³`，`t∈[0,duration]`；按段顺序累计时间。
  p/v 连续，段间加速度允许跳变。采样包含精确首尾点与所有段边界，间隔不超过 `sample_step`。
  中间边界的 acceleration 采样取前段末值；精确段接口可按右连续约定求值。
- `header.stamp` 为本轮开始的 ROS 时间；`cloud_stamp/odometry_stamp` 保留原输入时间。
  `sequence` 在本进程递增，`goal_revision` 每收到一个目标递增；重启会归零。
  没有飞行执行器所需的跨进程 session、控制租约、接轨连续性与有效期协议。
- 输出旋转为单位四元数，角速度/角加速度为零，**不代表已规划 yaw**。

## 算法边界

1. 这是基于障碍点的加权运动学 A*；未观测空间仍按上游假设可通行。
   非空地图只表示有障碍观测，不表示完整 FOV 或自由空间证明。空图明确返回未知/失败。
2. 双树每 `tree_period=50` 个有效输入切换一组；点最多保留约两个周期。
   10 Hz 输入时可能保留约 5～10 s 历史障碍；动态物体消失不等于立即消除残影。
   没有动态障碍跟踪或未来运动预测，不能宣称复现论文 20 mm/50 Hz 飞行效果。
3. 最大速度/加速度是**逐轴**约束；三维合速度最大可达逐轴值的 √3 倍。
   不是限 jerk 轨迹、全局最优搜索或概率占据地图。
4. 碰撞半径为 `safe_distance + √3*voxel_size + collision_step/2`。
   后两项补偿体素质心位移和离散采样之间的位移上界，防止细障碍在样点之间漏检。
   它只相对输入点成立，无法补偿未被传感器测到的障碍或未经测量的跟踪误差。
5. `search_budget=0.08 s` 是单次搜索的协作退出预算，不含输入转换、PCL 体素化/建树、输出采样与 DDS。
   单线程执行器和非实时系统不能保证全链硬实时；负载测试必须查看失败计数和最长结果间隔。
6. 地图点数超过 `max_cloud_points`、异常数据、坐标不一致或过期会失效，不静默抽稀障碍以掩盖超限。
   点云与里程计使用各自最新值，没有时间插值、外参补偿、自体点剔除或世界坐标修正。

这些限制是未来集成的输入要求；当前仓库没有与任何飞控系统集成。

## 回放与诊断

通用 ROS2 bag 基准保留消息内容和原时标，按 bag 接收时刻间隔发布到隔离域：

```bash
.venv/bin/python path_planning/test/benchmark.py \
  --bag /absolute/path/ros2_bag --cloud-topic /cloud_registered --odom-topic /Odometry \
  --goal-offset 1 0 0 --seconds 30 --output /tmp/replay.json
```

目标仅用于离线验证，是第一条里程计位置加指定偏移；没有发布到真实传感器或飞控话题。
基准使用 z 下界 -10 m，以容纳原点在机体处的地面数据；不平移点云，不伪造离地状态。
若输入包只有原始 LiDAR 而没有已配准点云/里程计，需先由外部 SLAM 生成，不能直接喂给搜索器。
上游公开 ROS1 bag 链接在 2026-09-23 检查时返回 HTTP 404；本仓库未携带或声称验证该数据集。

### Odin 实时台架

现场 `/odin1/cloud_slam` 和 `/odin1/odometry` 均为 `odom` 坐标系、设备启动时间戳时使用：

```bash
source install/setup.bash
ros2 launch path_planning odin.launch.py
# 另一个已 source 的终端：
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 4.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'
ros2 topic echo /plan_result --field status
ros2 topic echo /kino_path --once
```

专用启动文件覆盖 `planning_frame=odom`、`stamp_clock=receive` 和
`search.lower=[-50,-50,-2]`（米），其余参数继承通用配置；不会启动传感器或飞控。
下界用于包含初始位置附近的地面台架起点，不代表真实地面高度或允许飞行空间。
接收时间模式不能判断传感器端积压延迟。目标必须使用 `odom`；输出是 `/kino_path`，有下划线。
状态 1/2 才含轨迹，3 表示无路；即使配置正确，起点附近的地面或机体点也可能使安全距离检查失败。
不能通过缩小安全距离或删除近身障碍物来宣称台架规划成功。

#### 可选近距离盲区

通用配置 `cloud.blind_radius` 为三维球形半径（米），默认 `0.0` 关闭。
Odin 入口用启动参数显式覆盖 YAML 中的半径，例如：

```bash
ros2 launch path_planning odin.launch.py blind_radius:=0.5
```

该配置用于接受传感器盲区的台架实验，不识别机体：球内真实障碍也会被忽略。
它使用三维距离，不照搬上游 FAST_LIO 的 `sqrt(x*x+y*y)` 水平距离盲区。
默认球心是里程计位姿原点；若与传感器原点不同，在 YAML 中设置
`cloud.blind_origin_offset: [x,y,z]`（里程计 child 坐标系中的米制偏移），
节点会按匹配姿态旋转该偏移。零偏移不意味着已标定 Odin IMU/雷达外参。

每帧点云匹配最多256条有效历史里程计中时间戳最近的一条，最大时间差由
`cloud.blind_pose_tolerance` 控制，默认0.05秒，不插值、不等待未来里程计。
两路必须使用相同源时钟，即使选择 `stamp_clock=receive` 也是如此；不匹配时清空地图并报告状态。
过滤发生在累计/体素化之前，只剔除严格小于半径的点，不随飞机移动重新挖除旧地图。
全部点被滤掉时报告 `NO_MAP`，不会视为整片自由空间。

RViz 添加 PointCloud2 订阅 `/filtered_cloud`，选择 Best Effort、FlatColor，可与原始点云比较。
这是当前帧过滤后的 XYZ，不是累计地图。修改参数须重启；重启也会清除旧累计点。
盲区半径与碰撞安全半径独立；配置0.5m盲区并不保证满足默认约0.648m安全半径，更不保证目标净空。

ASan/UBSan（Ubuntu 24.04 系统 PCL，单独 build/install）：

```bash
colcon --log-base log-sanitized build --packages-select path_searching \
  --build-base build-sanitized --install-base install-sanitized \
  --cmake-args -DENABLE_SANITIZERS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
  colcon --log-base log-sanitized test --packages-select path_searching \
  --build-base build-sanitized --install-base install-sanitized
colcon test-result --test-result-base build-sanitized --verbose
```

检测构建显式维持系统 PCL 的 Eigen 16 字节 malloc 对齐 ABI；没有关闭检测或屏蔽错误。
自定义 AVX/PCL 构建应重新核对其对齐 ABI。
