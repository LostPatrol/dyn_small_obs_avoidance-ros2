<!-- Planner configuration reference: selected YAML profile, search effects and wrapper triggers. -->
# 规划参数说明

公共配置为 [`path_planning/config/planner.yaml`](../path_planning/config/planner.yaml)，
独立入口与正式桥共用 `path_planning.ros__parameters`。下表列出本次选择的“原默认值”配置；
它恢复指定数值，不代表完整恢复旧算法：当前仍保留方向范数限额、原语净空检查、超速制动等修复。
裸 C++ [`SearchConfig`](../path_searching/include/path_searching/kinodynamic_astar.h) 和未加载 YAML
的节点仍有自己的默认值（例如 `lambda_heu=1`、`max_vel_z=0.2`、`safe_distance=0.25`、
`tree_period=20`），不能据此推断 launch 的实际参数。

## 搜索与路径形状

以下参数均带 `search.` 前缀。距离、时间、速度和加速度单位分别为 m、s、m/s、m/s²。

| 参数 | 选定值 | 实际作用与调整方向 |
| --- | --- | --- |
| `max_tau` | 0.6 | 普通常加速度运动原语时长。增大改变单次展开的位移和转弯颗粒度；不是输出采样周期。 |
| `init_max_tau` | 0.8 | 显式 `init=true` 首段固定加速度的最大时长，搜索20个候选时长（0.04～0.8）。实际状态规划用 `init=false`；旧路径执行索引模式才保留此首段。 |
| `max_vel` | 2.0 | 兼容的逐轴速度绝对上限，与方向限额同时生效。 |
| `max_vel_xy` / `max_vel_z` | 2.0 / 2.0 | 水平合速度范数 / 垂直速度绝对上限。降低通常增加通行时间，并改变转弯/接续的可行性；已有起点超速时保留真实速度并尝试受限首段制动。 |
| `max_acc` | 2.0 | 兼容的逐轴加速度绝对上限，与方向限额同时生效。 |
| `max_acc_xy` / `max_acc_z` | 2.0 / 2.0 | 水平合加速度范数 / 垂直加速度绝对上限。降低会限制转弯和制动能力。原逐轴2允许水平合值达到 `2√2`，当前水平范数2并不等价。 |
| `w_time` | 10.0 | 基础代价为 `∫(||a||² + w_time)dt`。增大更偏向缩短时间，但不会提高硬速度限额。 |
| `vertical_distance_weight` | 0.0 | 每米累计上下移动的额外代价 `weight × ∫|vz|dt`；升后降分别累计。0关闭该额外代价；增大只是软偏好，不保证恒高。 |
| `horizon` | 100.0 | 离起点的位移达到此值可返回局部 `REACH_HORIZON`，不是累计路径长度，也不是停车保证。 |
| `lambda_heu` | 5.0 | 启发代价权重；较大通常更贪心，可能改变绕行方向和可行性，不保证更短、更平或全局最优。当前平局因子只乘一次 `1.0001`，有效倍率为 `5.0005`，不使用 `9.9995`。 |
| `resolution` | 0.1 | 位置 hash 栅格边长。增大减少状态数量但合并更多不同速度状态；减小增加搜索开销。不是地图体素尺寸。 |
| `time_resolution` | 0.8 | 核心 `dynamic=true` 的时间 hash 分箱宽度；当前 ROS 包装层用 `dynamic=false`，此值不启用动态障碍预测。 |
| `allocate_num` | 100000 | 启动时预分配搜索节点池；耗尽返回 `NODE_LIMIT`。增大主要增加内存和可探索状态量。 |
| `search_budget` | 0.3 | 每次核心搜索的合作式墙钟预算；耗尽返回 `TIMEOUT`。不含建图、输出构造和 DDS；旧索引模式的重试各有一次预算。 |

显式 launch scalar CLI（如 `search.max_vel:=1.5`）会填充未显式给出的对应方向 CLI 值；
方向 CLI 更优先；ROS节点会把最终 scalar 提高到不小于各方向限额，避免其意外裁剪显式方向配置。
直接 C++ 配置中 scalar 始终是额外逐轴上限。参数源优先级为显式 CLI → YAML → 默认值。
固定动作的半档速度增量为 `max_acc * max_tau / 2`；它与速度限额共同决定可用巡航档位。
因此单独提高加速度、却保留低速度上限，可能裁掉更多动作，并不保证巡航更快或路径更直。

## 地图、净空与空间范围

| 参数 | 选定值 | 实际作用 |
| --- | --- | --- |
| `search.safe_distance` | 0.45 | 到双树中最近体素质心的距离严格小于此值才拒绝。增大减少可通行空间；没有额外体素或采样膨胀，不等于所有原始点的表面净空。 |
| `search.voxel_size` | 0.1 | 累积地图体素边长；增大减小地图规模，同时改变障碍质心表示。 |
| `search.collision_step` | 0.05 | 末端三次连接和几何服务的离散检查最大行程间隔，须不大于 `safe_distance`。减小增加检查量；普通原语另用自适应净空证书，此参数不设其固定采样间隔。 |
| `search.tree_period` | 50 | 每棵树累计的有效点云帧数，两棵 KD-tree 轮换。历史大致覆盖50～100次有效更新；10Hz时约5～10s，不是墙钟过期。 |
| `search.max_cloud_points` | 500000 | 单帧输入及两棵体素化树合计点数上限；超限使地图失效，不自动抽稀绕过上限。 |
| `search.lower` / `search.upper` | `[-20,-20,0.2]` / `[20,20,1.3]` | 世界系 XYZ 的闭区间边界，约束起点、目标、普通原语和末端连接。Z范围限制高度，但不会产生恒高软偏好。 |
| `cloud.map_mode` | `dual_tree` | 兼容配置项，当前只接受原生双树模式。没有另一种快照地图实现。 |
| `cloud.blind_radius` | 0.0 | 累积前剔除球内点；0关闭。真实障碍也会被剔除，与碰撞半径独立。 |
| `cloud.blind_origin_offset` | `[0,0,0]` | 盲区球心相对 odom 位姿原点的 child 坐标系偏移，按匹配姿态旋转。 |
| `cloud.blind_pose_tolerance` | 0.05 | 开启盲区时，点云与最近历史 odom 源时标的最大允许差；不插值，两路须同源时钟。 |
| `cloud.blind_wait` | 0.15 | 缺匹配 odom 时缓存一帧点云的最长等待，须不超过 `input_timeout`；超时地图失效。 |

普通原语使用净空余量与弧长上界验证区间；末端连接和几何服务仍为离散检查，
整条含末端连接的曲线不具备连续净空证明。未观测空间没有自由空间证明；双树也不预测障碍运动。

## 输入、重规划与输出

以下为包装层参数，独立于核心搜索代价。
表中为本机公共YAML值；refresh保留现场 `replan_interval=1.0`、`replan_distance=100.0` 和 highfreq 输入映射。

| 参数 | 当前配置 | 实际作用 |
| --- | --- | --- |
| `planning_frame` | `odom` | cloud、odom pose、goal 的 `frame_id` 须精确匹配；节点不做 TF 变换。 |
| `odometry_twist_in_body` | 通用 `true`，Odin `false` | true将 child 系线速度按姿态转至世界系；false直接按世界系读取。选错会改变搜索初速度。 |
| `stamp_clock` | `receive` | `receive`只查接收年龄，保留源时标，不能测传感器积压；`ros`还查源 ROS 时标年龄及最多0.05s未来偏移。 |
| `input_timeout` | 0.5 | 点云和 odom 的接收年龄上限；`ros`模式也约束源年龄。增大容忍旧输入，不改善数据延迟。 |
| `planning_rate` | 10.0 | 定时检查频率，控制时间/位移触发、输入失效诊断和身份心跳检查；不是固定规划输出频率。 |
| `replan_interval` | 0.2 | 距上次实际尝试经过此秒数时刷新；正无穷 `.inf` 关闭这一触发，有限正数启用。 |
| `replan_distance` | 1.0 | 实际三维位置距上次尝试移动至少此米数时刷新；正无穷 `.inf` 关闭这一触发。 |
| `sample_step` | 0.02 | 公共输出最大时间采样间隔，包含精确首尾和段边界；减小增加序列化和 DDS 开销，不提高碰撞检查分辨率。 |
| `output.max_samples` | 10000 | 输出全曲线采样数量上限，超限返回空轨迹 `INVALID_INPUT`，不截短。 |
| `output.max_duration` | 60.0 | 输出全曲线时长上限；超限同样返回空轨迹，也限制几何曲线验证。 |
| `output.build_budget` | 0.1 | 输出构造的合作式墙钟预算，不含搜索和 DDS；超限返回空轨迹 `TIMEOUT`。 |

原生事件触发与时间/位移阈值独立：新目标在输入就绪时规划；点云更新检查旧路径第0、40、80……点，
旧路径受阻则请求重规划。这份内部路径按0.01s采样，即检查间隔约0.4s轨迹时间，
不同于 `sample_step`，也不是固定空间间隔。有效成功曲线且实际位置距目标小于0.3m时跳过重规划；
曲线撤销后输入恢复可从实际状态尝试恢复。

可选 `time_index` 输入保留旧路径模式：受阻事件可能因执行索引绝对变化小于10而延后，
并从缓存 `index+3` 位置和差分速度规划。新目标清索引；时间/位移刷新绕过该节流并从实际 odom p/v规划。
每次实际尝试（包括失败）重置时间和位移基准。若两阈值均为 `.inf`，仍保留目标/受阻事件；
安全旧绕行不会仅因障碍消失而缩短，且历史障碍必须先经双树轮换淘汰。

身份心跳约每秒发布一次，与搜索阈值独立；它携带空 `target_id`、status=9且无曲线，
只表示进程身份，不是规划失败或新执行轨迹。`/kino_path` 只更新成功预览，当前结果应看 `/plan_result`。

## 修改入口

- 参数启动读取并复制；修改后重启节点。默认 launch 加载安装目录的 YAML，修改源码配置后须重建，
  或使用 `config:=/absolute/path/planner.yaml` 指定外部文件。
- `odin_launch` / `planner_launch` 的 `cloud/odom/goal/time_index` 是入口话题映射；
  `avoidance_bridge` 的 `coordinate_mode/odometry_topic/preview_enabled` 是主工程桥接口设置，均不调整核心路径代价。
- 搜索实现见 [`kinodynamic_astar.cpp`](../path_searching/src/kinodynamic_astar.cpp)，
  输入/重规划/输出见 [`path_planning.cpp`](../path_planning/src/path_planning.cpp)，
  CLI优先级见 [`parameter_config.py`](../path_planning/launch/parameter_config.py)。
