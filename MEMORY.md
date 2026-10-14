<!-- 当前有效算法、接口、逐机部署与开发前必须了解的限制；过程和日志只放报告。 -->
# 当前基线

- 公共planner.yaml以用户选定的“original default values”红线为基础调优：scalar及方向v/a均2、w_time=10、vertical_distance_weight=10、lambda=5、safe_distance=.35、tree_period=20、XYZ边界[-20,-20,.2]至[20,20,1.8]、预算.5、input_timeout=1；tau=.6/init_tau=.8、resolution/voxel=.1、horizon100保持。当前盒子最大位移56.59m，horizon100不会触发局部返回。此为数值对照后的用户配置，当前水平范数及shot/碰撞修复仍保留，不是旧逐轴算法完整还原。裸SearchConfig/未加载YAML节点默认未改，入口实际以YAML/CLI回读为准。逐项作用见docs/planner_parameters.md。
- 原生身份心跳与规划频率解耦：健康活动目标等待事件/时空刷新时同样约每秒发送空target_id身份消息，不搜索、不替换曲线。replan_interval和replan_distance分别接受正无穷.inf关闭对应强制触发；目标/旧路径受阻事件仍有效，输入故障仍报告失败。两阈值均.inf的远目标/位移/受阻DDS回归已覆盖，正式桥无需放宽身份超时。

- Task5原生修复：实际状态规划直接`init=false`，不再每次强制零加速度coast；只有旧路径执行索引模式保留init及NO_PATH重试。超速实测起点保留真实p/v，先用满足方向/scalar加速度限额、连续净空与位置边界的恒加速度首段制动回名义包线，该单调减速首段允许仍超名义速度；目标速度和后续段须在包线内，不能制动/接续则失败。窄Z分支在原五档中补充未重复且合法的`-vz/max_tau`精确停速候选，保留125原候选、最多150，避免微小竖直漂移一直积累。失败detail区分越界、起终点净空、超速制动/接续失败、预算/节点耗尽。`odin.launch.py`按SDK世界系linear twist读取，通用/MAVROS入口仍为body twist；Odin专属`odin_launch.ros__parameters.odometry_twist_in_body`可覆盖入口默认，显式CLI优先。部署以主工程逐机记录为准。

- 可选竖直绕行偏好：裸core/节点vertical_distance_weight默认0，当前公共YAML为10；正权重代价增加 `weight*∫|vz|dt`，普通原语和shot按速度零点精确累计上下路程；新增高度差启发项 `weight*abs(goal.z-z)` 因诱发目标高度附近周期反向已暂时注释，仅保留原启发式。启用时比较既有shot时长候选的完整代价，并将完整候选与加权搜索队列比较；预算/节点耗尽已有完整可行候选则返回该候选，否则仍失败。裸core/节点默认lambda=1，公共YAML为5，显式CLI优先，position-only hash保留；tie因子1.0001只乘一次。半速时长下界并非可采纳启发保证，仍不保证全局最优或随权重单调变平。修改参数须重启；不解决坐标偏差、盲区或输入上升速度形成的拱形。

- 独立入口保留goal和旧路径受阻事件；额外在距上次实际尝试经过replan_interval秒（默认0.2，名义5Hz），或实际三维位置变化>=replan_distance米（默认1）时重规划。时间/位移触发从实际odom p/v规划，绕过执行索引节流；每次尝试包括失败均重置时间/位移基准。planning_rate默认10Hz控制定时检查及输入失效诊断。输入有效后可补首次规划或重试初次失败；实际起点距目标<0.3m仅在成功曲线仍有效时早退；原生失败/输入故障撤销后，恢复允许一次actual状态规划，后续失败再按既有时间重试。
- 上游是HKU-MARS dyn_small_obs_avoidance，不是hubernikus的Python速度调制库。d47a718恢复ROS1事件策略；3bce7ae加入时间/位移刷新，使旧绕行在地图中障碍淘汰后可自动缩短。仍按旧路径第0、40、80……点检查碰撞，失败保留旧路径及已置位replan_flag，成功才清标记。
- 内部旧路径检查/执行索引缓存按0.01s采样，公共输出仍由sample_step控制。launch默认将time_index映射到/demo_node/trajectory_time_index：原受阻事件有索引时绝对差<10节流，从旧路径index+3及差分速度规划并保留init=true/NO_PATH重试；目标/时间/位移触发及无索引时用实际odom p/v和自由首段加速度。
- 普通恒加速度原语直接在现有核心用净空余量证书自适应二分：Da+Db>=2r+h*(|va|+|vb|)/2接受区间，否则查询中点；14层仍未证安全则拒绝原语，沿用既有搜索期限。核心数学见docs/PRIMITIVE_CLEARANCE_MATH.md。节点起点距离在候选间复用，PCL float坐标/距离舍入作微小保守修正。普通候选、代价、hash/queue和双树保持；首init扩展在dynamic=false且仍同cell时，短候选临时节点不入空间hash，保留0.04..0.8s全部20个时长，避免闭合root误裁剪短coast。常规同cell异速仍按旧position-only hash裁剪。
- YAML与core共用horizon=100m、resolution=voxel_size=.1m、max_tau=.6s、init_max_tau=.8s；YAML搜索预算.5s，裸core仍为.3s。本库search.*是避障单源，主工程control.yaml负责直线。YAML当前使用上述用户调优配置；裸core仍为safe_distance=.25、scalar v/a=1/2、方向vxy/vz=1/.2、axy/az=2/2。旧算法加速度2是逐轴值，可产生水平2√2；当前水平合范数明确限制为2。普通原语保持.6s和原5×5×5候选；窄速度轴按可行加速度区间采样，Z额外包含可用的精确停速档。普通同cell/hash策略未改，不保证已逼近障碍的起点可恢复。除实测超速恢复首段外，普通原语与三次shot在原规划持续时间内验证名义速度限额；全部段仍遵守加速度限额。shot水平速度范数极值解析求根，不依赖执行层retime。
- scalar search.max_vel/max_acc保留兼容：显式launch scalar覆盖未显式给出的方向CLI值，方向CLI优先；直接ROS scalar未给方向时同样填两方向。C++ SearchConfig scalar仍是额外逐轴上限，直接使用方向限额时应保证scalar不小于方向限额。所有参数启动复制，修改须重启。三个launch均CLI→同一planner.yaml→节点默认；planning_frame=odom、stamp_clock=receive，空间边界以当前YAML为准，不保留正式桥专属profile。
- 正式桥与独立手动入口共用原生双树、目标/时间/位移/旧路径碰撞触发及实际状态自由首段搜索。service_only和latest_observation已删除；cloud.map_mode仅保留可读兼容值dual_tree。桥配置仅coordinate_mode、odometry_topic=/mavros/local_position/odom、preview_enabled；原始点云由原生node直接接收。
- SetGoal服务(request header/goal/target_id/active，response accepted/planner_session/goal_revision/detail)是持久原生目标生命周期。active=true要求非空ID、合法frame和位置；同ID同坐标幂等。active=false保留地图并停止搜索，非空取消ID必须匹配活跃目标。新目标/取消清旧路径和执行索引。手动PoseStamped goal继续用同一逻辑，每次命令分配manual:session:counter ID。
- 原生PlanResult包括planner_session、target_id及原sequence/goal_revision；新增max_velocity_xy/max_velocity_z/max_acceleration_xy/max_acceleration_z四个float64实际限额，由core.config最终scalar与方向min生成，所有结果含失败、idle心跳及PlanMotion均填写。桥/执行直接消费，不另存避障限额；主工程消费者需要重建新接口。现有timer在idle及健康活动目标等待时每秒发布空target_id、status=9、planning_ms=0且无segments的身份心跳，使独立原生重启的新session可见；心跳不搜索、不撤销成功曲线，消费端必须按空ID识别身份消息。PlanMotion保留PLAN与CHECK_LINE/VALIDATE_TRAJECTORY，查询不替换原生目标/身份，响应result target_id为空。几何服务离散调用旧isSafe，不提供连续净空证明。

# 部署

- 心跳修复`aeb07b9`及当前用户调优YAML已在开发机及refresh安装并回读；本次只改配置/说明，沿用已构建的Release代码。refresh保留现场1s/100m刷新、/odin1/odometry_highfreq和Odin世界系twist，已重启规划器并恢复[15,0,1]预览，结果REACH_END。new及hp-desktop未同步本次更新，下次连接须补齐。refresh上次调整前配置/安装归档位于/home/nvidia/ros2-ardupilot-maintenance/heartbeat-original-values-20261014/，本次调整前红线配置位于/home/nvidia/ros2-ardupilot-maintenance/planner-tuning-20261014/；命名stash仅供恢复特定现场值，勿恢复整份旧YAML覆盖当前调优参数。

- KT_HORIZON对应search.tree_period；当前公共YAML和裸core均为20帧，区别于曾短暂部署的50帧红线对照。new未应用此次调优及心跳修复，下次连接须一起同步。
- Task5代码基线`333dfbe`（含`ca769f0`）曾在开发机、refresh和hp-desktop原生Release部署，主工程控制代码`a76a069`同步重建；new仍待同步原生接入、接口消费者及后续修复。Task5当时refresh权重30、Z[.6,1.4]、lambda1为历史现场，不覆盖当前用户选定的红线配置。静态观察未解锁、零setpoint，结束后所有本次机载/避障节点停止。两端控制26/26、core28/28，开发机最终wrapper35/35通过。
- 最终对原bag的107条成功计划固定输入子集107/107成功（不含原69条失败），refresh ARM64搜索中位43.994ms、max176.898ms；未叠加全传感器长期负载。最终两次自主SITL均26/26规划成功、完成任务，但表面净空.29570/.29367m仍未过>.35m，不能称完整实飞验收。详细分析/执行报告保存在主工程agent/report本地目录。
- 本次更新前的HEAD、现场差异和install分别备份于refresh `/home/nvidia/ros2-ardupilot-maintenance/task5-avoidance-20261013/`、hp `/home/scq/backups/task5-avoidance-20261013/`；refresh命名stash只保存旧现场YAML。不要恢复整份旧YAML覆盖新默认或复制ARM/x86安装产物。

- drone-new=.169，drone-refresh=.186；两机项目路径均/home/nvidia/dyn_small_obs_avoidance-ros2。净空证书代码24e518f已同步两机，均完成Release安装、核心/DDS测试及固定现场点云节点回放，安装核心库SHA256一致。186曾出现源码空文件及损坏构建缓存，已恢复；同步时须核实实际源码与安装库，不能仅看Git HEAD。持续传感器CPU量化复测未记录。
- 时间/位移重规划3bce7ae及DDS测试90024ae已同步两机；当前默认参数replan_interval=0.2s、replan_distance=1.0m，C++默认及YAML已同步两机并完成Release安装。
- 当前两机由Odin host SDK直接发布cloud_slam_down（0.1m降采样、0.4m近身剔除、auto球心），独立Python已退出生产链；二次blind_radius=0不等于完整输入链无近身剔除。驱动逐机实际运行状态以主工程MEMORY和现场进程为准。

# 未解决限制

- 竖直路程权重是软代价，加权启发及位置hash仍可选择竖直绕行。历史lambda1配合tie修复的固定有限墙场景可保持同高横绕，窄横向范围仍能竖向逃逸；lambda5曾在闭环柱场景反复制动后失去可行接续，lambda1对照完成任务。当前lambda5/weight10用户调优配置尚未闭环验收。SITL旧基线实测最小净空未达主工程.35m指标。自由首段与精确停速候选减小空图拱形，但输入竖直速度仍须物理制动，不能宣称所有蛇形/竖直偏好已修复。
- 原先普通原语端点间漏检已由净空证书修复；末端shot与几何服务仍为离散检查，不能宣称整条含shot轨迹具备连续净空证明。
- 独立/kino_path是最后成功预览，仅规划成功才更新；时间刷新仍受输入有效性、距目标<0.3m早退、10Hz检查及搜索耗时影响，并非严格5Hz输出。输入新鲜度和当前可执行性不能仅根据旧Path或最后一次成功判断。
- 双树按输入帧累计而非墙钟到期，移动障碍可留残影；上游球形剔除会删除真实近身障碍。正式桥不再拒绝unknown（用户接受），有效Tag修正仍未支持。体素质心净空不保证每个raw点或真实表面的同等净空。
- 独立搜索TIMEOUT且没有完整候选时会清空搜索进度，下一次尝试从头搜索；单线程搜索同时延缓点云处理，使按帧淘汰的历史更久保留。普通原语证书是对ROS1稀疏采样漏检的限定修正，事件策略对齐不等于内核完整原样还原。当前.5s搜索预算配套1s输入时限，搜索后仍检查输入；主工程默认结果年龄上限.8s，预算不含建图/输出/DDS，仍不是5Hz输出保证。
- 两机处于同一ROS域且使用同名Odin话题时可能串流；调试必须先确认目标机数据源。隔离域回放与台架结果不能替代真实传感器长时负载或实飞验收。
## 原算法参考

`upstream-ros2-port/dyn_small_obs_avoidance-main` 为本地制作的ROS1原算法ROS2接口适配参考，并非上游官方ROS2发布。
它保留BUGS.md中记录的速度赋值、未初始化边界及NEAR_END等行为；不作为本库完整可执行终点的等价基线。
根目录构建应显式使用 `--base-paths path_searching path_planning`，避免与参考目录同名包冲突。

## 仓库身份与路径

- GitHub仓库名为`LostPatrol/dyn_small_obs_avoidance-ros2-modified`，根目录为维护中的修改版；参考适配保存在`upstream-ros2-port/dyn_small_obs_avoidance-main`。
- 开发机及飞机项目根路径继续使用`dyn_small_obs_avoidance-ros2`，不要因远程改名移动部署目录或重建。开发机origin已更新；refresh、new及hp-desktop的已有克隆下次连接时需将origin更新为新URL，无需因改名重启服务。
- 参考目录随改名首次纳入Git；本地已有参考版build/install缓存可能含旧路径，需要运行参考版时在独立工作区重新构建，不使用旧安装产物。
