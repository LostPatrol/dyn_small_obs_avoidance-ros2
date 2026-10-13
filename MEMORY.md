<!-- 当前有效算法、接口、逐机部署与开发前必须了解的限制；过程和日志只放报告。 -->
# 当前基线

- 竖直绕行偏好：公共 `planner.yaml` 启用 `search.vertical_distance_weight=10.0`，裸core/节点默认0保持旧行为。代价增加 `weight*∫|vz|dt`，普通原语和shot按速度零点精确累计上下路程；新增高度差启发项 `weight*abs(goal.z-z)` 因诱发目标高度附近周期反向已暂时注释，仅保留原启发式。启用时比较既有shot时长候选的完整代价，并将完整候选与加权搜索队列比较；预算/节点耗尽已有完整可行候选则返回该候选，否则仍失败。既有lambda=5和position-only hash保留，不保证全局最优或随权重单调变平。修改参数须重启；不解决坐标偏差、盲区或输入上升速度形成的拱形。

- 独立入口保留goal和旧路径受阻事件；额外在距上次实际尝试经过replan_interval秒（默认0.2，名义5Hz），或实际三维位置变化>=replan_distance米（默认1）时重规划。时间/位移触发从实际odom p/v规划，绕过执行索引节流；每次尝试包括失败均重置时间/位移基准。planning_rate默认10Hz控制定时检查及输入失效诊断。输入有效后可补首次规划或重试初次失败；实际起点距目标<0.3m仅在成功曲线仍有效时早退；原生失败/输入故障撤销后，恢复允许一次actual状态规划，后续失败再按既有时间重试。
- 上游是HKU-MARS dyn_small_obs_avoidance，不是hubernikus的Python速度调制库。d47a718恢复ROS1事件策略；3bce7ae加入时间/位移刷新，使旧绕行在地图中障碍淘汰后可自动缩短。仍按旧路径第0、40、80……点检查碰撞，失败保留旧路径及已置位replan_flag，成功才清标记。
- 独立入口先init=true、NO_PATH才init=false；内部旧路径检查/执行索引缓存按0.01s采样，公共输出仍由sample_step控制。launch默认将time_index映射到/demo_node/trajectory_time_index：原goal/受阻事件有索引时绝对差<10节流，从旧路径index+3及差分速度规划；新增时间/位移触发及无索引时用实际odom p/v。仅防未初始化读取和确定越界。
- 普通恒加速度原语直接在现有核心用净空余量证书自适应二分：Da+Db>=2r+h*(|va|+|vb|)/2接受区间，否则查询中点；14层仍未证安全则拒绝原语，沿用既有搜索期限。核心数学见docs/PRIMITIVE_CLEARANCE_MATH.md。节点起点距离在候选间复用，PCL float坐标/距离舍入作微小保守修正。普通候选、代价、hash/queue和双树保持；首init扩展在dynamic=false且仍同cell时，短候选临时节点不入空间hash，保留0.04..0.8s全部20个时长，避免闭合root误裁剪短coast。常规同cell异速仍按旧position-only hash裁剪。
- 当前YAML与core默认统一：horizon=100m、resolution=voxel_size=0.1m、safe_distance=0.25m、max_tau=0.6s、init_max_tau=0.8s、search_budget=0.3s、input_timeout=0.5s。直线与自主避障分开配置；本库search.*是避障单源，主工程control.yaml负责直线。避障scalar max_vel/max_acc=1/2；方向限额vxy/vz=1/0.2m/s、axy/az=2/2m/s²（水平范数/垂直绝对值）。旧算法加速度2是逐轴值，可产生水平2√2；当前水平合范数明确限制为2。普通原语保持0.6s和5×5×5候选；当某轴半幅加速度从静止的末速度超过该轴速度上限时，仅该轴根据当前速度和tau的可行加速度区间取负全/半/零/正半/全5值，默认仅Z需适配。普通同cell/hash策略未改，不保证已逼近障碍的起点可恢复。恒加速度原语与三次shot在原规划持续时间内验证限额；shot水平速度范数极值解析求根，不依赖执行层retime。
- scalar search.max_vel/max_acc保留兼容：显式launch scalar覆盖未显式给出的方向CLI值，方向CLI优先；直接ROS scalar未给方向时同样填两方向。C++ SearchConfig scalar仍是额外逐轴上限，直接使用方向限额时应保证scalar不小于方向限额。所有参数启动复制，修改须重启。三个launch均CLI→同一planner.yaml→节点默认；planning_frame=odom、stamp_clock=receive、下界[-50,-50,-2]，不保留正式桥专属profile。
- 正式桥与独立手动入口共用原生双树、目标/时间/位移/旧路径碰撞触发及init=true/NO_PATH重试。service_only和latest_observation已删除；cloud.map_mode仅保留可读兼容值dual_tree。桥配置仅coordinate_mode、odometry_topic=/mavros/local_position/odom、preview_enabled；原始点云由原生node直接接收。
- SetGoal服务(request header/goal/target_id/active，response accepted/planner_session/goal_revision/detail)是持久原生目标生命周期。active=true要求非空ID、合法frame和位置；同ID同坐标幂等。active=false保留地图并停止搜索，非空取消ID必须匹配活跃目标。新目标/取消清旧路径和执行索引。手动PoseStamped goal继续用同一逻辑，每次命令分配manual:session:counter ID。
- 原生PlanResult包括planner_session、target_id及原sequence/goal_revision；新增max_velocity_xy/max_velocity_z/max_acceleration_xy/max_acceleration_z四个float64实际限额，由core.config最终scalar与方向min生成，所有结果含失败、idle心跳及PlanMotion均填写。桥/执行直接消费，不另存避障限额；主工程消费者需要重建新接口。现有timer在idle或有效近目标早退时每秒发布空target_id、status=9、planning_ms=0且无segments的身份心跳，使独立原生重启的新session可见；心跳不搜索、不撤销成功曲线，消费端必须按空ID识别身份消息。PlanMotion保留PLAN与CHECK_LINE/VALIDATE_TRAJECTORY，查询不替换原生目标/身份，响应result target_id为空。几何服务离散调用旧isSafe，不提供连续净空证明。

# 部署

- KT_HORIZON对应search.tree_period，当前YAML及C++默认均为20；已同步drone-refresh(.186)及drone-new(.169)源码和Release安装目录，下次启动生效。
- 原生目标生命周期、方向动力学限额及删除桥profile已在开发机、refresh和hp-desktop更新；new仍待同步此基线和主工程接口消费者。新增竖直路程代价已在开发机、refresh原生构建并安装；refresh保留现场边界 `[-20,-20,0.2]..[20,20,2]`，安装配置权重仍为10；高度差启发项已关闭并重启refresh独立规划器。new与hp-desktop仍待同步竖直代价及此次启发项关闭。
- hp-desktop已于2026-10-13同步b527fbc及主工程62ce78e，原生Release重建独立库、桥、共享接口及控制消费者；当前新基线也可在笔记本使用。工作区为`/home/scq/dyn_small_obs_avoidance-ros2`，旧源码与安装归档在`/home/scq/backups/code-sync-20261013/`。

- drone-new=.169，drone-refresh=.186；两机项目路径均/home/nvidia/dyn_small_obs_avoidance-ros2。净空证书代码24e518f已同步两机，均完成Release安装、核心/DDS测试及固定现场点云节点回放，安装核心库SHA256一致。186曾出现源码空文件及损坏构建缓存，已恢复；同步时须核实实际源码与安装库，不能仅看Git HEAD。持续传感器CPU量化复测未记录。
- 时间/位移重规划3bce7ae及DDS测试90024ae已同步两机；当前默认参数replan_interval=0.2s、replan_distance=1.0m，C++默认及YAML已同步两机并完成Release安装。
- 当前两机由Odin host SDK直接发布cloud_slam_down（0.1m降采样、0.4m近身剔除、auto球心），独立Python已退出生产链；二次blind_radius=0不等于完整输入链无近身剔除。驱动逐机实际运行状态以主工程MEMORY和现场进程为准。

# 未解决限制

- 原先普通原语端点间漏检已由净空证书修复；末端shot与几何服务仍为离散检查，不能宣称整条含shot轨迹具备连续净空证明。
- 独立/kino_path是最后成功预览，仅规划成功才更新；时间刷新仍受输入有效性、距目标<0.3m早退、10Hz检查及搜索耗时影响，并非严格5Hz输出。输入新鲜度和当前可执行性不能仅根据旧Path或最后一次成功判断。
- 双树按输入帧累计而非墙钟到期，移动障碍可留残影；上游球形剔除会删除真实近身障碍。正式桥不再拒绝unknown（用户接受），有效Tag修正仍未支持。质心0.25m不保证每个raw点/真实表面0.25m。
- 独立搜索TIMEOUT会清空搜索进度，下一点云从头重试；单线程搜索同时延缓点云处理，使按帧淘汰的历史更久保留。普通原语证书是对ROS1稀疏采样漏检的限定修正，事件策略对齐不等于内核完整原样还原。调大0.3s搜索预算仍需考虑wrapper的0.5s输入时限及搜索后检查。
- 两机处于同一ROS域且使用同名Odin话题时可能串流；调试必须先确认目标机数据源。隔离域回放与台架结果不能替代真实传感器长时负载或实飞验收。
