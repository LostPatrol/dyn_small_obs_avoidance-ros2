<!-- 当前有效算法、接口、逐机部署与开发前必须了解的限制；过程和日志只放报告。 -->
# 当前基线

- 独立入口保留goal和旧路径受阻事件；额外在距上次实际尝试经过replan_interval秒（默认1），或实际三维位置变化>=replan_distance米（默认1）时重规划。时间/位移触发从实际odom p/v规划，绕过执行索引节流；每次尝试包括失败均重置时间/位移基准。planning_rate默认10Hz控制定时检查及输入失效诊断。输入有效后可补首次规划或重试初次失败；实际起点距目标<0.3m仍早退。
- 上游是HKU-MARS dyn_small_obs_avoidance，不是hubernikus的Python速度调制库。d47a718恢复ROS1事件策略；3bce7ae加入时间/位移刷新，使旧绕行在地图中障碍淘汰后可自动缩短。仍按旧路径第0、40、80……点检查碰撞，失败保留旧路径及已置位replan_flag，成功才清标记。
- 独立入口先init=true、NO_PATH才init=false；内部旧路径检查/执行索引缓存按0.01s采样，公共输出仍由sample_step控制。launch默认将time_index映射到/demo_node/trajectory_time_index：原goal/受阻事件有索引时绝对差<10节流，从旧路径index+3及差分速度规划；新增时间/位移触发及无索引时用实际odom p/v。仅防未初始化读取和确定越界。
- 普通恒加速度原语直接在现有核心用净空余量证书自适应二分：Da+Db>=2r+h*(|va|+|vb|)/2接受区间，否则查询中点；14层仍未证安全则拒绝原语，沿用既有搜索期限。核心数学见docs/PRIMITIVE_CLEARANCE_MATH.md。节点起点距离在候选间复用，PCL float坐标/距离舍入作微小保守修正。候选、代价、hash/queue、双树及shot保持。
- 当前YAML：horizon=100、resolution=voxel_size=0.1、safe_distance=0.25、max_vel=max_acc=2、max_tau=0.6、collision_step=0.05、search_budget=0.3s、input_timeout=0.5s。参数在启动时复制，修改须重启。
- Odin launch默认订阅/odin1/cloud_slam_down和/odin1/odometry，frame=odom、stamp_clock=receive、下界[-50,-50,-2]，上界来自YAML[50,50,10]；launch默认blind_radius=0覆盖YAML的0.3。勿混淆源码缺省bounds与实际启动覆盖。
- PlanMotion服务保持原Task38接口：service_only关闭独立goal/cloud规划及诊断定时器；PLAN按请求实际p/v直接init=false搜索。正式桥使用latest_observation完整快照替换；独立入口仍50帧双树累计。CHECK_LINE/VALIDATE_TRAJECTORY离散调用旧isSafe，不提供连续净空证明。
- 主工程接口3.4及执行层PD+DOB、水平1/垂直0.2m/s和加速度0.35/0.15m/s²保持；超包线完整曲线按统一时间缩放放慢，空间路径不变。本项目修复不修改主工程。

# 部署

- drone-new=.169，drone-refresh=.186；两机项目路径均/home/nvidia/dyn_small_obs_avoidance-ros2。净空证书代码24e518f已同步两机，均完成Release安装、核心/DDS测试及固定现场点云节点回放，安装核心库SHA256一致。186曾出现源码空文件及损坏构建缓存，已恢复；同步时须核实实际源码与安装库，不能仅看Git HEAD。持续传感器CPU量化复测未记录。
- 时间/位移重规划3bce7ae及DDS测试90024ae已同步两机，Release安装和每机12项ROS/DDS、2项C++检查通过；两机可执行文件、核心库及安装YAML的SHA256一致。新增默认参数replan_interval=1.0s、replan_distance=1.0m；service_only模式不自动重规划。
- /home/nvidia/down_samp_exe.py为现场独立点云发布者，不由本项目/桥自动启动；此前169配置为0.1m降采样、0.4m球形剔除、auto球心。二次blind_radius=0不等于完整输入链无近身剔除。

# 未解决限制

- 原先普通原语端点间漏检已由净空证书修复；末端shot与几何服务仍为离散检查，不能宣称整条含shot轨迹具备连续净空证明。
- 独立/kino_path是最后成功预览，仅规划成功才更新；新增时间刷新仍受输入有效性、距目标<0.3m早退及搜索耗时影响，并非严格1Hz输出。输入新鲜度和当前可执行性不能仅根据旧Path或最后一次成功判断。
- 双树按输入帧累计而非墙钟到期，移动障碍可留残影；上游球形剔除会删除真实近身障碍。正式桥拒绝unknown及有效Tag修正，稀疏观察可使航线不可执行。
- 独立搜索TIMEOUT会清空搜索进度，下一点云从头重试；单线程搜索同时延缓点云处理，使按帧淘汰的历史更久保留。普通原语证书是对ROS1稀疏采样漏检的限定修正，事件策略对齐不等于内核完整原样还原。调大0.3s搜索预算仍需考虑wrapper的0.5s输入时限及搜索后检查。
- 两机处于同一ROS域且使用同名Odin话题时可能串流；调试必须先确认目标机数据源。隔离域回放与台架结果不能替代真实传感器长时负载或实飞验收。
