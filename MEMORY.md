<!-- 当前有效算法、接口、逐机部署与开发前必须了解的限制；过程和日志只放报告。 -->
# 当前基线

- 独立入口已恢复ROS1事件重规划：goal在起点就绪时规划；有效点云更新后检查整个旧路径第0、40、80……点，受阻才规划；实际起点距目标<0.3m早退。失败保留旧路径及已置位replan_flag，成功才清标记。首次失败无旧路径、goal先于odom时不会自动补规划。planning_rate只控制输入失效诊断。
- 独立入口先init=true、NO_PATH才init=false；内部旧路径检查/执行索引缓存按0.01s采样，公共输出仍由sample_step控制。launch默认将time_index映射到/demo_node/trajectory_time_index：有索引时绝对差<10节流，从旧路径index+3及差分速度规划；无索引用实际odom p/v。保留原作者整个旧路径检查、索引节流对新目标也生效等行为，仅防未初始化读取和确定越界。
- path_searching仍为169选用的旧离散核心：普通原语最多42次采样（ceil(0.6×sqrt(3)×2/0.05)），没有引入连续距离求根或改写搜索限值。
- 当前YAML：horizon=100、resolution=voxel_size=0.1、safe_distance=0.25、max_vel=max_acc=2、max_tau=0.6、collision_step=0.05、search_budget=0.3s、input_timeout=0.5s。参数在启动时复制，修改须重启。
- Odin launch默认订阅/odin1/cloud_slam_down和/odin1/odometry，frame=odom、stamp_clock=receive、下界[-50,-50,-2]，上界来自YAML[50,50,10]；launch默认blind_radius=0覆盖YAML的0.3。勿混淆源码缺省bounds与实际启动覆盖。
- PlanMotion服务保持原Task38接口：service_only关闭独立goal/cloud规划及诊断定时器；PLAN按请求实际p/v直接init=false搜索。正式桥使用latest_observation完整快照替换；独立入口仍50帧双树累计。CHECK_LINE/VALIDATE_TRAJECTORY离散调用旧isSafe，不提供连续净空证明。
- 主工程接口3.4及执行层PD+DOB、水平1/垂直0.2m/s和加速度0.35/0.15m/s²保持；超包线完整曲线按统一时间缩放放慢，空间路径不变。本项目修复不修改主工程。

# 部署

- drone-new=.169，drone-refresh=.186；两机项目路径均/home/nvidia/dyn_small_obs_avoidance-ros2。两机已同步d47a718功能提交并完成Jazzy/ARM64 Release构建，无遗漏机待补同步。
- /home/nvidia/down_samp_exe.py为现场独立点云发布者，不由本项目/桥自动启动；此前169配置为0.1m降采样、0.4m球形剔除、auto球心。二次blind_radius=0不等于完整输入链无近身剔除。

# 未解决限制

- 已知核心Search.CollisionBetweenPrimitiveEndpoints不通过：离散采样可能漏掉端点间碰撞。此次不改核心或降低阈值，不能宣称连续净空或全量核心测试通过。
- 独立/kino_path是最后成功预览，不周期刷新；没有新PlanResult可能只是没有触发新规划。输入新鲜度和当前可执行性不能仅根据旧Path或最后一次成功判断。
- 双树按输入帧累计而非墙钟到期，移动障碍可留残影；上游球形剔除会删除真实近身障碍。正式桥拒绝unknown及有效Tag修正，稀疏观察可使航线不可执行。
- 独立搜索TIMEOUT会清空搜索进度，下一点云从头重试；单线程搜索同时延缓点云处理，使按帧淘汰的历史更久保留。核心密集原语采样仍与ROS1按位移稀疏检查不同，事件触发还原不等于内核完整还原。调大0.3s搜索预算时必须考虑wrapper的0.5s输入时限及搜索后检查，不能只改预算就保证输出。
- 两机处于同一ROS域且使用同名Odin话题时可能串流；调试必须先确认目标机数据源。隔离域回放与台架结果不能替代真实传感器长时负载或实飞验收。
