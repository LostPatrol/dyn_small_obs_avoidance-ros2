<!-- Shared native autonomous planning, target lifecycle and geometric query services. -->
# 规划服务适配

`path_planning_node` 现在额外提供相对命名服务 `plan_motion`（`path_planning/srv/PlanMotion`）。
`PLAN` 使用请求中的实际世界坐标位置、速度和当前目标，调用原有搜索；输出原始三次段和采样
p/v/a，不修改搜索、碰撞检查或动力学参数。请求由控制权威提供，不需要另外发布目标或里程计。
服务请求时间戳使用 ROS 时间；点云仍按现场 `stamp_clock` 参数校验。

所有入口共用原生双树地图和 ROS1 事件重规划：
目标到达且起点已就绪时规划；点云更新后按旧路径每40点检查，不安全才重规划。
实际起点距目标小于0.3m且成功曲线仍有效时跳过点云重规划；失败保留重规划标记和旧路径，成功才清标记。
此外，距上次规划尝试经过 `replan_interval` 秒（默认0.2秒，名义5Hz），或实际三维位置变化达到
`replan_distance` 米（默认1米），也会重规划。参数位于 `config/planner.yaml`，修改后重启。
这两种触发从实际里程计位置/速度规划，不受执行索引节流限制；每次实际尝试（包括失败）
更新计时和位移基准。输入就绪后也可补规划或重试首次失败。
`planning_rate` 控制定时检查和输入失效诊断频率（默认10Hz），时间触发有最多一个检查周期的延迟，
搜索耗时也可能延后触发。双树累积仍保留历史障碍，消失障碍须从地图淘汰后才能缩短路径。
两项阈值均接受正无穷 `.inf`，分别关闭对应的强制刷新；目标和旧路径受阻事件仍有效。
每秒身份心跳独立于重规划触发，活动远目标等待刷新时同样发送，不搜索、不替换曲线。

核心普通运动原语保留恒加速度模型，碰撞检查用端点净空余量覆盖弧长上界：
`Da + Db >= 2 * safe_distance + h * (|va| + |vb|) / 2` 时接受整段，否则查询中点并细分。
起点距离在本次节点展开的候选间复用；未能确认的区间受现有搜索期限及14层细分限制。
此检查只作用于普通原语。`collision_step` 仍用于末端shot和几何服务的离散检查，
整条含shot轨迹仍不提供连续净空证明。修改核心检查应从
`path_searching/src/kinodynamic_astar.cpp` 的 `isPrimitiveSafe()` 入手。

正式桥通过 `set_goal`（`SetGoal`）设置或取消持久目标；无需循环发送 PLAN。
请求字段为 `header/goal/target_id/active`；响应字段为
`accepted/planner_session/goal_revision/detail`。设置时目标ID非空且frame/位置合法；
同ID同坐标幂等。取消时非空ID必须匹配活跃目标；保留地图，停止目标搜索；现有timer每秒发一次空target_id身份心跳(status=9)，不搜索。
手动 `goal` 的 PoseStamped 走同一逻辑，由节点为每次命令分配自己的ID。
换目标清空旧路径和执行索引；每条原生PlanResult携带进程session与目标ID。
成功曲线被输入故障/失败撤销后，输入恢复触发一次实际状态规划，不受近目标早退或旧索引限制；
幂等SetGoal不触发搜索。健康输入下等待重规划时可发空ID身份心跳，不作为新目标/执行曲线。

公共YAML的选定对照配置：水平速度/加速度范数为2.0 m/s、2.0 m/s²，垂直绝对限额同为2.0。
裸C++/节点默认保持原值；完整参数作用见[参数说明](../docs/planner_parameters.md)。
恒加速度原语在端点检查速度范数，末端shot解析检查水平速度范数的极值，
两者均检查加速度范数/垂直限额，保持原持续时间，不依赖执行层retime。
实际起点超速时保留真实初速度，先用满足加速度、净空及位置边界的首段减速回限额内；
该单调减速首段可暂时超过名义速度；不能完成制动时返回失败，不截断或伪造起始速度。
后续段仍严格遵守名义限额。
普通原语 `search.max_tau` 保持原生0.6s；若某轴半幅加速度从静止的末速度已经超过该轴速度上限，
仅该轴按当前速度与时长推导可行正负加速度幅值，再保留负全幅/半幅/零/正半幅/全幅5值。
默认Z另外补入未重复且在限额内的`-vz/max_tau`精确停速候选，避免细小竖直速度持续积累；
原125候选保留，最多150候选。其他轴保持旧候选；显式init固定加速度20个时长仍为0.04..0.8s。
配置用 `search.max_vel_xy/max_vel_z/max_acc_xy/max_acc_z`；旧scalar CLI填充未显式指定
的方向CLI限额，显式方向CLI优先。直接C++配置的scalar仍是额外逐轴上限。
`search.*` 是避障动力学唯一配置源；每条PlanResult（含身份心跳、失败与PlanMotion响应）
从最终core配置填写 `max_velocity_xy/max_velocity_z/max_acceleration_xy/max_acceleration_z`，
执行端直接使用这些实际限额。直线模式由主工程 `control.yaml` 独立配置。
原算法2.0m/s²是各轴绝对上限，水平合加速度可到2√2；当前水平合范数上限明确为2.0m/s²。

可选 `time_index` 订阅由launch默认映射到 `/demo_node/trajectory_time_index`，沿用原作者
绝对索引差小于10的节流及旧路径index+3位置/差分速度；内部检查与索引缓存按0.01s采样。
不发布索引时从实际里程计状态直接选择加速度，时间/位移刷新同样不强制零加速度coast，
避免每次重规划把真正转向推迟到下一次刷新之后。仅旧路径索引模式保留固定零加速度首段及NO_PATH重试。
初始短coast和超速恢复即使未离开起点cell也保留为临时节点；普通原语仍按原空间hash剪枝。
`odin.launch.py`默认按SDK世界系速度读取Odin；通用/MAVROS入口仍按body twist处理。
Odin入口默认可由YAML的`odin_launch.ros__parameters.odometry_twist_in_body`覆盖，显式CLI最终优先。
`/kino_path` 仅在规划成功后更新；有活跃目标且输入失效时仍会发布结果，不用“没有新结果”推断规划失败。

`CHECK_LINE` 校验完整闭线段，`VALIDATE_TRAJECTORY` 校验尚未执行的曲线部分，两者不运行 A*。
适配器调用旧核心 `isSafe()`，沿空间按 `search.collision_step` 离散采样，保留严格小于
`search.safe_distance` 的体素质心距离规则。曲线采样用速度 Bezier 控制凸包约束相邻采样间的
最大行程，包含剩余部分首尾；**仍不提供连续净空证明**，不引入 Task37 的连续距离求根。
输入要求有限系数、正持续时间、C1连续、末点等于请求目标；最多256段、100000检查，
总时长受 `output.max_duration` 限制，校验期限沿用 `search.search_budget`。
失败不附带轨迹，`REACH_HORIZON` 仍只表示局部路径。每次响应附带进程唯一 `planner_session`
及实际地图 `cloud_stamp`，消费端必须自行关联请求与地图版本。

构建无需新增依赖，但接口消费者必须随服务生成后重建：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to path_planning --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ctest --test-dir build/path_planning -R test_motion_adapter --output-on-failure
.venv/bin/python -m pytest path_planning/test/test_motion_service.py -q
```

上述测试仅使用隔离 DDS 域和合成几何，无飞控命令或实机解锁/起飞。

参数来源按显式launch参数 → `planner.yaml`节点段 → 默认值；公共段为`path_planning`，
正式桥也使用同一公共段。文件不可读/语法错误打印警告并回退；
参数值非法仍由节点校验。可直接传`search.max_vel:=1.5`、`blind_radius:=0.0`等，
修改源码YAML须更新安装配置并重启。共享配置见[使用说明](../docs/USAGE.md)。
