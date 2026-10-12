<!-- ROS1 standalone replanning and Task38 service adapter; core/field configuration stay authoritative. -->
# 规划服务适配

`path_planning_node` 现在额外提供相对命名服务 `plan_motion`（`path_planning/srv/PlanMotion`）。
`PLAN` 使用请求中的实际世界坐标位置、速度和当前目标，调用原有搜索；输出原始三次段和采样
p/v/a，不修改搜索、碰撞检查或动力学参数。请求由控制权威提供，不需要另外发布目标或里程计。
服务请求时间戳使用 ROS 时间；点云仍按现场 `stamp_clock` 参数校验。

默认 `service_only=false`、`cloud.map_mode=dual_tree`。独立入口恢复 ROS1 事件重规划：
目标到达且起点已就绪时规划；点云更新后按旧路径每40点检查，不安全才重规划。
实际起点距目标小于0.3m时跳过点云重规划；失败保留重规划标记和旧路径，成功才清标记。
初次失败且没有旧路径时不自动重试；目标先于里程计时不由定时器补发规划。
`planning_rate` 只控制输入失效诊断，不再控制搜索频率。双树累积和现场参数不变。

核心普通运动原语保留恒加速度模型，碰撞检查用端点净空余量覆盖弧长上界：
`Da + Db >= 2 * safe_distance + h * (|va| + |vb|) / 2` 时接受整段，否则查询中点并细分。
起点距离在本次节点展开的候选间复用；未能确认的区间受现有搜索期限及14层细分限制。
此检查只作用于普通原语。`collision_step` 仍用于末端shot和几何服务的离散检查，
整条含shot轨迹仍不提供连续净空证明。修改核心检查应从
`path_searching/src/kinodynamic_astar.cpp` 的 `isPrimitiveSafe()` 入手。

正式集成设置 `service_only=true`，仅响应显式服务请求；
可同时设置 `cloud.map_mode=latest_observation`，由桥提供完整占用快照，每次在 wrapper 中
`clearMap()` 后 `setKdtree()`，防止已由桥退役的障碍继续残留。此模式要求 service-only，
只改变桥快照适配；核心文件、独立算法入口和 YAML 无额外算法替换。

可选 `time_index` 订阅由launch默认映射到 `/demo_node/trajectory_time_index`，沿用原作者
绝对索引差小于10的节流及旧路径index+3位置/差分速度；内部检查与索引缓存按0.01s采样。
不发布索引时从实际里程计状态规划。先固定零初始加速度搜索，仅NO_PATH时再自由加速度搜索。
保留整个旧路径检查及稀疏步长；只对原版未初始化读取和确定越界保留必要保护。
`/kino_path` 不周期刷新；新规划或输入失效时才有对应结果，不用“没有新结果”推断规划失败。

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
