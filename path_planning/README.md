<!-- Task38 service adapter contract; deployed search/core and field configuration stay authoritative. -->
# 规划服务适配

`path_planning_node` 现在额外提供相对命名服务 `plan_motion`（`path_planning/srv/PlanMotion`）。
`PLAN` 使用请求中的实际世界坐标位置、速度和当前目标，调用原有搜索；输出原始三次段和采样
p/v/a，不修改搜索、碰撞检查或动力学参数。请求由控制权威提供，不需要另外发布目标或里程计。
服务请求时间戳使用 ROS 时间；点云仍按现场 `stamp_clock` 参数校验。

默认 `service_only=false`、`cloud.map_mode=dual_tree`，独立 Odin 入口维持既有目标定时搜索、
双树累积行为及现场参数。正式集成设置 `service_only=true`，关闭定时目标搜索；
可同时设置 `cloud.map_mode=latest_observation`，由桥提供完整占用快照，每次在 wrapper 中
`clearMap()` 后 `setKdtree()`，防止已由桥退役的障碍继续残留。此模式要求 service-only，
只改变桥快照适配；核心文件、独立算法入口和 YAML 无额外算法替换。

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
