<!-- Source-preserving migration decisions and fixes relative to the pinned ROS1 revision. -->
# 移植与缺陷修复说明

基准为上游 `9d25dd6`，不是将另一套 A* 或样条规划器换名交付。
核心 `search/stateTransit/estimateHeuristic/cubic/quartic/computeShotTraj` 由原源码迁移。
ROS API 只留在 `path_planning`；`path_searching` 使用普通 C++17/Eigen/PCL。

| 上游问题或耦合 | 本仓库处理 |
| --- | --- |
| catkin/roscpp/ROS1 launch | ament_cmake/rclcpp/rosidl/Python launch，Jazzy 原生构建 |
| ROS 参数、时钟、点云调试发布耦合进核心 | `SearchConfig` 配置；steady_clock 搜索预算；核心不链接 rclcpp |
| 起点 z 速度写入 y、未初始化目标/起点 | wrapper 分别保存 p/v、输入就绪状态；默认按 Odometry 语义将 child-frame twist 旋转到 world |
| 路径点数加一、外部 time_index 越界 | 不保留 demo time-index 执行器协议；从实际状态重规划；标准容器长度与精确采样 |
| 搜索失败留下旧轨迹 | 每轮 reset；PlanResult 携带失败状态和空轨迹；kino_path 保留最后成功预览，不刷新旧时间戳 |
| 无定义的 origin/map_size/time、timeToIndex 无返回 | 显式上下界、完整状态初始化、时间索引返回与正确动态 hash 索引 |
| priority_queue 内部节点原地改 f-score | 队列保存不可变 score 快照，降分时重新入堆，跳过旧条目 |
| shot 限幅被注释 | 解析检查三次曲线速度/加速度极值与位置边界；不通过就拒绝 |
| 无约束启发式 shot 时长可能太短 | 同一个三次边值解尝试 1、1.5、2、3、4 倍时长；受同一搜索预算控制 |
| near-end shot 失败就提前退出 | 继续搜索可绕行分支，不把局部进展当目标到达 |
| 原语 z 硬编码为 0.2～1.3，shot 另一套边界 | 统一配置上下界；原语转折点、shot 解析极值都检查 |
| 固定半径、体素与粗稀疏碰撞检查 | 默认0.45 m半径、0.1 m体素与ROS1相同；距离严格小于半径才拒绝，不另加补偿；连续曲线距离驻点检查 |
| cloud_all 无限累加，树/点数无容量 | 删除未消费的 cloud_all；两树轮换语义保留；超容量失效且清图 |
| 空 KD-Tree 视为 safe | 核心区分 NO_MAP；空/坏输入不允许规划 |
| 无时间预算、节点池访问边界隐患 | steady-clock 协作预算、分配前检查、TIMEOUT/NODE_LIMIT 明确输出 |
| 原 getKinoTraj 丢首尾、采样不均和 getSamples 空路径越界 | 从搜索节点/shot 导出原多项式；统一有界采样包含精确端点，移除无人使用的 getSamples |
| ros::Time 受校时影响、旧点云无限复用 | 计算预算与输入接收超时用 steady_clock；ROS 源时标另校验，可显式选择设备时钟模式 |

原算法的加权启发式、位置栅格状态合并、逐轴速度/加速度限制保持原有含义，
不额外声称全局最优或完备。`dynamic=true` 核心接口修复了时间索引，但空间碰撞查询没有运动预测；
ROS wrapper 使用 `dynamic=false`。

双树仍以输入帧计数轮换。清图、时间倒退与超时恢复会重新开始累计；不是按墙钟淘汰的概率地图。
同一体素多次质心滤波仍保持点在该体素包围盒内。2026-10-08按用户要求对齐ROS1：
最近存储点距离严格小于 `safe_distance` 才拒绝，不再增加体素位移或采样间隙补偿。
因此0.45 m约束针对KD-Tree中的体素质心；当前连续曲线检查采用距离平方驻点，
仍不能保证对每个原始点同样净空，也不解释未知空间或跟踪误差。

定时规划、源时标检查、原子状态和带时间轨迹是通用 ROS2 边界，不包含航点业务状态机。
无人机控制中的 HOLD/LAND、yaw、轨迹衔接、观察覆盖判据、真实机体包络及外参不属于本次移植。

## 差异与实现范围

`PlanResult` 是当前规划有效性的权威状态，`/kino_path` 仅用于预览最后成功曲线。
只消费 Path 的程序须增加状态协议或改用 PlanResult；RViz 仍显示线条不能证明当前可执行。

task37 审查的 A01–A28 是差异核对表，包含有意的 ROS2 行为选择、已保留的算法近似及历史缺陷，
不能把整张表当作必须恢复旧行为的 bug 清单。当前差异包括：

- A01–A07：输入接受/预处理、body twist 旋转、定时触发、实际状态重规划、自由初始加速度、
  时标/新鲜度检查及可选盲区。没有实现 ROS1 原始点序重放、执行器索引或两阶段 init 重试。
- A08–A14：明确统一边界、碰撞距离与ROS1对齐并改为连续曲线驻点检查、shot 时长候选和动力学校验、终止策略及前置拒绝。
  没有恢复未初始化边界、粗碰撞检查或不限导数的历史 shot。
- A15–A22：主要运动学/代价和位置格近似保留；加权启发式未调成另一算法。
  堆修复、数值保护、墙钟预算和容量失败语义属于明确的实现差别；没有共同确定性工作量
  限制的 ROS1 修复参考 harness、稳定等分队列协议或六维状态格。
- A23–A28：双树逐帧质心累积保留，但接受/清图事件和精确端点采样不同；输出改为原子结果和
  世界系轨迹，没有恢复旧长度 bug、旧分散消息或未实际消费的字段。

审查中的 ROS1 修复参考版、执行器、控制租约、飞行闭环和端到端验收是设计建议，
不是本仓库已实现的功能。task37 修复独立核心和 ROS2 边界，不引入飞控耦合，也不声称轨迹逐点等价。
