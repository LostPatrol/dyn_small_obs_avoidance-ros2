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
| 搜索失败留下旧轨迹 | 每轮 reset；原子结果携带失败状态和空轨迹；Path 同步清空 |
| 无定义的 origin/map_size/time、timeToIndex 无返回 | 显式上下界、完整状态初始化、时间索引返回与正确动态 hash 索引 |
| priority_queue 内部节点原地改 f-score | 队列保存不可变 score 快照，降分时重新入堆，跳过旧条目 |
| shot 限幅被注释 | 解析检查三次曲线速度/加速度极值与位置边界；不通过就拒绝 |
| 无约束启发式 shot 时长可能太短 | 同一个三次边值解尝试 1、1.5、2、3、4 倍时长；受同一搜索预算控制 |
| near-end shot 失败就提前退出 | 继续搜索可绕行分支，不把局部进展当目标到达 |
| 原语 z 硬编码为 0.2～1.3，shot 另一套边界 | 统一配置上下界；原语转折点、shot 解析极值都检查 |
| 固定半径、体素与粗稀疏碰撞检查 | 参数化；按最大速度和时长限制采样间距；增加体素及样间位移保守余量 |
| cloud_all 无限累加，树/点数无容量 | 删除未消费的 cloud_all；两树轮换语义保留；超容量失效且清图 |
| 空 KD-Tree 视为 safe | 核心区分 NO_MAP；空/坏输入不允许规划 |
| 无时间预算、节点池访问边界隐患 | steady-clock 协作预算、分配前检查、TIMEOUT/NODE_LIMIT 明确输出 |
| 原 getKinoTraj 丢首尾、采样不均和 getSamples 空路径越界 | 从搜索节点/shot 导出原多项式；统一有界采样包含精确端点，移除无人使用的 getSamples |
| ros::Time 受校时影响、旧点云无限复用 | 计算预算与输入接收超时用 steady_clock；ROS 源时标另校验，可显式选择设备时钟模式 |

原算法的加权启发式、位置栅格状态合并、逐轴速度/加速度限制保持原有含义，
不额外声称全局最优或完备。`dynamic=true` 核心接口修复了时间索引，但空间碰撞查询没有运动预测；
ROS wrapper 使用 `dynamic=false`。

双树仍以输入帧计数轮换。清图、时间倒退与超时恢复会重新开始累计；不是按墙钟淘汰的概率地图。
同一体素多次质心滤波仍保持点在该体素包围盒内；查询膨胀包含 √3 倍体素边长以覆盖原始点。
样点之间到最近样点的路程上界为 `collision_step/2`，该项同样加入碰撞半径。

定时规划、源时标检查、原子状态和带时间轨迹是通用 ROS2 边界，不包含航点业务状态机。
无人机控制中的 HOLD/LAND、yaw、轨迹衔接、观察覆盖判据、真实机体包络及外参不属于本次移植。
