<!-- Shared native primitive-duration adaptation, validation and remaining limits. -->
# 低加速度与原生原语时长对齐

> 历史实验：本报告0.35/0.15m/s²与max_tau=1.2s已被分模式最终配置取代。当前避障为1/0.2m/s、2/2m/s²、max_tau=0.6s；见report-2026-10-13-独立避障动力学与实时限额共享.md。

2026-10-13；开发机离线与隔离DDS验证，无硬件连接、解锁或起飞。依据当前授权，
只修改公共core/YAML的普通原语时长，并维护默认参数文档；没有桥专用override。

| 参数 | 修改前 | 修改后 | 理由 |
|---|---|---|---|
| search.max_tau（YAML/core） | 0.6s | 1.2s | 默认Z加速度0.15m/s²从静止位移0.108m，超过0.1m位置格，末速度0.18m/s仍低于0.2限额 |

本次其余参数均未改：init_max_tau=0.8s、max_vel=1m/s、max_acc=0.35m/s²，
方向限额vxy/vz=1/0.2m/s、axy/az=0.35/0.15m/s²；w_time=10、horizon=100m、lambda_heu=5，
resolution=0.1m、time_resolution=0.8s、safe_distance=0.25m、voxel_size=0.1m、collision_step=0.05m，
search_budget=0.3s、allocate_num=100000、tree_period=20、max_cloud_points=500000，
lower=[-50,-50,-2]m、upper=[50,50,10]m；planning_rate=10Hz、replan_interval=0.2s、
replan_distance=1m、input_timeout=0.5s与输出预算不变。

普通扩展仍只用一个max_tau及原5×5×5加速度候选，position-only hash与代价/启发式不变。
0.6s时静止Z位移只有0.027m，水平单轴只有0.063m；普通同cell异速候选会被旧规则裁剪，
是否跨格取决于起点相位。1.2s保证静止Z最大原语能跨0.1m格，未另建速度hash或搜索方法。
已完成的首init同cell临时节点修复保持；20个初始时长仍为0.04..0.8s。

离线探针仅链接原生库，使用init=true、NO_PATH才false、dynamic=false，精确执行新曲线前0.1/0.2s
后用返回p/v重规划，无retime/平滑/跟踪器。地图固定，因而此处“滚动”指运动状态滚动，
不预测移动障碍，不重现实时点云双树更新时序。每条完整输出曲线按5ms独立检查导数和raw/质心距离。

| 有限圆柱（半径0.15m，高0..1.8m，中心1.5/0）前缀 | max_tau=0.6s | max_tau=1.2s |
|---|---|---|
| 0.1s | 100s仍距目标1.97045m；1000次REACH_END | 36.8s到目标0.0234m内；369次REACH_END |
| 0.2s | 22.2s NO_PATH，距目标1.92713m | 35.6s到目标0.0256m内；179次REACH_END |

新时长圆柱两场完整输出曲线最大vxy/vz/axy/az为0.995896/0.18/0.35/0.15，
最小体素质心距离0.250019m、raw点距离0.234521m。raw距离小于0.25m是既有体素质心规则边界，
不能把该结果宣称为raw表面0.25m或旧严格SITL脚本0.35m通过。

Release安装成功，core22/22、DDS重规划/服务/ROS输入输出/统计18/18、输出契约/几何CTest2/2、
git diff --check通过。旧2m/s与2m/s²包线的125候选回归显式保留其原0.6s时长：
该旧包线用1.2s时最大输入端速度2.4m/s超旧2m/s限额，只剩27候选，原断言并未降低；
默认包线测试直接使用新的1.2s SearchConfig。未新增冗余用例。

安装头文件/YAML均1.2，node已重编。core动态库SHA256仍为
6b9035ac0d900b840c95f84965e50fa2ba3eba7bdad2f64a0443b14207f36a40：
SearchConfig默认实参在调用端生成；参数化库算法没有因仅改默认值而变化，这是预期行为。

额外只读复盘resumed.samples.json的9个WAYPOINT状态转换，用固定trace-map5地图搜索，
使用诊断目标(3,0,1.5)，0.6/1.2结果相同：首次vz=0.2008716m/s为INVALID_INPUT；后续逼近障碍且有速度的起点
虽safe却NO_PATH，随后三个起点已unsafe。此固定图并不证明当时实时地图相同，
但不能声称加长时长可恢复已不可制动/起点unsafe的状态。

所有过程探针与CSV位于agent/codex/low-acc-primitive-fit；首轮误用近似goal=(3,0,1.5)的
trace文件单独标记approx-goal，不用于精确trace对比。精确trace目标为
(2.9850890068,0.0100364666,1.5)，使用原trace首p/v及315240点固定图。四场精确复验已完成：

| 固定trace图前缀周期 / max_tau | 结果 | 完整曲线最小质心净空 | 完整曲线最小raw净空 | 最大vxy / vz / axy / az |
|---|---|---|---|---|
| 0.1s / 0.6s | 325次REACH_END，32.4s到目标附近 | 0.25002375m | 0.235505m | 0.930911 / 0.163730 / 0.35 / 0.15 |
| 0.1s / 1.2s | 288次REACH_END，28.7s到目标附近 | 0.25004445m | 0.240200m | 0.989117 / 0.192230 / 0.35 / 0.15 |
| 0.2s / 0.6s | 136次REACH_END，27.0s到目标附近 | 0.25001080m | 0.240183m | 0.944498 / 0.198230 / 0.35 / 0.15 |
| 0.2s / 1.2s | 139次REACH_END，27.6s到目标附近 | 0.25006918m | 0.241763m | 0.814124 / 0.195230 / 0.35 / 0.15 |

0.6s结果复现既有首init修后结果；1.2s在0.1s前缀较快，在0.2s前缀略慢，
不能表述为普遍提升速度。四场均无NO_PATH/TIMEOUT且遵守方向动力学限额。
并行离线运行的最大单次搜索时间0.6s两场为91.4/36.7ms，1.2s为14.6/8.5ms；
这不代表实时ROS节点性能基准。完整CSV及summary.json保存在上述过程目录。

两机均尚未同步本次基线；真实SITL由主任务继续验收，不能以本报告替代。
未修改TODO/.gitignore，未commit/push。
