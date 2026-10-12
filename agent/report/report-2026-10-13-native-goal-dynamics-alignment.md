<!-- Final native target lifecycle, directional limits and verification baseline. -->
# 原生目标生命周期与动力学最终基线

本报告描述2026-10-13最终实现。此前低加速度/1.2s原语为已撤销实验，保留记录于
report-2026-10-13-low-acc-native-primitive-fit.md；当前分模式参数和滚动结果见
report-2026-10-13-独立避障动力学与实时限额共享.md。

## 当前实现

正式桥与手动入口共用原生dual_tree地图、目标/时间/位移/旧路径受阻触发；保持
init=true、仅NO_PATH再init=false。service_only、latest_observation与正式节点专属profile已删除。
cloud.map_mode仅保留dual_tree可读兼容参数。桥配置只负责坐标模式、FCU实际里程计源及预览。

SetGoal请求header/goal/target_id/active，响应accepted/planner_session/goal_revision/detail。
同ID同坐标幂等；取消保留地图，停止目标搜索；非空取消ID必须匹配目标。
替换/取消清旧路径、执行索引与恢复标志；手动PoseStamped通过同一生命周期生成manual ID。
PlanMotion保留PLAN/CHECK_LINE/VALIDATE_TRAJECTORY，不替换自主目标。

所有PlanResult含session/target_id/sequence/goal_revision与四个float64实际限额：
max_velocity_xy/max_velocity_z/max_acceleration_xy/max_acceleration_z。
限额从core最终scalar与方向min生成；成功、失败、空ID身份心跳、PlanMotion响应均填写。
idle/有效近目标每秒发送空ID、status=9、planning_ms=0、无曲线的身份心跳。
输入故障撤销成功曲线后，恢复时允许一次actual-state规划；幂等SetGoal与心跳不触发搜索。

普通原语以端点速度范数和恒加速度验限。三次shot加速度端点范数覆盖全区间，
水平速度范数极值由解析三次方程求根；垂直及旧逐轴极值仍解析检查。
输出保持原生多项式及持续时间，无执行层retime。

## 当前参数

| 参数 | 最终值 | 语义 |
|---|---|---|
| search.max_vel / max_acc | 1.0 / 2.0 | SI逐轴scalar兼容上限 |
| search.max_vel_xy / max_vel_z | 1.0 / 0.2 | m/s；水平合范数/垂直绝对值 |
| search.max_acc_xy / max_acc_z | 2.0 / 2.0 | m/s²；水平合范数/垂直绝对值 |
| search.max_tau / init_max_tau | 0.6 / 0.8 | s；普通原语/首固定加速度最长时长 |
| resolution / voxel_size / safe_distance | 0.1 / 0.1 / 0.25 | m；空间格/体素/质心净空 |
| search_budget / tree_period | 0.3 / 20 | s/有效点云帧数 |
| replan_interval / replan_distance | 0.2 / 1.0 | s/m；实际时间或三维位移刷新 |

避障search.*是单源，执行消费result限额；直线由主工程control.yaml独立配置。
原算法加速度2为各轴上限，水平合可到2√2；当前水平合上限明确为2。
普通扩展仍5×5×5：只有当轴半幅加速度从静止末速度超过速度上限时，按当前速度和tau
可行区间取该轴负全/半/零/正半/全5值，默认只需Z适配。其他轴保持原候选与端点拒绝。
首init同cell临时节点修复保留，20时长仍0.04..0.8s；普通position-only hash不变。

## 验证与边界

最终Release核心22/22及输出契约/几何CTest2/2通过。三维/纯垂直目标与有限障碍、
125候选、首init20时长均验收；完整曲线独立检查p/v/a、连续性与原生时长。
DDS最终结果、滚动原曲线数值及瞬时测试问题见分模式补充报告。

只在开发机更新源码与安装；refresh/new均尚未同步本基线，需与主桥/消费者一起更新。
本次不连接硬件，不解锁/起飞，不改TODO/.gitignore，不commit/push。
普通原语净空证书保留；末端shot/几何服务仍离散净空，不能宣称整条连续证明。
体素质心0.25m不等于raw点/真实表面0.25m。合成与DDS验证不能替代现场负载或实飞验收。
