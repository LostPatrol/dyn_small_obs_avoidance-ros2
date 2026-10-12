<!-- 当前真实算法、服务适配和逐机部署；验证边界与关键历史。 -->
# 当前基线与部署状态

- 2026-10-12按用户最新要求，在169当前旧算法上恢复Task38接口。`path_searching`整个目录与任务开始时169快照完全一致，没有恢复连续距离算法、轴向/合量搜索限值改写或新版搜索逻辑。当前原语最多42次采样（ceil(0.6×sqrt(3)×2/0.05)）。
- 169当前源YAML：horizon=100、resolution=0.1、voxel_size=0.1、safe_distance=0.25、max_vel/max_acc=2、max_tau=0.6、collision_step=0.05、预算0.3s、input_timeout=0.5s。以本次实时源码及ROS读参为准，旧记忆中的20/0.2配置已失效。
- 独立Odin入口仍默认订阅 `/odin1/cloud_slam_down`，receive时钟、下界[-50,-50,-2]、上界由YAML指定[50,50,10]；launch默认blind_radius=0覆盖YAML的0.3。源码缺省bounds为[-50,-50,0.2]/[50,50,5]，沿用169现场源码，不与实际启动参数混淆。
- 新增 `PlanMotion` 兼容服务、service_only及完整桥快照替换，仅属于wrapper/独立MotionAdapter。PLAN原样调用旧search/getSegments；CHECK_LINE及剩余曲线检查离散调用旧isSafe，不提供连续净空证明。独立入口的50帧双树累计行为保持。
- 主工程执行层保留PD+DOB、水平1/垂直0.2m/s与加速度0.35/0.15m/s²；超包线完整曲线按统一时间缩放放慢，空间路径不变，实际p/v接入及60s时长门槛不放宽。
- drone-new=.169、drone-refresh=.186，两机路径均 `/home/nvidia/dyn_small_obs_avoidance-ros2`。两机已同步源码并原生构建；主工程共享接口恢复3.4，隔离ARM64控制/桥/服务适配测试及无FCU smoke通过。169原Odin、降采样、独立目标规划进程未重启。
- 两机现有 `/home/nvidia/down_samp_exe.py` 为独立点云发布者，不由桥自动启动；169实测0.1m降采样、0.4m球形剔除、auto球心。可能删除真实近身障碍，保持现场配置，不把0m二次盲区当作完整链路无剔除。
- 同步前逐机备份 `/home/nvidia/ros2-ardupilot-maintenance/task38-restore-20261012/`；开发机169源快照和运行参数位于主工程 `agent/codex/task38-restore-20261012/169/`。逐机相机/Tag及/etc配置保留，没有实机解锁或起飞。

# 未解决风险与验证边界

- 核心19例18通过，旧 `Search.CollisionBetweenPrimitiveEndpoints` 仍失败：0.099990361693828175m < 0.1m。保留旧离散碰撞采样，不修算法、不降低测试阈值，不能宣称全量通过。
- 新MotionAdapter四例、输出契约六例、服务真实DDS一例、原Python回归八例通过；两机ARM64几何/输出契约验证通过。主工程真实SITL六场景通过，不替代真实传感器、安装外参、长时热负载或实飞验收。
- `/plan_result` 是独立入口每轮有效性依据；`/kino_path` 保留最后成功预览，不能直接执行旧预览。参数只在启动时复制，修改须重启。
- 正式桥仍拒绝unknown与有效Tag修正，实际稀疏/近身覆盖可能使真实航线无法执行。上游球形剔除和旧碰撞采样均有边界，软件测试不能证明真实飞行净空。

# 关键历史

- 2026-10-12早先回退到169的4119937加现场修改：开发机/186的备份在 `agent/codex/rollback-169-20261012-153708/`，186更早完整备份在 `/home/nvidia/ros2-ardupilot-maintenance/sync-from-169-20261012-1536/`。本次接口恢复取代“主工程3.3且无PlanMotion”的部署结论，旧核心选用仍有效。
- 2026-10-10 Odin YAML优先修复已回退，历史报告不是当前入口行为。旧0.35m净空下起点近点诊断不能当作本次0.25m配置实飞验收。
