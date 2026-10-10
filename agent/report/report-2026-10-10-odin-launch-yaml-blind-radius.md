<!-- 仅记录用户明确授权的独立 Odin launch 参数优先级修复；未修改规划算法。 -->
# Odin launch 的 blind_radius 加载修复

日期：2026-10-10。主任务来自 `ros2-ardupilot-sitl-hardware/agent/task/new-1-path-delay.md`，用户明确允许仅修改本库 launch 文件。

`path_planning/launch/odin.launch.py` 原默认 `blind_radius=0.0` 覆盖 YAML。现在默认为空，不生成 radius override；使用 YAML 的参数，缺失时由节点默认 0.0 接管。用户显式数值（包括 0.0）仍覆盖 YAML。配置文件、算法源码、碰撞检查和搜索参数未改。

用户要求的 `gpt-6.1-sol / high` 子代理实施 launch 修复，无实机操作；主代理完成 refresh 选择性部署及原生构建，保留现场 YAML 的 0.8。new SSH 超时后用户指定改用 refresh，未改 new。

验证使用主工程项目 `.venv`、隔离 ROS domain 146 / LOCALHOST 实际启动规划器并读取参数服务，6 个场景全部通过：默认 YAML 0.0、指定 YAML 0.63、缺失 key 默认 0.0、显式 0.25 覆盖、显式 0.0 覆盖、嵌套 YAML key。该测试没有扫描输入，不能单独证明滤波或实际规划。

refresh 原生安装后源码与安装 launch SHA256 一致，实际参数为现场 YAML 的 0.8（此前运行值为 0.35）。真实 Odin/FCU 的短时采样中出现成功规划，也出现输入配对超时；正式主工程地图桥查询仍 NO_PATH。未解锁或起飞实机，未改变现场 YAML、飞控参数或标定。盲区滤波会删除半径内真实障碍，不等于 Odin 内部 SLAM 输入过滤，不构成实飞许可。

独立 Odin 默认非 service-only，仍按定时器对最近 goal 持续规划；“最后业务航点完成后停止请求、悬停”由主工程机载控制层处理，本库未改任务生命周期。

详细控制层实现、SITL、失败记录和台架边界见主工程 `agent/report/report-2026-10-10-new-1-path-delay.md`。现场归档/日志位于 refresh `/home/nvidia/ros2-ardupilot-maintenance/path-delay-20261010/`。本库仅 launch 和本报告提交 main；既有 `.gitignore`、旧报告及其他过程文件不纳入本次提交。
