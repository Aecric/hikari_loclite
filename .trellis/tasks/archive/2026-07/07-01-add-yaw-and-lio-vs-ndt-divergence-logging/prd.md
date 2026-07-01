# add yaw and LIO-vs-NDT divergence logging

## Goal

当前 stdout 日志把所有位姿都只打了 translation、丢了 yaw，且 NDT 校正只在"被采纳"时打、还带 5s 节流。结果是 NDT 自信定错（走廊混叠 / 退化区）把系统"拉回 GOOD"到一个错误位姿时，日志里 conf/inlier/quality_good 全程满分、看不出任何异常——错误对排查完全不可见。

本任务只加**可观测性**：给位姿日志补 yaw，并新增 LIO 位姿 ↔ NDT 对齐位姿的**原始散度**日志（增益混合 / cap 之前），让下一次跑 bag 能直接在 log 里看出"落点偏没偏、偏在平移还是 yaw"。**不改任何控制逻辑 / 阈值 / 状态机行为。**

背景来自对 `hikari-indoor2-loc-20260701-160532-edd8.log` 的分析：3 次 `output jump rejected → tracking lost → ndt_local_recovery → GOOD`，恢复位姿实为错误（用户确认），但日志无法暴露。参见记忆 [[loclite-steady-log-no-yaw-confidently-wrong]]、[[ndt-correction-cap]]。

## Requirements

1. **位姿日志补 yaw**（yaw = `atan2(R(1,0), R(0,0)) * 180/π`，沿用 `reloc_manager.cpp:288` 约定，单位 deg）：
   - `FastLioFixedMap: reset to pose [...]`（fast_lio_fixed_map.cpp:232）
   - `FastLioFixedMap diag: ... pose=[...]`（fast_lio_fixed_map.cpp:432）
2. **NDT-Good 原始散度，每周期无条件打**（`MaybeNdtCorrectGood`, loclite_node.cpp:1119）：
   - NDT 收敛后（`res.valid`）立即打，**位于 degenerate / 低TP / inlier 门控之前**，去掉 5s 节流。
   - 内容：raw 散度 = `fast_pose.inverse() * res.pose` 的 `dtrans_m` / `dyaw_deg`，以及 LIO 绝对 yaw 与 NDT 绝对 yaw、`TP` / `inlier` / `degenerate` 标志。
   - `res` 不收敛时打一行 "ndt not converged" 便于对齐时间线。
   - 现有 `[NDT-Good] drift corrected` 采纳行补 yaw（保留其原节流）。
3. **冻结期 / 毛刺逐帧可见**：
   - GOOD 分支 `output_jump_rejected` 时（loclite_node.cpp:825），把已算好的 `smoother_delta`（loclite_node.cpp:789）的 `dtrans_m` / `dyaw_deg` 打出来（现在只当布尔、只有一句节流 warn），去掉/放宽节流做到逐帧，用于区分"LIO 真发散 vs 单帧毛刺"。
   - LOST 分支每帧（有 LIO 输出时）打 raw LIO lidar 位姿 vs `frozen_T_map_lidar_` 的 `dtrans_m` / `dyaw_deg` + 两者绝对 yaw。
4. **recovery 落点 yaw 可见**：
   - `[NDT-LocalRecovery] success`（loclite_node.cpp:888）补 frozen yaw、recovered yaw、dyaw。
   - `[LostRecover] ...`（loclite_node.cpp:1361/1370）补 recovered pose 的 yaw。

## Acceptance Criteria

- [ ] `colcon build --packages-select hikari_loclite --cmake-args -DCMAKE_BUILD_TYPE=Release` 通过（容器内 jazzy，见 [[docker-jazzy-build-procedure]]）。
- [ ] 重放同一段数据，`reset to pose` / `diag` 行含 yaw(deg)。
- [ ] 每个 NDT-Good 周期都有一行原始散度日志，包含被 degenerate/低TP 跳过的周期。
- [ ] 每次 `output jump rejected` 打出该帧 `smoother_delta` 的平移+yaw 幅度；LOST 期间逐帧有 raw-LIO vs frozen 的散度行。
- [ ] recovery / LostRecover 行能读出落点 yaw 与相对冻结位姿的 dyaw。
- [ ] 无控制逻辑 / 阈值 / 发布行为改动（纯日志），CPU 预算无实质影响（1–10Hz 文本行）。

## Definition of Done

- 纯日志改动，不动状态机 / 门控 / 发布路径。
- Release 构建 green。
- 日志字段命名与现有风格一致（英文标识、`dt=%.3f m` / `dr=%.2f deg` 口径）。

## Out of Scope

- 不改 smoother 冻结→recovery 的锚定逻辑、不改任何阈值（另开任务）。
- 不加 ROS 调试 topic / 不落盘 CSV（仅 stdout 日志）。
- 不动 Initializing 分支的 jump-rejected 文案（可选，非本任务重点）。
- 不引入新依赖 / 新配置项（yaw 计算用现有 Eigen/Sophus）。

## Technical Notes

- yaw 取法：`const Mat3d R = pose.so3().matrix(); double yaw = std::atan2(R(1,0), R(0,0)) * 180.0 / M_PI;`。仓库既有用法：reloc_manager.cpp:288/316/561。
- 散度：`SE3 div = fast_pose.inverse() * res.pose; dtrans = div.translation().norm(); dyaw = div.so3().log().z()`（或对 `div` 取 yaw；小角度下等价，统一用 `atan2` 取法更稳）。
- 关键代码位：
  - fast_lio_fixed_map.cpp:232（reset to pose）、:424-433（diag）
  - loclite_node.cpp:787-795（raw/smoothed/smoother_delta）、:823-828（NDT-Good 调用 + jump warn）、:881-896（LOST recovery）、:1119-1176（MaybeNdtCorrectGood）、:1361/1370（LostRecover）
- 日志用 glog `LOG(INFO)`（fast_lio 侧）与 `RCLCPP_INFO`（node 侧），与各自文件现状一致。
