# 物理运动合理性 LOST 门控 + TF 冻结 + 重定位恢复

## Goal

给 hikari_loclite 增加一道"物理运动合理性"硬门控：当 LIO 估计出的本体速度 > 1 m/s 或加速度 > 0.8 m/s²（机器人本体物理上不可能这么快）时，立即判定为 LOST。进入 LOST 的那一刻冻结 TF（锁定在最后一次可信位姿）并保存该位姿，然后用 NDT / KISS-Matcher 尝试恢复。目的是在 LIO 跟踪发散（典型表现为速度/位姿瞬间飞出）时，避免把发散的位姿继续广播出去污染下游（Nav2/RViz），并尽快重定位回正。

## What I already know (from repo inspection)

* `NavState.vel_`（`common/nav_state.h:138`）是 ESKF 估计的世界系速度，`.norm()` 即 m/s 速率——速度门控数据现成，无需额外推导。
* 当前 LOST 只由 `LocLiteStateMachine::ObserveTrackingQuality` 的坏帧计数触发：`degraded_bad_frames=3`、`lost_bad_frames=10`（`loclite_state_machine.hpp:70-76`）。没有"物理不可能运动"的硬触发，也没有一帧直接 LOST 的路径。
* `ProcessFrame()`（`loclite_node.cpp:498-630`）：Good/Degraded 分支跑 smoother + `ObserveTrackingQuality` + `MaybeNdtCorrectGood`；LOST 分支当前仍 `PublishPose(state)` 把（可能发散的）LIO 位姿照发（`:616-618`），并 arm reloc、超 `lost_timeout_sec` 转 WAIT_FOR_INITIALPOSE。
* `PublishPose()`（`:875-926`）发布 odom(map→lidar) + TF(map→base_link) + path，均直接来自传入的 `state`。没有"冻结/锁定"概念。
* 进入 LOST 已经会 `reloc_->Arm("lost")`（KISS/SC，`:556-560`）；LOST 帧里 `TryScRelocalize` 跑 reloc 候选 → `NdtCorrector::Validate` 验证 → `lio_->ResetToMapPose` 恢复（`:611-613`, `1085+`）。
* smoother 已有输出跳变门限（`max_output_jump_trans_m=0.5` / `rot=15°`）但只在 Good/Degraded/Initializing 分支生效，且只是"拒绝本帧、回退到上一帧"，不改状态、不冻结 TF。
* `lio_->ResetToMapPose(pose)` 是恢复入口；NDT `Validate(prior_pose, scan)` 需要一个先验位姿做局部配准；KISS 是无先验全局配准。

## Assumptions (temporary, to validate)

* 速度阈值 1.0 m/s、加速度阈值 0.8 m/s² 均做成 yaml 可配（`reloc.*` 或新 `sanity_gate.*` 段），默认即用户给的值。
* 加速度用相邻帧 ESKF 速度差分 `|v_k - v_{k-1}| / dt` 得到（语义="估计运动的合理性"），而非 IMU 原始加速度。
* 速度/加速度用 3D 模长（含 z），以便也能抓到 LIO 垂直方向发散。
* 该门控只在 Good/Degraded（已有可信位姿、LIO 正常产出）时生效；Initializing/WAIT/Uninitialized 不参与。

## Decision (ADR-lite, 2026-06-17 user-confirmed)

**Context**: LIO 跟踪发散时速度/位姿瞬间飞出, 现有坏帧计数 LOST 太慢且 LOST 期间仍广播发散位姿污染下游。

**Decisions**:
1. **TF 冻结语义** = 持续重广播冻结位姿。进 LOST 保存最后一次可信 `T_map_base`, LOST(及后续 WAIT)期间每帧用新时间戳重发该固定位姿, TF 树保持新鲜, 机器人在图上"原地不动"。odom 同步冻结。
2. **加速度来源** = 差分 ESKF 速度 `a = |v_k - v_{k-1}| / dt`（速度门用 `vel_.norm()`）, 与速度门同源同尺度, 无需去重力/去偏置。
3. **恢复策略** = 先用保存的最后位姿作 NDT 局部恢复先验跑一次 `Validate`, 通过即 `ResetToMapPose`; 失败再退回现有 KISS/SC 全局重定位。
4. **触发** = 即时(1 帧超阈值即 SetLost), 绕过坏帧计数。

**Consequences**: 响应最快但单帧毛刺可能误触发(交由 NDT/KISS 恢复兜底, 可后续加去抖); 冻结+局部恢复让原地小发散能秒回正; 全局发散退 KISS 全局。

## Resolved (moved from Open Questions)

* 触发即时 / 加速度差分 / TF 重广播冻结 / NDT 局部优先 — 均已确认, 见上 ADR。

## Requirements (final)

* **物理合理性门控**（仅 Good/Degraded 帧, LIO 正常产出时）：`speed = state.vel_.norm()`；`accel = |state.vel_ - last_vel_| / (ts - last_vel_ts_)`（首帧或 dt≤0 跳过 accel）。任一超阈值 → 立即 `SetLost("sanity_speed"/"sanity_accel")`, 绕过坏帧计数, 日志带数值。
* **保存最后位姿**：Good/Degraded 每帧通过门控后, 持续记录 `last_good_T_map_base_` / `last_good_T_map_lidar_`（用于冻结与 NDT 先验）。门控在第 k 帧触发时, 冻结基准取第 k-1 帧的 last-good（当前帧已发散, 不能用）。
* **统一冻结**：任何路径进入 LOST（门控触发 / 坏帧计数 / 输入超时）都置 `pose_frozen_=true` 并锁定 `frozen_T_map_base_=last_good`。这同时修掉现有"LOST 仍 PublishPose 发散位姿"的问题。
* **冻结期发布**：LOST/WAIT 期间 `PublishPose` 改为发布冻结位姿(map→base_link TF + odom)且用当前帧时间戳重广播; path 不再追加。
* **恢复**：LOST 帧内先 `ndt_->Validate(frozen_T_map_lidar→imu prior, scan)` 局部恢复, 通过即 `ResetToMapPose` + 解冻; 失败退回现有 KISS/SC 全局重定位路径。恢复成功统一走"解冻 + smoother.Reset + ClearAccumulation + 进 Initializing/Good"。
* **配置**：新增 `sanity_gate.{enabled, max_speed_mps:1.0, max_accel_mps2:0.8}` 与 `reloc.ndt_local_recovery:true`（或并入现有段）, yaml 可配, 默认即用户给定值。

## Acceptance Criteria (evolving)

* [ ] 速度 >1 m/s 或加速度 >0.8 m/s² 时，状态机在该帧立即转 LOST（日志含触发原因与数值）。
* [ ] LOST 期间 map→base_link TF 锁定在进入 LOST 前最后一次可信位姿，不随发散 LIO 漂移。
* [ ] 保存的最后位姿被 LOST 恢复路径使用（NDT 先验或冻结基准）。
* [ ] 恢复成功后 TF 解冻并跟随新位姿；阈值可由 yaml 调整。
* [ ] Release 构建通过（容器内 jazzy，见 docker-jazzy-build-procedure 记忆）。

## Definition of Done

* 状态机/节点改动 + yaml 默认值 + 注释（中文注释，英文标识，符合本包约定）。
* 容器内 Release 构建 green。
* 设计文档/CLAUDE.md 状态机段落相应更新（若行为变化）。

## Out of Scope (explicit)

* 不引入持久后台 reloc 线程（维持现有 one-shot 有界查询边界）。
* 不改 KISS/SC 配准算法本身。
* 不引入 PGO / 动态图 / UI。

## Implementation Plan (small steps)

* **S1 — 门控 + 硬 LOST 入口**：`loclite_state_machine.hpp` 加 `SetLost` 已有, 复用; `ProcessFrame` Good/Degraded 分支 RunOnce 后、smoother 前算 speed/accel, 超阈值即 SetLost。新增成员 `last_vel_`/`last_vel_ts_`。读 `sanity_gate.*` yaml。
* **S2 — 保存 last-good + 冻结**：节点加 `pose_frozen_` / `frozen_T_map_base_` / `frozen_T_map_lidar_` / `last_good_T_map_*`; Good/Degraded 通过帧更新 last-good; 进 LOST 置冻结。`PublishPose` 在冻结态发布 frozen 位姿(新时间戳)。
* **S3 — NDT 局部恢复**：LOST 帧内, 在现有 `TryScRelocalize` 前先 `ndt_->Validate(frozen prior, scan)`; 成功 → ResetToMapPose + 解冻 + 状态恢复; 失败 → 走原 KISS/SC。抽公共"恢复成功"收尾。
* **S4 — 配置 + 文档 + 构建**：yaml 默认值 + 注释; 更新 CLAUDE.md / runtime-and-relocalization 状态机段(若行为变化); 容器内 jazzy Release 构建验证。

## Technical Notes

* 改动集中在：`loclite_state_machine.hpp`（硬 LOST 入口）、`loclite_node.cpp`（ProcessFrame 门控 + PublishPose 冻结）、新增/复用 yaml 段、可能 `lite_pose_smoother.hpp`（保存 last-good）。
* 状态机是 CLAUDE.md 标注的敏感区，改转换前需对齐设计文档语义。
* 相关记忆：[[ndt-correction-cap]]（NDT 校正硬封顶/走廊残差检测）、[[corridor-drift-frontend-architecture]]（前端发散根因）。
