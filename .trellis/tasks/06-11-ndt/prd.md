# NDT 本体与稳定门控

## Goal

让 hikari_loclite 的 NDT 链路真正生效：当前 `NdtCorrector` 是 stub（`Validate` 恒通过 → 任何 /initialpose 都被接受；`Align` 恒无效 → GOOD 态 1Hz 漂移校正不工作）。本任务实现 NDT 真实配准 + 置信度/delta 门限、移植轻量稳定门控（Initializing→Good 放行），并修复 fixed-map 不变量问题。

## Requirements

1. **NdtCorrector 本体**（`src/ndt/ndt_corrector.cpp` + 头文件按需扩展）
   - `SetMap()` 真实保存 target 点云并一次性 `setInputTarget`（复用同一 pclomp NDT 实例，避免每次重建体素协方差；fixed_map 已按 voxel_leaf 降采样）。
   - `Align(scan, guess)`：scan（lidar 系去畸变点云）先降采样（新增 `ndt.source_leaf`，默认 0.5m），以 guess 为初值跑 pclomp NDT OMP；输出收敛位姿、TP（`getTransformationProbability()`）、delta_trans/delta_rot（相对 guess）。
   - `Validate(candidate, scan)`：基于 Align，valid 判定 = 收敛 + TP ≥ `ndt.min_confidence` + delta ≤ `ndt.max_delta_trans_m`/`max_delta_rot_deg`。
   - 置信度口径：**仅 TP + delta 门限**（用户拍板）；覆盖率指标留作后续扩展（Out of Scope）。
   - `ndt.min_confidence` 默认值按 TP 量纲重新标定（lightning 经验值参考其 yaml/代码，stub 时代的 1.0 可能不适配）。
2. **稳定门控**（轻量版 lightning StabilityGate，建议放 `system/` 下独立小类或并入 loclite_node）
   - /initialpose 验证通过后状态置 Initializing（不直接 Good）；滑窗 `deque<(t, SE3)>` 内位姿抖动 < 阈值持续 `window_sec` 才放行 Good；NDT 置信度 ≥ `conf_upper_thres` 可提前放行。
   - yaml knobs（`system.stability_gate_*`）：enabled=true、trans_thres=0.1、rot_thres_deg=4.0、window_sec=3.0、conf_upper_thres 按 TP 量纲定。
   - 参照实现：lightning localization.cpp:1543 `ApplyStabilityGate`、localization.h:373-379。
3. **fixed-map 不变量修复**（`src/lio/fast_lio_fixed_map.cpp` RunOnce 首帧分支）
   - 首帧/reset 后不再向 `fixed_ivox_` `AddPoints`——固定地图永不被当前 scan 污染。确认 ResetToMapPose 路径同样不触发插入。

## Acceptance Criteria

- [ ] Release 构建通过（lightning-jazzy:dev 容器，`colcon build --base-paths src/hikari_loclite --packages-select hikari_loclite --cmake-args -DCMAKE_BUILD_TYPE=Release`）
- [ ] 非 Release（RelWithDebInfo，独立 build base，验后删除）构建通过
- [ ] 编译产物校验：Release 产物仍无 run_loclite_offline / Pangolin

> 行为验收（bag 回放、/initialpose 拒绝/通过、漂移校正效果）由用户手动执行，不在本任务 AI 验收范围内（用户 2026-06-11 拍板："验收我来做 你只管编译是否通过"）。

## Definition of Done

- 双构建绿；代码内中文注释说明 TP 量纲与门限标定依据
- 新 yaml 键写入 config/loclite_livox.yaml（带中文注释）
- 若发现新契约/陷阱，更新 .trellis/spec

## Decision (ADR-lite)

**Context**: NDT 置信度口径、UI 归属、验收方式需用户拍板。
**Decision**（用户 2026-06-11）：
1. 置信度仅 TP + delta 门限；覆盖率指标后续扩展。
2. Pangolin UI 单独开任务（仅 Debug/offline 生效，Pangolin 拷入 thirdparty/）。
3. 验收不绑定数据：AI 负责编译通过，行为验收用户手动做。
4. fixed-map 首帧污染问题随本任务修。
**Consequences**: 任务保持算法聚焦；TP 单指标对错误初值的拒绝能力弱于双指标，依赖 delta 门限兜底，实测若误收率高再加覆盖率。

## Out of Scope

- Pangolin UI（单独任务）
- SC 重定位管线（P5）
- NDT 级联（ndt_status=1）
- 覆盖率/inlier 置信度指标
- bag 行为验收（用户手动）

## Technical Notes

- 包内已有 pclomp：`include/hikari_loclite/ndt/ndt_omp.h/_impl.hpp`、`voxel_grid_covariance_omp*`、`src/ndt/ndt_omp.cpp`（已编译通过）。
- 调用面已就绪（上一任务）：`LocLiteNode::HandlePendingInitPose()` 调 `Validate(pending, 去畸变scan[lidar系])`；`MaybeNdtCorrectGood()` 按 `ndt.good_rate_hz` 调 `Align` → `smoother_.ApplyNdtCorrection(gain_good)` → `ResetToMapPose`。注意：稳定门控接入后 HandlePendingInitPose 的"验证通过→SetGood"需改为"→Initializing+门控放行"。
- lightning 置信度参考：lidar_loc.cc:1566/1595/1743（TP）；1466/1620（覆盖率，本次不做）。
- 现有 ndt yaml：threads=1, resolution=1.0, min_confidence=1.0(待重标), max_delta_trans_m=1.0, max_delta_rot_deg=10.0, good_rate_hz=1.0, gain_good=0.5。
- 构建必须在 lightning-jazzy:dev 容器内 + `--base-paths src/hikari_loclite`（root 有 AMENT_IGNORE）。
- 禁止链接 lightning.libs / include 其头文件。
