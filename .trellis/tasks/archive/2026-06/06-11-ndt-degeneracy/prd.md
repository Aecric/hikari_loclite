# NDT 退化检测

## Goal

为 hikari_loclite 的 NDT 链路增加退化检测：当前门控仅 TP + delta + inlier，退化场景（长走廊/平面/开阔地）下 NDT 可能收敛到错误解且 TP 仍然偏高。pclomp NDT 的 `getHessionMatrix()` 已在 `computeTransformation` 末尾存储但从未被提取。lightning-lm 的 `DegeneracyEigenCheck`（gating_features.h:97-159）是成熟的参考实现。

## Requirements

1. **新增 `include/hikari_loclite/ndt/degeneracy_check.hpp`**（header-only, 无状态, 线程安全）
   - 从 lightning-lm `gating_features.h` 提取 `DegeneracyResult` 结构体 + `DegeneracyEigenCheck()` 函数
   - 移除 lightning 命名空间，改为 `hikari::loclite`
   - 移除 `R_body` 参数（简化：在世界系下分析，退化方向用于日志而非机体系投影）
   - 接口：`DegeneracyEigenCheck(hessian_6x6, ratio_threshold=50.0, min_ev_threshold=10.0)`
   - 返回：`DegeneracyResult { is_degenerated, trans_degenerated, rot_degenerated,
     trans_condition_ratio, rot_condition_ratio, trans_lambda_min, rot_lambda_min,
     trans_degen_dir, rot_degen_dir, trans_eigenvalues, rot_eigenvalues }`

2. **扩展 `include/hikari_loclite/ndt/ndt_corrector.hpp`**
   - `NdtResult` 新增字段：
     - `bool degenerate = false` — 退化标志
     - `double trans_condition_ratio = 0.0` — 平移条件数
     - `double rot_condition_ratio = 0.0` — 旋转条件数
   - 新增 `#include "ndt/degeneracy_check.hpp"`
   - 新增 yaml knobs (private members)：
     - `degeneracy_ratio_threshold_`（double, 默认 50.0）— λ_max/λ_min 超过此值判退化
     - `degeneracy_min_ev_threshold_`（double, 默认 10.0）— λ_min 低于此绝对值判退化
     - `degeneracy_reject_in_validate_`（bool, 默认 true）— Validate 中退化是否直接拒绝

3. **修改 `src/ndt/ndt_corrector.cpp`**
   - `Init()`：读 `ndt.degeneracy_ratio_threshold`, `ndt.degeneracy_min_ev_threshold`, `ndt.degeneracy_reject_in_validate`
   - `Align()`：收敛后调 `ndt_->getHessionMatrix()` → `DegeneracyEigenCheck()`，填入 `result.degenerate`, `result.trans_condition_ratio`, `result.rot_condition_ratio`。日志：退化时 `LOG(WARNING)` 打印 condition_ratio + lambda_min。
   - `Validate()`：若 `degeneracy_reject_in_validate_` 且 `result.degenerate`，直接 `result.valid = false`，日志标注 "degenerate rejected"。

4. **修改 `src/system/loclite_node.cpp`**
   - `MaybeNdtCorrectGood()`：收敛后若 `res.degenerate`，跳过本次校正（不打扰 LIO），日志打印退化方向 condition_ratio。
   - `HandlePendingInitPose()`：Validate 已在 NdtCorrector 内拒绝退化候选，无需额外改动。
   - `ndt_status` 话题（Int32）：不改消息类型，退化信息仅通过 RCLCPP 日志输出。

5. **修改 `config/loclite_livox.yaml`**
   ```yaml
   ndt:
     # ... existing knobs ...
     # Phase2: 退化检测
     degeneracy_ratio_threshold: 50.0    # λ_max/λ_min 超此值判退化 (走廊/平面典型 50~100)
     degeneracy_min_ev_threshold: 10.0   # λ_min 低于此绝对值判退化 (特征值与 TP 同量纲)
     degeneracy_reject_in_validate: true  # Validate 中退化直接拒绝 (init/SC 候选)
   ```

## Acceptance Criteria

- [ ] 容器内（lightning-jazzy:dev）Release 构建通过
- [ ] `git diff --check` 过
- [ ] 退化检测逻辑正确：Align 后 Hessian 被提取，日志带 condition_ratio
- [ ] Validate 退化拒绝生效（degeneracy_reject_in_validate=true 时）

## Out of Scope

- 退化方向用于 gain 调制（仅跳过/拒绝，不做方向性增益衰减）
- 退化信号接入状态机（Good→Degraded 加速，留后续）
- 退化信号接入 StabilityGate（Initializing 延迟放行，留后续）
- 体素级退化预测（alignment 前预判，留后续）

## Technical Notes

- pclomp NDT `getHessionMatrix()` 返回 `Eigen::Matrix<double,6,6>`，在 `ndt_omp_impl.hpp:254` 的 `computeTransformation` 末尾存储。
- lightning-lm 参考实现：`/home/aecriclin/3d_slam_ws/src/lightning-lm/src/core/localization/lidar_loc/gating_features.h` 第 46-159 行。
- 包内已有 Eigen/Eigenvalues 依赖（ESKF 已用 `SelfAdjointEigenSolver`），无需新增。
- 构建必须在 lightning-jazzy:dev 容器内 + `--base-paths src/hikari_loclite`。
