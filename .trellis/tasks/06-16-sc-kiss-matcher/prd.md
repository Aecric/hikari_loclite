# 重定位砍掉 SC 改用 KISS-Matcher 全局配准

## Goal

重定位**默认改用 KISS-Matcher 全局点云配准**做冷启动首帧 / 手动重定位（SC 代码保留但默认禁用，可切回做 A/B/回退）。SC 在走廊场景结构性失效（ring-key 沿轴不可观 → "收向原点"，详见 [[sc-reloc-domain-mismatch]] / [[corridor-drift-frontend-architecture]]）。KISS-Matcher 用真实 3D 特征对应 + GNC 鲁棒求解，无需初值出全局 6DOF，判别力远强于 SC。重定位是一次性、有界操作，**允许高 CPU（单次 ≤20s 可接受）**。

## 用户约束 (2026-06-16)
- 冷启动重定位单次 **≤20s 可接受**；机载 **RAM 16G** 充足；`global.pcd` **室内 600–800㎡ @ 0.2m 体素**（小图）；**接受 vendor 第三方库构建**。

## 关键前提（研究已确认，见 Research References）
- workspace 已 vendor **KISS-Matcher v1.0.2**（`src/lightning-lm/thirdparty/3rd/KISS-Matcher/`），lightning 有同款用例（固定图+累积查询+inlier 闸）封装 `kiss_matcher_wrapper.{h,cc}` + CMake 接线 + PoC，**跑过 ARM64**。本任务 = 把已验证模式平移进 hikari（不链 lightning.libs，自己 vendor 一份）。
- KISS API：`KISSMatcher::estimate(vector<Vector3f> src, tgt) → RegistrationSolution{valid, R(3x3), t}`；置信闸 `getNumRotationInliers()/getNumFinalInliers()`；**无需初值**；`R` 需 `Eigen::Quaterniond` 正交化后再构 SO3。MIT 许可、C++17。

## Requirements

### R1 — vendor KISS-Matcher 进 hikari
- 离线 vendor `cpp/kiss_matcher` 核心 + ROBIN(v1.2.7) + PMC(pin commit, 勿用 master) + TEASER++(v2.0) 进 `src/hikari_loclite/thirdparty/`（**不联网 FetchContent**）。
- CMake 照搬 lightning（`src/lightning-lm/CMakeLists.txt:71-139`）：`add_subdirectory` vendored 源码 + `USE_KISS_MATCHER` 编译开关 + `#ifdef` 守卫（未编时降级）+ robin `find_package` 修复 + `NOT TARGET` 幂等守卫。
- **绝不引** `KISS-Matcher/ros/` wrapper（绑 gtsam/visualization_msgs/pcl_ros）。

### R2 — KissMatcherWrapper（移植 lightning RunKiss 内核）
- 落 `src/reloc/`（如 `kiss_global_matcher.{hpp,cpp}`），命名空间 `hikari::loclite`，`RCLCPP_*` 日志，去 glog。
- 懒加载 `global.pcd` → 预降采样 `target_pre_voxel=0.2m` 缓存为 `vector<Vector3f>` target。**整图 MatchGlobal**（小图无需 crop）；保留 `max_target_pts=800000` 护栏（超限告警跳过）。
- `estimate(query)` → `RegistrationSolution`；置信闸 `valid && rotation_inliers>=min_rotation_inliers && final_inliers>=min_final_inliers`；`R` 经 `Eigen::Quaterniond` 正交化 → 拼 SE3（hikari 有 vendored Sophus）。

### R3 — RelocManager 加 KISS backend（保留 SC，默认禁用）
- **保留复用**：`Arm/Disarm`、`AccumulateScan`/`BuildAccumulatedQueryCloud`（20 帧累积）、`GravityAlignCloud`（去 yaw 到 level 系）、cooldown/blackout、`RelocCandidate{valid,pose,score,source}`。
- **新增 KISS 路径**：`RunKissOnce`：累积 query → 重力对齐 → `KissGlobalMatcher.MatchGlobal`（target=`lio_->FixedMapCloud()`，P3 在 Init 时 `SetTarget`）→ inlier 闸 → **yaw 微扫**（KISS yaw ±9°@3°，逐个起点跑 `NdtCorrector::Validate`，取最佳 TP）→ 过 NDT 闸则 `pose=最佳 NDT 解`，`source="kiss"`。
- **backend 选择器**（用户改动 2026-06-16）：加 `reloc.reloc_backend`（`kiss`|`sc`，**默认 `kiss`**）。`TryRelocalize`/`ManualRelocalize` 按 backend 分派到 `RunKissOnce` 或既有 `RunScanContextOnce`。
- **SC 代码全部保留**（`ScanContextManager`/`sc_manager_`/SC database/`LoadPoses`/`KfIdToMapPose`/`kf_poses_`/`RunScanContextOnce`）：仅在 `reloc_backend==sc` 时加载/运行；默认 backend=kiss 时**不加载 SC database/poses、不跑 SC**（按 backend 守卫，避免无谓 IO/CPU）。

### R4 — 节点接线 + 配置（保留 SC 文件）
- **不删** `scan_context.{cc,h}`（保留编译）。
- `loclite_node.cpp`：SC 调用点（`TryScRelocalize`、手动服务 `hikari_loc/sc_reloc`）按 backend 分派 KISS/SC；手动服务名**保留** `hikari_loc/sc_reloc` 兼容下游；`sc/accum_cloud` 保留改发 KISS query 云；SC 专用调试话题（`sc/candidates`/`sc/init_guess`/`sc/gravity_check`）仅 backend=sc 时发布（默认 kiss 不发，但发布器/代码保留）。
- config：**保留** `reloc.sc_*` / `poses_txt`（SC 启用时仍用）；新增 `reloc.reloc_backend`(默认 `kiss`) + `reloc.kiss_*`：`kiss_voxel_size`(0.3)、`target_pre_voxel`(0.2)、`max_target_pts`(800000)、`min_rotation_inliers`(50)、`min_final_inliers`(30)、`yaw_refine_range_deg`(9)、`yaw_refine_step_deg`(3)。NDT 验证沿用 `ndt.*`；`reloc.sc_max_delta_*` 两 backend 共用 → 更名 `reloc.reloc_max_delta_*`（保留旧键兼容读取或直接更名, 实现时定）。

### R5 — CLAUDE.md 禁令重评
- `hikari_loclite/CLAUDE.md` "Do NOT add: KISS-Matcher" 正式解除并记录决策（按 [[pangolin-ui-scope-decision]] 先例：嵌入式重定位收益 > 依赖成本，用户拍板）。更新依赖清单。

## Acceptance Criteria
- [ ] 容器 Release 编译通过（`USE_KISS_MATCHER` ON；lightning-jazzy:dev，`--base-paths src/hikari_loclite`）。
- [ ] 默认 backend=kiss：重定位默认走 KISS；SC 代码保留可编译，`reloc_backend=sc` 时仍可切回（A/B 用）；默认 kiss 不加载 SC database/poses。
- [ ] KISS 重定位链路跑通：冷启动/手动触发 → 累积+重力对齐 query → KISS estimate → inlier 闸 → yaw 微扫 NDT → Reset；低置信/失败干净拒绝（日志）。
- [ ] bag 冒烟（AI 自测）：走廊远端冷启动**不再收向原点**（候选 pos 非原点；或干净拒绝），并量出单次耗时（目标 ≤20s）。
- [ ] CLAUDE.md + config 更新一致。
- [ ] 行为终验（真机/bag 准确率、耗时体感）由用户完成。

## Definition of Done
- 容器 Release 构建绿；KISS 路径冒烟通过；SC 无残留；CLAUDE.md/config/依赖清单更新；trellis-check 通过。

## Technical Approach
重定位管线（冷启动 + 手动，Q3）：
```
accumulate 20 frames (existing) → gravity-align to level (existing)
  → KISS.estimate(query_level, target_global)        # 全局 6DOF, 无需初值
  → gate: valid && rot_inliers>=50 && final_inliers>=30; R 正交化
  → yaw 微扫: for d in [-9,-6,-3,0,3,6,9]°: NDT.Validate(pose(R·Rz(d), t)); keep best TP
  → best TP 过 NDT 闸? → ResetToMapPose(best) + smoother.Reset + 进稳定门控/Good
                       : 拒绝, 等 cooldown 重试 / 转 WAIT
```
target = 整张 global.pcd 预降采样 0.2m 缓存（小图，无需 crop）。

## Decision (ADR-lite)
- **算法 (Q1)**: 两者结合 — 真 KISS-Matcher 粗 6DOF → yaw 微扫 → NDT 精修。
- **触发 (Q3)**: 冷启动自动 + 手动服务；LOST 不自动（走 lost_timeout→WAIT→/initialpose）。
- **目标域 (Q2)**: 整图 MatchGlobal（小图 + 冷启动无先验），max_target_pts 护栏兜底。
- **SC 去留 (Q4, 用户改 2026-06-16)**: **保留 SC 代码，默认禁用**（`reloc.reloc_backend=kiss` 默认）。加 backend 选择器, SC 可切回做 A/B/回退。poses.txt 保留（仅 SC backend 用; 默认 kiss 不加载）。
- **yaw (Q6)**: KISS yaw ±9°@3° 多起点 NDT 取最佳 TP（退化场景保险丝）。
- **依赖 (Q5/R5)**: 引 KISS（连带 ROBIN/PMC/TEASER/TBB/flann/lz4，全 permissive），解除 CLAUDE.md 禁令。

## Out of Scope
- 走廊轴向几何退化的根治（运动累积 / 外部先验）—— KISS 比 SC 强但非根治，靠 inlier 闸 + NDT 双重拦截。
- LOST 态自动重定位（仅冷启动 + 手动）。
- crop 子图 / 大图分块（本图小，整图即可；护栏保留但不触发）。
- 逐帧热路径性能（仅重定位允许高 CPU）。

## Implementation Plan (phased)
- **P1 vendor + 构建**: KISS-Matcher(+ROBIN/PMC/TEASER pin) 进 `thirdparty/`；CMake `USE_KISS_MATCHER` 接线；空 hook 编过（容器 Release 验证）。
- **P2 wrapper**: `kiss_global_matcher.{hpp,cpp}`（target 加载/voxel、estimate、inlier 闸、R 正交化）；小冒烟。
- **P3 接线 + backend 切换**: RelocManager 加 KISS 路径（复用累积/对齐 + yaw 微扫 NDT）+ `reloc_backend` 选择器（默认 kiss）；节点调用点按 backend 分派；config knob；**保留 SC 代码默认禁用**；CLAUDE.md 更新。容器构建 + bag 冒烟 + 量耗时。

## Technical Notes
- 移植源（不链, 自 vendor 一份）：`src/lightning-lm/thirdparty/3rd/KISS-Matcher/`、`src/lightning-lm/src/core/localization/kiss_matcher_wrapper.{h,cc}`（RunKiss 内核 :150-206）、`src/lightning-lm/CMakeLists.txt:71-139`（CMake 接线）、`src/lightning-lm/src/app/poc_kiss_global.cc`（耗时 PoC 可参考）。
- hikari 改造落点：`src/reloc/reloc_manager.*`、新 `src/reloc/kiss_global_matcher.*`、`src/lio/scan_context.*`(删)、`src/system/loclite_node.cpp`（SC 调用点 ~957 / 手动服务 ~275 / SC 调试话题）、`src/ndt/ndt_corrector.*`（yaw 微扫复用 Validate）、`CMakeLists.txt`、`config/loclite_livox.yaml`、`CLAUDE.md`、`thirdparty/`。
- 编译风险：TBB+TEASER+PMC 同编吃内存 → 控 `BUILD_JOBS`；PMC vendor 时 pin commit。TBB 运行期 .so 需在镜像。
- 相关 memory：[[docker-jazzy-build-procedure]]、[[pangolin-ui-scope-decision]]、[[sc-reloc-domain-mismatch]]。

## Research References
- [`research/kiss-matcher-integration.md`](research/kiss-matcher-integration.md) — KISS-Matcher 集成 6 点调研（API/依赖/ARM64/大图/退化/集成）+ 总结论：已 vendor + lightning 已验证，引入=平移已验证模式。
