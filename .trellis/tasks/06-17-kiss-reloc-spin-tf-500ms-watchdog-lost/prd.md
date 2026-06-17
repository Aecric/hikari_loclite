# KISS reloc 异步化: 移出单线程 spin 修复 TF 500ms 延迟与 watchdog 误判 LOST

## Goal

把 KISS 全局重定位从 `rclcpp::spin` 单线程上**内联同步**执行改为**不阻塞主线程**，消除两个由同一根因产生的现网问题：
(1) `map->livox_frame` TF 恒定滞后 ~500ms；(2) watchdog 因主线程被 KISS 冻结而误判 `input_timeout` 进 LOST，并与 LOST 重新 arm reloc 形成死循环。

## Root Cause（hunt 已确认）

- `TryScRelocalize`（`src/system/loclite_node.cpp:1268`）在 `ProcessFrame` 内联调用 `reloc_->TryRelocalize/ManualRelocalize`，后者 `RunKissOnce`（`src/reloc/reloc_manager.cpp:470`，**无 std::thread/async**）跑在 spin 线程上，单次可达数百 ms～秒级（设计允许 ≤20s）。
- 节点入口 `run_loclite_online.cpp:51` 是 `rclcpp::spin(node)` **单线程**；`ProcessFrame` 全程持 `mutex_`。
- 症状 1：lidar 订阅 `rclcpp::QoS(5).best_effort()`（`loclite_node.cpp:281`，深度5，好版本 d1dc414 即如此、非本次回归）。KISS 阻塞期间 5 帧灌满队列，恢复后 executor 永远取最旧帧 → 5×100ms=500ms 常驻队列。TF 盖 `state.timestamp_=lidar_end_time`（新鲜）→ 真实延迟，非时戳 bug。
- 症状 2：`health_timer_` 200ms wall timer（`:415`）也在同一 executor，KISS 阻塞时无法触发；释放后立刻算出 `lidar_age=now−last_lidar_wall_ts_` 超 `lidar_timeout_sec=2.0`（或 imu 1.0）→ `SetLost("input_timeout")`。
- 死循环：KISS 阻塞→watchdog 误 LOST→LOST 重新 arm reloc→下一帧又触发 KISS→…（解释为何延迟稳定 500ms 而非偶发尖峰）。

## Verification（已有证据）

- A/B：**关 KISS → 延迟降到 ~100ms（=1帧正常值）、误 LOST 消失**（用户实测）。直接坐实根因。

## What I already know

- 修复方向：KISS 移到 **per-attempt 工作线程**（一次性跑完即退，不违反"无持久后台线程"约束）；spin 线程继续 tracking/TF/IMU/watchdog；结果回来后在后续帧持锁做 NDT 校验 + `RecoverFromLost(ResetToMapPose)`。
- 触发点：自动 reloc 两处 `TryScRelocalize(scan, ts)`（init 路径 `:730`、LOST 路径 `:716`）+ 手动 reloc 服务（`:341` 调 `ManualRelocalize`，同样同步阻塞）。
- 候选回来后的收尾 `RecoverFromLost` 会改 LIO/状态机/smoother → **必须主线程持锁**。

## 关键设计约束

- **`ndt_` 非线程安全且被共享**：KISS 内部 yaw 微扫用 `ndt_`，校验也用 `ndt_->Validate`；而主线程在 LOST 态并发跑 NDT-LocalRecovery（3Hz）、GOOD 态跑漂移校正。worker 直接用 `ndt_` 会与主线程竞争。需决定：worker 用**独立 NDT 实例**（干净，+内存）还是加锁复用（有再串行化风险）。

## Decisions

- **[Q1] MVP = 异步化 + lidar QoS 改 KeepLast(1)**；**不**调 watchdog 阈值（异步后 watchdog 不再被饿死，调大只会掩盖真实掉线）。
- **[Q2] worker 用独立 NDT 实例**（同一固定地图，专供 reloc），无共享竞争、逻辑最干净；接受嵌入式多一份 target 体素协方差格+kdtree 的内存代价。

- **[Q3] auto + manual 统一走同一异步 worker**；`/hikari_loc/sc_reloc`(Trigger) 立即返回 `success=true / "reloc started"`（=已受理，非已定位），结果后续帧异步应用。
- **[范围] 异步只覆盖 KISS backend**（默认、唯一肇事者）；SC backend（默认关闭、查询轻量）保持同步 `TryScRelocalize`，避免 `debug_` 跨线程竞争、refactor 减半。

## 实现状态（2026-06-17，容器内 jazzy Release 构建通过 53.2s）

- `RelocManager`: `RunKissOnce` 拆为 `PrepareKissQueryLevel`(主线程读 accum 缓冲, 建 level 查询云) + `MatchKissOnSnapshot`(const, worker 只读 kiss_/config, 用传入独立 ndt 跑 MatchGlobal+yaw 微扫); `RunKissOnce` 退化为二者组合。加 `ArmedElapsed()` 供异步路径自判 max_runtime。
- `LocLiteNode`: 新增独立 `ndt_reloc_` 实例(仅 kiss backend) + `MaybeDispatchKissReloc`(持锁快照→`std::async` per-attempt worker, worker 入口 `DropThreadToNormalSched()` 防抢占 RT spin) + `CollectKissReloc`(每帧 poll, gen 守卫 + 状态守卫(手动豁免) + 当前帧 scan 复核 + `RecoverFromLost`)。`reloc_gen_` 在 `/initialpose`/`RecoverFromLost`/手动触发 bump。auto 两处派发点 + 手动服务按 backend 分支(kiss→异步立即返回 started)。`Shutdown` join worker。
- lidar 订阅 QoS `QoS(5).best_effort()` → `QoS(KeepLast(1)).best_effort()`(两处)。
- CMake: `find_package(Threads)` + link `Threads::Threads`。

## 待验证（用户实机 / bag）

- [ ] 开 KISS 时 `map->livox_frame` TF 延迟稳定 ≈100ms（=关 KISS 基线）
- [ ] KISS 查询进行期间不再触发 watchdog 误判 LOST
- [ ] 冷启动自动 KISS / 手动 reloc / LOST 恢复功能不退化（候选仍经 NDT 复核）

## Technical Approach

**异步 reloc worker（per-attempt，一次性 std::async/std::thread，跑完即退）**

- **主线程派发**（`ProcessFrame` / 服务回调，持 `mutex_`）：过 cooldown/blackout/帧数门后，若无在飞 worker，则快照 `{查询点云副本, current_imu_pose, T_lio_snapshot, reloc_gen}`，置 `reloc_inflight_=true`，启动 worker，**立即返回**。
- **worker 线程**：仅跑 KISS 重活（特征匹配 + GNC + yaw 微扫），用**独立 NDT 实例**（[Q2]）；产出粗候选 `{valid, pose, gen}` 存入受 `result_mutex_` 保护的成员 / future。**不**触碰 `ndt_`/`lio_`/状态机。
- **主线程收尾**（每帧 poll，持 `mutex_`）：发现结果就绪且 `gen` 仍匹配 → 用 `ndt_->Validate(candidate, 当前实时 scan, reloc_max_delta_*)` 复核（解决 KISS 耗时数秒期间机器人移动导致候选陈旧的问题；validate 单次 align 有界、留在主线程）→ 通过则 `RecoverFromLost(ResetToMapPose)`，否则丢弃等下个 cooldown。
- **lidar QoS** 改 `rclcpp::KeepLast(1).best_effort()`（[Q1]）。

## Requirements

- KISS 全局配准不在 spin 线程上执行；spin 线程在 KISS 期间持续产出 TF/odom/处理 IMU/喂 watchdog。
- 同一时刻至多一个在飞 KISS worker；cooldown + `reloc_inflight_` 双重去重。
- worker 用独立 NDT 实例，与主线程 `ndt_` 零共享。
- 最终 NDT 校验在主线程用**当前帧** scan 做（抗陈旧）。
- `reloc_gen` 守卫：/initialpose、Disarm、手动触发都 bump generation；迟到的 worker 结果若 gen 不匹配则丢弃（防覆盖新 /initialpose）。
- 节点析构 / shutdown 时 join/wait worker，无悬空 `this`。
- lidar 订阅 QoS = KeepLast(1).best_effort()。

## Edge Cases (MVP-critical)

- 查询期间机器人移动 → 主线程对当前 scan 复核校验，不通过即丢弃。
- 查询期间收到 /initialpose → reloc_gen 不匹配，worker 结果作废。
- 查询期间转 GOOD（不应发生，因 reloc 在 init/LOST）→ 结果应用前检查当前态仍需要 reloc。
- worker 仍在飞时又到 cooldown → 跳过派发（inflight 守卫）。

## Acceptance Criteria (evolving)

- [ ] 开启 KISS 时 `map->livox_frame` TF 延迟稳定 ≈100ms（1帧），与关 KISS 时一致
- [ ] KISS 查询进行期间不再触发 watchdog 误判 LOST
- [ ] 重定位功能本身（init 冷启动 + 手动 + LOST 恢复）行为不退化
- [ ] 同一时刻至多一个在飞 KISS 查询；无数据竞争（ndt/lio/state）

## Out of Scope (explicit)

- 不改 KISS 算法本身 / 不改 SC backend 逻辑
- 不引入持久后台线程或线程池
- 不改 LIO 前端 / 不动 lidar_end_time 时戳逻辑

## Definition of Done

- 构建通过（容器内 jazzy，见 [[docker-jazzy-build-procedure]]）；实测 A/B 复验两项验收
- 行为变更记录到 README / 任务 journal

## Technical Notes

- 入口单线程：`rclcpp::spin`（`run_loclite_online.cpp:51`）
- 锁：`ProcessFrame` 全程持 `mutex_`（`loclite_node.cpp:517`），`OnImu` 也持 `mutex_`
- 候选收尾：`RecoverFromLost`（`loclite_node.cpp` 内）改 lio_/smoother_/state_machine_/reloc_
