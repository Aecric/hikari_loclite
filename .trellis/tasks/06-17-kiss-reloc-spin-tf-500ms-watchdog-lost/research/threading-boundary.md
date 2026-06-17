# 研究: KISS reloc 异步化的线程边界与共享状态

## 根因复盘（hunt 已确认，A/B 坐实：关 KISS→100ms）

`TryScRelocalize`(`src/system/loclite_node.cpp:1268`) 在 `ProcessFrame` 内联调用 `reloc_->TryRelocalize/ManualRelocalize` → `RunKissOnce`(`src/reloc/reloc_manager.cpp:470`，**无线程**) 跑在 `rclcpp::spin` 单线程上，阻塞数百ms~秒级（≤20s）：
- 症状1 TF 500ms：lidar `QoS(5).best_effort()`(`:281`) 深度5，阻塞期 5 帧灌满 → 常驻队列 5×100ms。
- 症状2 误LOST：`health_timer_` 200ms wall timer(`:415`) 同 executor 被堵，释放后 `lidar_age` 超 `lidar_timeout_sec=2.0` → `SetLost("input_timeout")`。
- 死循环：误LOST→LOST 重新 arm reloc→又触发 KISS。

## RunKissOnce 内部拆解（决定线程边界）

| 步骤 | 触碰的状态 | 归属 |
|---|---|---|
| 构造查询云 `BuildAccumulatedQueryCloud()`(`:246`) | 读 `accum_buffer_`（主线程 `AccumulateScan`(`:222`) 每帧写）**← 竞争点** | **主线程持锁快照** |
| `GravityAlignCloud`(`:271`) | 纯函数（用 current_imu_pose） | 主线程或 worker 均可，建议主线程一起快照 |
| 写 `last_query_cloud_`/`debug_`(`:475,512`) | 共享成员，仅供调试话题 | 主线程设置（或加锁），别在 worker 裸写 |
| `kiss_.MatchGlobal(cloud_level)`(`:518`) | **重活**；`MatchGlobal`(`kiss_global_matcher.cpp:75`) 每次 new 局部 `KISSMatcher`，只读 `target_vec_`（SetTarget 后只读）→ **读安全** | **worker** |
| yaw 微扫循环(`:545-558`) | 调 `ndt->Validate(guess, scan, ...)`，用传入 ndt + 当前 scan | **worker**（用独立 ndt + 快照 scan） |

## 验证结论

- `kiss_.MatchGlobal` 对共享态只读（local matcher + 只读 target）→ 单 worker 保证下从 worker 调用安全。
- `NdtCorrector`(`ndt_corrector.cpp:65`) 每实例自持 `new NdtType()`，无 static/global → **第二独立实例安全**；代价=多一份 target 体素协方差格+kdtree 内存。
- `accum_buffer_` 全部访问（AccumulateScan/Build/Clear）必须留在主线程持 `mutex_`；worker 不得调 `BuildAccumulatedQueryCloud`。

## 落地线程边界

1. **主线程派发(持 mutex_)**：过 cooldown/blackout/帧数门 + 无在飞 worker → `cloud_level = GravityAlign(BuildAccumulatedQueryCloud())`，快照 `{cloud_level, scan, current_imu_pose, reloc_gen}`，置 `inflight`，启 worker，立即 return。
2. **worker(独立 ndt_reloc_)**：`MatchGlobal(cloud_level)` + yaw 微扫(`ndt_reloc_->Validate(guess, scan_snapshot)`) → 粗候选 `{valid, T_map_lidar, gen}` 存入 result_mutex_ 保护成员。
3. **主线程收尾(每帧 poll，持 mutex_)**：结果就绪且 gen 匹配 → `ndt_->Validate(candidate, 当前实时 scan, reloc_max_delta_*)` 复核（抗陈旧）→ 过则 `RecoverFromLost`，否则丢弃等下个 cooldown。

## 改动面预判

- 新方法（建议 RelocManager 上）：`PrepareKissQueryLevel()`（主线程，读 accum 缓冲）+ `RunKissMatchOnSnapshot(cloud_level, scan, imu_pose, NdtCorrector* ndt_worker)`（worker，只读 kiss_）。
- 节点新增成员：`std::unique_ptr<NdtCorrector> ndt_reloc_`、worker future/thread、`std::mutex result_mutex_`、`bool reloc_inflight_`、`uint64_t reloc_gen_`、候选结果缓存。
- 析构 join/wait worker，防悬空 this。
- lidar QoS → `KeepLast(1).best_effort()`(`:281,286`)。
- auto(`:716,730`) + manual 服务(`:320-395`) 统一走此异步路径；manual Trigger 立即返回 "started"。

参见根因记忆 [[kiss-sync-spin-thread-blocks-pipeline]]、构建流程 [[docker-jazzy-build-procedure]]。
