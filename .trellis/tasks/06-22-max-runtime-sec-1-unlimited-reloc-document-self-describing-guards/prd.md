# WaitForInitialPose 持续自动重定位 + max_runtime_sec=-1 不限时

## Goal

让 `WaitForInitialPose` 状态成为一个**持续自动重定位**的状态：无论因何进入（冷启动 / LOST 超时 / `/initialpose` 被 NDT 拒绝），只要配置 `reloc.auto_on_init: true` 且 backend ready，就 (重新) arm 并自动跑全局重定位配准；配 `reloc.max_runtime_sec=-1`（`<=0`）即**不限时、永不放弃**，直到成功或收到有效 `/initialpose`。`/initialpose` 到来打断自动配准、改用人工位姿；该 initialpose 被 NDT 拒绝则回到 WAIT 并恢复自动配准。LOST 维持现状、与本逻辑解耦。

## 用户原始需求（2026-06-22 澄清，逐条）

1. 状态机变为 `WaitForInitialPose` 时——**不管来自 LOST 还是冷启动**——若 `auto_on_init: true` 则自动启动重定位配准。
2. **除非**收到 `/initialpose` → 打断自动配准，改用 initialpose。
3. **除非**本次 initialpose 被 NDT 拒绝 → 重新启用自动配准。
4. LOST 状态下是 LOST 自己尝试救（局部 NDT 恢复 / `auto_on_lost`），**与 `WaitForInitialPose` 状态机无关**。

## 现状核实（已读码）

- **不变量缺口**：进入 `WaitForInitialPose` 后 reloc 是否 armed 不一致——
  - 冷启动 `loclite_node.cpp:314-318`：SetWAIT + `if auto_on_init: Arm("auto_on_init")` ✓（已是想要的）
  - LOST 超时 `:769-770`：SetWAIT + `Disarm("lost_timeout")`，之后无人 re-arm ✗
  - initialpose 被拒/重试耗尽 `:846 / :899 / :909`：仅 SetWAIT，reloc 维持 `:583` 的 disarmed ✗
- **打断已实现**：`OnInitialPose :582-584` 收到 initialpose 即 `Disarm("initialpose_blackout")` + `++reloc_gen_`（作废在飞 KISS worker）+ blackout 窗口 + 转 `Initializing`。
- **WAIT 态自动 reloc 入口**：`:815-819`，条件 `Armed() && AutoOnInit() && RelocReady()`（KISS 走异步 worker）。只要进 WAIT 时 armed，就会逐帧跑。
- **不限时已被守门支持**：`max_runtime_sec` 超时 disarm 两处都前置 `> 0.0`（`reloc_manager.cpp:338`、`loclite_node.cpp:1525`），`<=0` 直接跳过；解析 `:39 .as<double>(10.0)` 无 clamp。WAIT 态除 `max_runtime_sec` 外无其它超时 disarm 自动 reloc，故 `-1` ⟹ WAIT 自动 reloc 真正不限时。
- **LOST 独立**：`:761-805` LOST 自有处理（lost_timeout 转 WAIT、局部 NDT 恢复、`auto_on_lost` 全局 reloc），不动。

## 核心设计：WaitForInitialPose ⟺ reloc-armed 不变量

引入 `LocLiteNode::EnterWaitForInitialPose(const char* reason)` 统一所有进入 WAIT 的转移：
```
state_machine_.SetWaitForInitialPose(reason);
if (reloc_->AutoOnInit() && reloc_->RelocReady())
    reloc_->Arm(reason, this->now().seconds());   // 冷启动/LOST/拒绝 一致
// 否则保持 disarmed（auto_on_init=false 时仍纯等 /initialpose）
```
替换 `:769/:846/:899/:909` 处的 `SetWaitForInitialPose(...)`（`:769` 同时删掉紧随的 `Disarm("lost_timeout")`）。冷启动 `:314-318` 也收敛到该 helper。
- `/initialpose` 仍在 `OnInitialPose` 打断（不变）；blackout 窗口期内即便 re-arm，自动 reloc dispatch 仍被 `:1518` blackout 节流，给 initialpose 公平窗口——可接受。
- `max_runtime_sec` 语义：`<=0` ⟹ WAIT 自动 reloc 不限时；`>0` ⟹ 每次进入 WAIT 的自动 reloc 跑满该窗口后 disarm 停下（等下次 `/initialpose` 或状态再入 WAIT），re-arm 只在**状态进入** WAIT 时触发，不会把有限上限变成无限循环。

## 自描述 + 文档（沿用上一轮已批方案）

- `RelocManager` 加 `bool RuntimeUnlimited() const { return max_runtime_sec_ <= 0.0; }`，两处守门改 `!RuntimeUnlimited()`。
- `Arm()` 时若 `RuntimeUnlimited()` 打一条 INFO（unlimited runtime, no timeout disarm）。
- `config/loclite_livox.yaml:76` 注释写明 `-1`(`<=0`)=不限时；并在 `auto_on_init` 注释补一句“控 WAIT 态是否持续自动 reloc”。
- spec `runtime-and-relocalization.md` 补「WaitForInitialPose ⟺ reloc-armed 不变量」+ `max_runtime_sec<=0` 契约 + 与 LOST 的解耦说明。

## Requirements

- R1: 新增 `EnterWaitForInitialPose` helper 统一 5 处转移，落地 `auto_on_init ⟹ arm` 不变量；删 `:770` 的 lost_timeout disarm。
- R2: `RuntimeUnlimited()` 自描述守门（两处）+ arm 时 unlimited INFO 日志。
- R3: config 注释（`max_runtime_sec` + `auto_on_init`）。
- R4: spec 文档（不变量 + `<=0` 契约 + LOST 解耦）。
- R5: `/initialpose` 打断 + NDT 拒绝后恢复，全链路验证（拒绝路径已能进 WAIT，靠 R1 恢复自动 reloc）。

## Acceptance Criteria

- [ ] `colcon build --packages-select hikari_loclite` 通过。
- [ ] `auto_on_init=true, max_runtime_sec=-1`：冷启动找不到图时持续重试不放弃。
- [ ] LOST 跟丢 → lost_timeout 转 WAIT 后，自动 reloc 持续运行（不再永久 disarmed 干等）。
- [ ] 下发 `/initialpose` 打断自动 reloc；该 pose 被 NDT 拒绝/重试耗尽后回 WAIT，自动 reloc 恢复运行。
- [ ] `auto_on_init=false` 时 WAIT 态不自动 reloc（纯等 /initialpose），行为不回归。
- [ ] arm 日志可见 unlimited 模式；config/spec 记载契约。

## Definition of Done

- 不变量集中在单一 helper，5 处转移一致；行为可由日志审计。
- LOST 路径未改动（解耦）。
- config 注释 + spec 同步；默认 `max_runtime_sec` 维持 20.0（仅说明 -1 合法）。

## Out of Scope

- 不改 LOST 逻辑、`lost_timeout_sec`、`auto_on_lost`（用户明确 LOST 解耦）。
- 不新增配置开关（行为由既有 `auto_on_init` + `max_runtime_sec` 表达）。
- 不动 cooldown / blackout / NDT / 累积等其它 reloc 子系统（除引用其守门）。

## Technical Notes

- 守门两处：`reloc_manager.cpp:338`、`loclite_node.cpp:1525`；`Arm/Disarm`：`reloc_manager.cpp:188-202`。
- SetWaitForInitialPose 调用点：`:314`(startup) `:769`(lost_timeout) `:846`(init_accum_timeout) `:899`(init_retry_exhausted) `:909`(init_accum_rejected)。
- `OnInitialPose`：`:537-591`（打断逻辑：disarm + gen bump + blackout + SetInitializing）。
- WAIT 自动 reloc 入口：`:815-823`；blackout 节流：`:1516-1523`。
- spec 时钟域条目已有 “Reloc runtime limit / Arm timestamps: wall vs wall”（runtime-and-relocalization.md:297）。
