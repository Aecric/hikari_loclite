# Phase 6 产品化 — CPU affinity/RT + 输入 watchdog/富状态 + 日志限频 + systemd unit

## Goal

把 `hikari_loclite` 从"能跑"推到"能上嵌入式产线"。落地设计文档 `hikari_loclite_build_2026-06-10.md` 第 18 节 Phase 6「产品化」里**运行时侧**三项（CPU affinity / 日志限频 / watchdog 状态上报），并补一个薄的 systemd unit 让上述能力在真机生效。

**本 PRD 不是 Phase 6 全量**。用户 2026-06-15 决策：先做运行时优先（affinity + watchdog + 日志），systemd unit 因落地方式选了"service 内 + 节点自绑"而一并带上；**精简部署参数模板拆到后续任务**。

## 现状已核查（不要重新调研）

Phase 6 设计文档列 6 项，逐项核过真实代码：

| 项 | 现状 | 本 PRD |
|---|---|---|
| setcap | ✅ 已做 `docker2/postinst.in:43-50`（授 `cap_sys_nice+ep`） | 复用，不重做 |
| CPU affinity | ❌ `grep setaffinity\|SCHED_FIFO\|setschedparam` = **NONE**。cap 授了没人用 | **做** |
| 日志限频 | ⚠️ `log.h` 自研 shim 只有计数式 `LOG_EVERY_N`（无时间节流）；`grep THROTTLE` 命中 node 10 / fast_lio 5 处，未系统化。glog 风格 `LOG()` 55 处 + `RCLCPP_*` 61 处混用 | **做**（仅 hot-path 限频，不做 glog→RCLCPP 全迁移） |
| watchdog 状态上报 | ⚠️ 只有 NDT 驱动的 `lost_timeout_sec`（`loclite_node.cpp:542`）。`loc_state`/`ndt_status`(Int32) + `loc_status`(Marker) 已发（`:259-261`）。**无输入掉线检测、无富诊断** | **做** |
| systemd service | ❌ `find *.service` = 无 | **做**（薄 unit，随 .deb 装，不 enable、不 sd_notify） |
| 精简部署参数模板 | ⚠️ 仅 `config/loclite_livox.yaml`（开发用全注释 12KB） | **不做**，拆后续任务 |

### 关键代码锚点（已读）

- **入口**：`src/app/run_loclite_online.cpp` — `main()` 在 `node->Init()` 后、`rclcpp::spin()` 前（`:37-46`）。CPU 绑核/RT 设置落这里（cap 已授，spin 前一次性设当前线程）。
- **回调驱动**（非 wall-timer 主循环）：`OnImu`(`:375`) 加 IMU；`OnLivox`(`:385`)/`OnCloud`(`:393`) 加点云后调 `ProcessFrame()`（`:449`）。→ 输入时戳在这三个回调里记。
- **已有 1Hz timer**：`wait_state_timer_`（`:358`，仅 WAIT 态上报）。健康检查可扩此 timer 或新建独立 timer（不依赖点云到达，掉线时仍要跑）。
- **状态上报**：`PublishStatusTopics()`(`:868`) 发 `loc_state`/`ndt_status`，`PublishStatusMarker()`(`:882`) 发文字 Marker；`last_ndt_confidence_` 已有成员可复用。
- **config 现有顶层段**：`common / system / reloc / runtime / fixed_map / fast_lio`（`config/loclite_livox.yaml`）。新 knob 就近挂 `runtime.*`（watchdog/日志）与新 `system.rt_*`（绑核/调度）。

## Requirements

### 6A — CPU affinity + 实时调度（节点自绑，yaml 驱动）

1. 在 `run_loclite_online.cpp main()`、`Init()` 成功后 `spin()` 前，对当前（spin）线程设置：
   - `sched_setaffinity()` 绑到 yaml 指定核列表；
   - `pthread_setschedparam(SCHED_FIFO, prio)` 设实时优先级。
   - 复用 postinst 已授的 `cap_sys_nice`；**无权限时降级为 warning 继续跑**，不 fail（开发机无 cap、bag 回放要能起）。
2. yaml knob（新 `system.rt_*`，附中文注释 + "需 cap_sys_nice"）：
   - `rt_enabled`（默认 `false`，产线模板再开）、`rt_cpu_cores`（如 `[2,3]`，空=不绑）、`rt_sched_fifo`（bool）、`rt_priority`（如 `80`）。
3. 实现放一个小 helper（如 `src/system/realtime_setup.*` 或 main 内静态函数），纯 POSIX，无新依赖。

### 6B — 输入掉线 watchdog + 富状态话题

1. **输入时戳**：`OnImu`/`OnLivox`/`OnCloud` 各记 `last_imu_ts_` / `last_lidar_ts_`（壁钟 `now()`，**不用消息内 stamp**——bag 回放/时钟域坑见 memory `sc-blackout-clock-bug`，watchdog 走墙钟）。
2. **健康 timer**（独立 wall timer，~2–5Hz；掉线时点云不来也要跑）：
   - IMU 静默 > `imu_timeout_sec` 或 Lidar 静默 > `lidar_timeout_sec` → 置降级/LOST + `RCLCPP_WARN_THROTTLE` 告警；恢复后回正常。
   - 与现有 `state_machine_` 协作：掉线属"输入级"故障，进 LOST/告警但**不污染 Fast-LIO 状态**（不 reset），输入恢复后继续。
3. **富状态话题** `/hikari_loc/status`（用现有依赖内消息，**不引 visualization/diagnostic_msgs 新依赖**——`std_msgs/Float32MultiArray` 或 `String`(JSON)，二选一在实现时定）：字段 = `state`、`ndt_confidence`、`imu_age_s`、`lidar_age_s`、`frame_fps`、`in_map`(bool)。沿用 `PublishStatusTopics` 节奏发布。
4. 现有 `loc_state`/`ndt_status`/Marker **保留不动**，富话题是叠加。
5. yaml knob（`runtime.*`）：`watchdog_enabled`、`imu_timeout_sec`（默认 ~0.5）、`lidar_timeout_sec`（默认 ~1.0）、`status_topic_enabled`。

### 6C — 日志限频系统化

1. 审 hot-path（`ProcessFrame`/`RunOnce`/NDT/每帧回调）里每帧级 `LOG(INFO)`/`RCLCPP_INFO`，统一改时间节流：`RCLCPP_*_THROTTLE`（ROS 侧）或给 `log.h` 加**时间式** `LOG_EVERY_T(sev, sec)` 宏（现仅有计数式 `LOG_EVERY_N`）。
2. 目标：稳态 GOOD 下 stdout/stderr 行频可控（量级目标实现时定，例如关键状态 ≤1–2 行/s），**异常/状态翻转日志保持即时**（不限频）。
3. **不做** glog`LOG()`→`RCLCPP` 全量迁移（55 处，超本任务边界，CLAUDE.md 偏好 RCLCPP 但属独立清理项）。只动会刷屏的热点。

### 6D — systemd unit（薄，随 .deb）

1. `docker2/hikari-loclite.service.in`（占位符同现有 `postinst.in`/`postrm.in` 风格，如 `__ROS_DISTRO__`）：
   - `ExecStart` = `run_loclite_online`（source ROS env + 传 config 路径）；
   - `Restart=on-failure`、`RestartSec`；`After=network.target`。
   - **不接** `sd_notify`/`WatchdogSec`（用户未选 sd_notify；保持薄）。
   - **不** `[Install] WantedBy` 自动 enable —— 装到 `/lib/systemd/system/`，交付方手动 `systemctl enable`。
2. 由 `docker2/Dockerfile` + CI 打包流程（任务 `06-12-add-hikari-loclite-packaging-ci`）把 `.service` 渲染并装进 .deb。本任务只产 unit 模板 + 接打包；**不改定位运行时行为**。

## 约束

- 不新增运行时依赖（禁 `diagnostic_msgs`/`visualization` 之外的新包；富状态用现有 `std_msgs`）；不链 `lightning.libs`；不抽 lightning 代码。
- affinity/RT 无权限必须**优雅降级**（warning + 继续），开发机/bag 回放零门槛可跑。
- watchdog 走**墙钟**，不比较消息 stamp（见 `sc-blackout-clock-bug`）；掉线**不 reset** Fast-LIO 状态。
- 落点限于 `run_loclite_online.cpp` + `loclite_node.{hpp,cpp}` + 新 `realtime_setup`/`log.h` 宏 + `docker2/*.service.in` + yaml；**不碰** NDT/SC 算法、状态机转移条件、固定图逻辑。
- 嵌入式预算：watchdog 仅时戳比较 + 低频 timer；富话题低频发布；日志限频净降 CPU。

## Out of Scope（明确拆走）

- 精简部署参数模板（deploy 版 yaml）—— 后续独立任务。
- systemd `sd_notify`/`WatchdogSec` 心跳与进程级自愈。
- glog `LOG()` → `RCLCPP` 全量迁移。
- 改 setcap（已由 CI postinst 完成）。

## 验收

- `colcon build --packages-select hikari_loclite` 通过（AI 负责编译通过）。
- 6A：yaml 开 `rt_enabled` + 有 cap 时绑核/RT 生效；无 cap 时 warning 不崩。
- 6B：拔 IMU/Lidar（停发 topic）→ 数秒内进 LOST + 告警；`/hikari_loc/status` 持续发且 `imu_age_s`/`lidar_age_s` 随掉线增长；恢复后回 GOOD。现有 `loc_state`/Marker 不回归。
- 6C：稳态 GOOD 下日志不再每帧刷屏；异常日志仍即时。
- 6D：`.service` 模板占位符可被打包渲染；装机后 `systemctl start` 能起、崩溃按策略重启。
- 真机/嵌入式实测由用户负责（绑核收益、掉线恢复体感）。
