# ZUPT 零速更新 — 静止检测 + ESKF 速度伪观测

## Goal

治理 hikari_loclite 在走廊里**静止时沿轴向蠕动**（用户 2026-06-15 现场观察：停车后 hikari 仍往前走一小段；lightning 用 ZUPT 解决）。

物理本质：走廊沿轴方向**几何不可观**（同 Phase 2 退化检测命中的 rotation/translation 退化方向）。静止时 IMU 残余 acc bias + 重力对齐误差漏进一个小的非零 `vel_`，积分成位置蠕动；固定图在走廊沿轴约束不住（eff 点崩塌），scan-match 钉不住沿轴位置。

**关键：NDT（已调到 3Hz/gain0.2）在该方向同样退化，拉不回沿轴蠕动。** ZUPT 用零速**运动模型**而非几何，正好在几何退化处有效——与 NDT 正交，不是替代。

## 实测事实（已核查，不要重新调研）

### hikari 现状（无任何 ZUPT/静止检测）
- ESKF 是 **12 维**：`pos(3) + vel(3) + rot(3) + bg(3)`；`ba` 与 `grav` **冻结**（`nav_state.h:125 Getba()` 恒返回 Zero，`grav_` 不更新）。
- `vel_` 是被估计状态：`nav_state.h:138`，`GetVel()/SetVel()` 在 `nav_state.h:127-128`，索引 `kVelIdx`。→ **对速度子块做零速伪观测结构上直接可行**（H 选 3 行速度，z = −vel_，小 R）。
- `grep zupt|static_det|parked` 全空。仅 `nav_state.h:134` 一个 `bool is_parking_ = false;` **死字段，零引用**（可复用）。
- LIO 入口：`src/lio/fast_lio_fixed_map.cpp`（`RunOnce`）+ `include/hikari_loclite/lio/eskf.hpp`（`Predict` 在 `:77`，pose 观测 `ObserveSE3`/`UpdateIterated` 一类）+ `imu_processing.hpp`。

### lightning 参照实现（不可 drop-in）
- lightning 的 ZUPT **只在 `src/lightning-lm/src/core/lio/point_lio/point_lio_frontend.{h,cc}`**，不在 fast_lio_aa/eskf 里。hikari 用移植版 AA-FasterLIO eskf → **必须在 hikari 的 eskf 上重写**，不能抽取。
- lightning 设计要点（参照，不照搬）：
  - `StaticDetector`：滑窗 gyro/acc std + **非对称迟滞**（进静止慢 `park_enter_frames≈10`、出静止快 `park_exit_frames≈3`）+ warmup。
  - 第一道门用 EKF 速度本身：`zupt_vel_gate_≈0.05` m/s（速度已经很小才允许 ZUPT），再叠 IMU std 双门防误触。
  - `ZuptOutput()`：vel 子块零速 Kalman 伪观测，`zupt_vel_cov≈1e-3`。
  - lightning 还有 `parking_freeze`（整 ESEKF 冻结）——**hikari 不做**：hikari 冻结了 ba/grav，整冻结收益小、风险大（误判把真实慢速运动钳死）。

## Requirements

### Phase 0 — 标定工具（先交付，用户先收数据）
两个独立 Python ROS2 节点（标定用，**不进 colcon 包构建**，放 `scripts/zupt_calib/`）：
1. **`imu_static_stats.py`** — 订阅 IMU，滑窗算 gyro/acc 每轴 + norm 的 std，实时打印 + 跑全程 min/max/mean。用户**静止站立**时跑 → 读出静止噪声地板，定 `gyro_std_thres` / `acc_std_thres`。
2. **`zupt_detector_sim.py`** — 用候选参数（window/enter/exit/thres）在 Python 里复刻 StaticDetector 逻辑，订阅 IMU 实时打印「当前 static? 已持续多少帧/秒 + 状态翻转」。用户**站立↔慢走**交替跑 → 不重编 C++ 就能验证迟滞 + 阈值是否误触/漏触，定 enter/exit frames。
- 两节点都参数化 IMU topic（默认 `common.imu_topic` 的值）、window 大小，用 `RCLCPP`/`rclpy`，纯标准库 + numpy。

### Phase 1 — C++ ZUPT 实现（拿到标定值后做）
1. **`StaticDetector`**（新小类，落 `include/hikari_loclite/lio/` 或并入 imu 处理）：滑窗 gyro/acc std + 非对称迟滞 + warmup，输出 `bool is_static`。
2. **ESKF 零速伪观测**：eskf 加 `ZuptUpdate()`——vel 子块 z=−vel_、H 选速度 3 行、R=`zupt_vel_cov`，标准 Kalman 更新后回写 `dx`。
3. **双门触发**：`StaticDetector.is_static && GetVel().norm() < zupt_vel_gate` 才触发。
4. **接线**：`fast_lio_fixed_map RunOnce` 在 pose 观测后调用；复用/置位 `is_parking_`。
5. **yaml knobs**（`fast_lio.zupt_*`，附中文注释 + 标定来源）：`zupt_enabled`、`zupt_vel_gate`、`zupt_vel_cov`、`static_gyro_std_thres`、`static_acc_std_thres`、`static_window`、`park_enter_frames`、`park_exit_frames`、`warmup_frames`。默认值用 Phase 0 标定结果。

### 约束
- 不新增依赖、不链 lightning.libs、不抽取 lightning 的 `point_lio_frontend.*`。
- **不做** parking_freeze 整滤波器冻结（理由见上）。
- 落点限于 `eskf.{hpp,cc}` + StaticDetector + `fast_lio_fixed_map` + yaml；**不碰 NDT / SC / 状态机**。
- 嵌入式预算：StaticDetector 仅滑窗统计，每帧 O(window)；ZUPT 更新仅 3 维观测。
- 误判风险必须配缓解：阈值保守 + 非对称迟滞 + 速度门第一道闸。

## 验收
- 用户自测（bag 回放 / 真机）：停车后沿轴蠕动消除，且慢速运动不被误冻结。AI 只负责编译通过。
