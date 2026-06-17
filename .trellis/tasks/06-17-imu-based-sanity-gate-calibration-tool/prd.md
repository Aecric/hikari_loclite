# IMU/odom sanity_gate 阈值标定工具

## Goal

给上一个任务实现的 `sanity_gate.{max_speed_mps, max_accel_mps2}` 提供经验标定工具：跑真机做受控运动（正常最大速度/最急加减速），独立 rclpy 节点采集数据并推荐阈值，使门控只拦"物理不可能"的发散值、不误伤真实运动。

## Decisions (user-confirmed 2026-06-17)

* **速度来源** = 订阅定位系统 `/hikari_loc/odom`，**差分位置**得速度（实测 odom 只填 `pose.pose`、`twist` 为空，`loclite_node.cpp:997/1046`，故不能读 twist）。
* **加速度来源** = IMU `/livox/imu` `linear_acceleration`，启动静止窗估重力向量后逐样本去重力，取 3D 模长。
* **阈值推导** = p99.5 百分位 × 安全系数（默认 1.5），同时打印峰值/分布与可直接粘贴的 yaml 片段。
* **形态** = 独立单文件 rclpy 节点，放 `scripts/sanity_gate_calib/`，不进 colcon，`python3` 直接跑（照 `scripts/zupt_calib/` 先例）。

## Requirements

* 订阅 IMU + odom（话题、静止窗、百分位、margin 全部 `--ros-args -p` 可配，默认对齐 `config/loclite_livox.yaml`）。
* IMU 加速度去重力：首 `static_calib_sec` 秒静止估 `g_vec=mean(accel)`，方差过大则告警（未真静止）。
* odom 速度：差分相邻 `pose.position`，跳过 `dt<=0` 或 `dt>max_gap`（防陈旧位姿假峰）。
* 交叉参考：另报 odom 差分速度再差分得的 Δv/dt（门控内部实际用的量），供与 IMU 加速度对照。
* 实时打印（print_hz）当前/峰值；Ctrl-C 汇总 min/mean/p50/p95/p99/p99.5/max + 推荐阈值 + yaml 片段。
* 时间用 header.stamp，stamp 为 0 退回节点时钟；IMU 用 sensor_data QoS（best-effort）兼容驱动。

## Acceptance Criteria

* [ ] `python3 scripts/sanity_gate_calib/sanity_gate_calib.py` 能起、订阅到 IMU/odom 并实时打印。
* [ ] Ctrl-C 输出三组统计（speed / accel_imu / accel_odom）+ 推荐 `max_speed_mps`/`max_accel_mps2` + yaml 片段。
* [ ] 话题/参数可经 `--ros-args -p` 覆盖；默认值与 yaml 一致。
* [ ] 静止窗非静止时有告警。
* [ ] 配 README（用途/流程/参数），风格照 `scripts/zupt_calib/README.md`。

## Out of Scope

* 不进 colcon 构建 / 不装 `ros2 run` 入口（独立脚本）。
* 不改动 C++ 节点（含不补 odom twist —— 仅工具侧差分；是否补 twist 另议）。
* 不做自动写回 yaml（只打印片段，人工粘贴）。

## Technical Notes

* odom twist 为空是实测结论；若日后 C++ 补了 twist，可加 `--use_twist` 直接读，省差分。
* 域差异：门控 accel = Δ(ESKF vel)/dt，IMU accel 是比力去重力；正常运动二者近似，IMU 更噪 → 阈值偏保守（少误 LOST），p99.5 抑制振动毛刺。已在工具输出里注明 odom-Δv/dt 交叉量。
* 相关：[[base-to-lidar-yaw-initialpose-only]] 无关；本工具纯读话题。
