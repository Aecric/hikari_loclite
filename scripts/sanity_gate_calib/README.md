# sanity_gate 阈值标定工具

独立 ROS2 (rclpy) 节点, 给 hikari_loclite 的 `sanity_gate.{max_speed_mps, max_accel_mps2}`
(物理运动合理性硬门控) 标定阈值。**不进 colcon 包构建**, 直接 `python3` 跑。
依赖: `rclpy` (ROS2 jazzy/humble) + `numpy`。

## 门控背景

`sanity_gate` 判定: 当 LIO 估计的本体速度 / 加速度超过机器人**物理上不可能**的值, 即认为
跟踪发散 → 立即 LOST + 冻结 TF + 重定位恢复。阈值必须设在「真实运动峰值」之上留余量,
既能拦住发散, 又不误伤正常运动。本工具就是测出这条线。

## 数据来源

- **加速度**: IMU (`/livox/imu`) `linear_acceleration`, 启动静止窗估重力向量后逐样本去重力, 取 3D 模长。
- **速度**: 定位 odom (`/hikari_loc/odom`)。优先读 `twist.twist.linear` 的模长——自 `b54fae6`
  (feat(loclite): publish odometry twist from ESKF) 起 odom 填了 ESKF 速度 (旋到 lidar 子系,
  模长即门控用的 `vel_.norm()`, 最准且与门控同域)。`velocity_source=auto` 时若 twist 全程为 0
  (旧构建未填 twist) 自动回退**差分 `pose.position`**。

## 流程

```bash
source /opt/ros/jazzy/setup.bash        # 或 humble

# 1) 起定位 (run_loclite_online / loclite.launch.py), 确认有 /hikari_loc/odom
# 2) 让机器人**静止 ~2s** (估重力), 再开始受控运动:
#    正常最大速度直线 + 最急加速 + 最急刹车 + 转向, 覆盖本体真实运动极限
python3 scripts/sanity_gate_calib/sanity_gate_calib.py

# 跑够后 Ctrl-C, 看汇总里的「推荐阈值」与 yaml 片段, 粘进 config/loclite_livox.yaml
```

改话题 / 参数:
```bash
python3 scripts/sanity_gate_calib/sanity_gate_calib.py --ros-args \
  -p imu_topic:=/livox/imu -p odom_topic:=/hikari_loc/odom \
  -p static_calib_sec:=2.0 -p percentile:=99.5 -p margin:=1.5
```

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `imu_topic` | `/livox/imu` | IMU 话题 |
| `odom_topic` | `/hikari_loc/odom` | 定位 odom 话题 |
| `velocity_source` | `auto` | `auto`\|`twist`\|`pose_diff`；auto: twist 有运动则用 twist, 否则回退 pose_diff |
| `static_calib_sec` | `2.0` | 启动静止窗时长 (秒), 估重力向量 |
| `static_acc_std_warn` | `0.3` | 静止窗 acc std 超此值告警 (未真静止) |
| `max_odom_gap_sec` | `0.5` | odom 相邻帧间隔超此值跳过 (防陈旧位姿假峰) |
| `motion_eps_mps` | `0.05` | auto 判定 twist 是否真在动的速度下限 |
| `percentile` | `99.5` | 推荐阈值取的百分位 |
| `margin` | `1.5` | 推荐阈值 = 百分位值 × margin |
| `print_hz` | `2.0` | 实时打印频率 |

## 输出说明

退出时打印各组分布 (min/mean/p50/p95/p99/p99.5/max):
- `speed(twist)` → ESKF 速度 (与门控 `vel_.norm()` 同域), auto 默认据此推荐 `max_speed_mps`
- `speed(posediff)` → 差分位置的速度, 旧构建无 twist 时的回退源 / 对照
- `accel(imu)` → 推荐 `max_accel_mps2`
- `accel(twistΔv)` → **交叉参考**: 门控内部实际用 `Δv/dt`(对 ESKF 速度差分), 与此量同域。
  IMU 比力去重力通常比 `Δv/dt` 偏大且更噪, 所以基于 IMU 的加速度阈值偏保守 (少误 LOST),
  p99.5 用于抑制振动毛刺。

## 注意

- 必须先静止再运动: 重力靠首段静止窗估计, 没静止好会告警且阈值不准。
- `velocity_source=auto` 会在 twist 全程为 0 (旧构建未填 twist) 时自动回退 `pose_diff` 并提示。
- 地面机器人假设倾斜小; 跑陡坡时重力会漏进水平分量, 标定请在平地做。
