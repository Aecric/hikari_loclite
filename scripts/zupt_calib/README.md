# ZUPT 标定工具

两个独立 ROS2 (rclpy) 节点, 给 hikari_loclite 的 ZUPT 静止检测标定阈值。**不进 colcon 包构建**, 直接 `python3` 跑。依赖: `rclpy` (ROS2 jazzy/humble) + `numpy`。

输入话题: `sensor_msgs/msg/Imu`, 默认 `/livox/imu` (与 `config/loclite_livox.yaml` 的 `common.imu_topic` 一致)。

## 流程

### 1. 测静止噪声地板 → 定阈值
机器人**完全静止**站立, 跑:
```bash
source /opt/ros/jazzy/setup.bash
python3 imu_static_stats.py
# 静置 ~30s 后 Ctrl-C, 看汇总里的 "建议静止阈值"
```
拿到 `static_gyro_std_thres` / `static_acc_std_thres` / `static_window`。

### 2. 验证迟滞不误触 → 定 enter/exit
带上一步阈值, **站立↔慢走交替**跑:
```bash
python3 zupt_detector_sim.py --ros-args \
  -p gyro_std_thres:=<上一步值> -p acc_std_thres:=<上一步值> \
  -p window_sec:=0.5 -p park_enter_frames:=10 -p park_exit_frames:=3
```
观察实时状态行 + 翻转日志:
- 静止时应稳定 `STATIC` 不抖;
- 一开始动应在 ~`exit_frames` 个采样内退出 `STATIC`;
- 慢走若误进 `STATIC` → 调大 `gyro_std_thres`/`acc_std_thres` 或 `park_enter_frames`。

合格的一组参数即为 Phase 1 C++ `fast_lio.zupt_*` 的 yaml 默认值。

## 也能跑 bag
两节点都用 IMU header.stamp 做滑窗 (stamp 为 0 时退化到墙钟), bag 回放同样可用:
```bash
ros2 bag play <bag> &
python3 zupt_detector_sim.py --ros-args -p ...
```

## 参数

`imu_static_stats.py`: `imu_topic` `window_sec` `print_hz` `margin`
`zupt_detector_sim.py`: `imu_topic` `window_sec` `gyro_std_thres` `acc_std_thres` `park_enter_frames` `park_exit_frames` `warmup_frames`
