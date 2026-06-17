#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
humanoid_imu_spike_calib.py — 人形机器人 IMU 加速度脉冲阈值标定工具

用途:
  回放或运行包含: 静止 → 启动 → 行走 → 转弯 → 停止 的人形机器人数据集,
  计算 |acc|、|acc[k]-acc[k-1]|、|gyro| 的百分位统计,
  输出可直接粘贴到 YAML 的 imu_humanoid_* 阈值建议。

输出:
  - 实时 (2Hz): 滚动百分位摘要
  - 退出时 (Ctrl-C): 全程 + 分阶段百分位 + YAML 建议

用法:
  python3 scripts/imu_calib/humanoid_imu_spike_calib.py --ros-args -p imu_topic:=/livox/imu
  python3 scripts/imu_calib/humanoid_imu_spike_calib.py --ros-args -p imu_topic:=/livox/imu -p warmup_sec:=3.0 -p output_csv:=/tmp/imu_calib.csv

参数:
  imu_topic   (string, 默认 /livox/imu)  IMU 话题
  warmup_sec  (double, 默认 3.0)         启动丢弃时长 (秒)
  print_hz    (double, 默认 2.0)         实时打印频率
  output_csv  (string, 默认 "")          CSV 输出路径 (空=不输出)
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


PHASE_STATIC = "static"
PHASE_STARTUP = "static→moving"
PHASE_WALKING = "walking"
PHASE_STOPPING = "moving→static"
PHASE_UNKNOWN = "unknown"


class HumanoidImuSpikeCalib(Node):
    def __init__(self):
        super().__init__("humanoid_imu_spike_calib")
        self.imu_topic = self.declare_parameter("imu_topic", "/livox/imu").value
        self.warmup_sec = float(self.declare_parameter("warmup_sec", 3.0).value)
        self.print_hz = float(self.declare_parameter("print_hz", 2.0).value)
        self.output_csv = self.declare_parameter("output_csv", "").value

        self.use_wall = False
        self.start_time = None
        self.warmup_done = False
        self.last_print_t = 0.0

        self.prev_acc = None
        self.acc_norms = []
        self.acc_deltas = []
        self.gyro_norms = []
        self.timestamps = []

        self.phase_window = deque()
        self.phase_window_sec = 1.0
        self.current_phase = PHASE_UNKNOWN
        self.phases = []

        self.sub = self.create_subscription(Imu, self.imu_topic, self.on_imu, 200)
        self.get_logger().info(
            "humanoid_imu_spike_calib 已启动 | topic=%s warmup=%.1fs"
            % (self.imu_topic, self.warmup_sec)
        )

    def now_sec(self, msg):
        t = stamp_to_sec(msg.header.stamp)
        if t <= 0.0:
            self.use_wall = True
        if self.use_wall:
            return self.get_clock().now().nanoseconds * 1e-9
        return t

    def on_imu(self, msg: Imu):
        t = self.now_sec(msg)
        if self.start_time is None:
            self.start_time = t
        if (t - self.start_time) < self.warmup_sec:
            return
        if not self.warmup_done:
            self.warmup_done = True
            self.get_logger().info("warmup 完成, 开始采集...")

        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        acc_norm = np.linalg.norm(acc)
        gyro_norm = np.linalg.norm(gyro)

        acc_delta = np.linalg.norm(acc - self.prev_acc) if self.prev_acc is not None else 0.0
        self.prev_acc = acc

        self.acc_norms.append(acc_norm)
        self.acc_deltas.append(acc_delta)
        self.gyro_norms.append(gyro_norm)
        self.timestamps.append(t)

        self.phase_window.append((t, acc_norm, gyro_norm))
        while self.phase_window and (t - self.phase_window[0][0]) > self.phase_window_sec:
            self.phase_window.popleft()
        self.update_phase(t)

        if (t - self.last_print_t) >= (1.0 / max(self.print_hz, 0.1)):
            self.last_print_t = t
            self.print_live()

    def update_phase(self, t):
        if len(self.phase_window) < 10:
            return
        arr = np.array(self.phase_window)
        gyro_std = np.std(arr[:, 2])
        acc_std = np.std(arr[:, 1])

        if gyro_std < 0.015 and acc_std < 0.01:
            raw_class = PHASE_STATIC
        elif gyro_std > 0.05 or acc_std > 0.1:
            raw_class = PHASE_WALKING
        else:
            raw_class = "transition"

        if raw_class == PHASE_STATIC:
            new_phase = PHASE_STATIC
        elif raw_class == PHASE_WALKING:
            new_phase = PHASE_WALKING
        else:
            if self.current_phase in (PHASE_STATIC, PHASE_STARTUP, PHASE_UNKNOWN):
                new_phase = PHASE_STARTUP
            elif self.current_phase in (PHASE_WALKING, PHASE_STOPPING):
                new_phase = PHASE_STOPPING
            else:
                new_phase = PHASE_STARTUP

        if new_phase != self.current_phase:
            self.phases.append((t, new_phase))
            self.current_phase = new_phase

    def print_live(self):
        n = len(self.acc_norms)
        if n < 10:
            return
        recent_acc = self.acc_norms[-min(n, 200):]
        recent_delta = self.acc_deltas[-min(n, 200):]
        p99_acc = np.percentile(recent_acc, 99)
        p99_delta = np.percentile(recent_delta, 99)
        self.get_logger().info(
            "[%s] n=%d | acc| p99=%.2f delta| p99=%.2f"
            % (self.current_phase, n, p99_acc, p99_delta)
        )

    def print_summary(self):
        if len(self.acc_norms) < 10:
            self.get_logger().warn("没有收到足够 IMU 数据, 无法汇总。")
            return

        dur = self.timestamps[-1] - self.timestamps[0] if len(self.timestamps) > 1 else 0.0
        rate = len(self.timestamps) / dur if dur > 0 else float("nan")

        print("\n" + "=" * 64)
        print("Humanoid IMU Spike Calibration 汇总")
        print("=" * 64)
        print("样本数=%d  时长=%.1fs  估计 IMU 速率=%.1f Hz" % (len(self.acc_norms), dur, rate))
        print("-" * 64)

        self._print_percentiles("acc_norm", self.acc_norms)
        self._print_percentiles("acc_delta", self.acc_deltas)
        self._print_percentiles("gyro_norm", self.gyro_norms)

        phase_segments = self._segment_by_phase()
        if phase_segments:
            print("\n--- 分阶段统计 ---")
            for phase_name, data in phase_segments.items():
                if len(data["acc_norm"]) < 5:
                    continue
                print("\n[%s] n=%d" % (phase_name, len(data["acc_norm"])))
                self._print_percentiles("  acc_norm", data["acc_norm"], indent="  ")
                self._print_percentiles("  acc_delta", data["acc_deltas"], indent="  ")

        acc_n = np.array(self.acc_norms)
        acc_d = np.array(self.acc_deltas)
        print("\n" + "=" * 64)
        print("推荐 YAML 配置 (基于 p99.5)")
        print("=" * 64)
        print("fast_lio:")
        print("  imu_filter: true")
        print("  imu_humanoid_acc_norm_min: %.1f" % max(np.percentile(acc_n, 0.5), 2.0))
        print("  imu_humanoid_acc_norm_max: %.1f" % np.percentile(acc_n, 99.5))
        print("  imu_humanoid_acc_delta_max: %.1f" % np.percentile(acc_d, 99.5))
        print("  imu_humanoid_acc_clamp_norm_max: %.1f" % np.percentile(acc_n, 99.9))
        print("  imu_humanoid_acc_cov_scale_on_spike: 10.0")
        print("=" * 64)

        if self.output_csv:
            self._write_csv()

    def _print_percentiles(self, name, data, indent=""):
        arr = np.array(data)
        print("%s%s: p95=%.2f  p99=%.2f  p99.5=%.2f  p99.9=%.2f  min=%.2f  max=%.2f"
              % (indent, name,
                 np.percentile(arr, 95), np.percentile(arr, 99),
                 np.percentile(arr, 99.5), np.percentile(arr, 99.9),
                 np.min(arr), np.max(arr)))

    def _segment_by_phase(self):
        if not self.phases:
            return {}
        segments = {}
        ts = np.array(self.timestamps)
        an = np.array(self.acc_norms)
        ad = np.array(self.acc_deltas)

        boundaries = [self.timestamps[0]]
        for t, _ in self.phases:
            boundaries.append(t)
        boundaries.append(self.timestamps[-1])

        phase_names = [PHASE_UNKNOWN]
        for _, name in self.phases:
            phase_names.append(name)

        for i in range(len(boundaries) - 1):
            lo, hi = boundaries[i], boundaries[i + 1]
            mask = (ts >= lo) & (ts < hi)
            if mask.sum() < 5:
                continue
            pname = phase_names[i]
            if pname not in segments:
                segments[pname] = {"acc_norm": [], "acc_deltas": []}
            segments[pname]["acc_norm"].extend(an[mask].tolist())
            segments[pname]["acc_deltas"].extend(ad[mask].tolist())

        return segments

    def _write_csv(self):
        import csv
        path = self.output_csv
        try:
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "acc_norm", "acc_delta", "gyro_norm"])
                for i in range(len(self.timestamps)):
                    writer.writerow([self.timestamps[i], self.acc_norms[i],
                                     self.acc_deltas[i], self.gyro_norms[i]])
            print("CSV 已写入: %s" % path)
        except Exception as e:
            self.get_logger().warn("CSV 写入失败: %s" % e)


def main():
    rclpy.init()
    node = HumanoidImuSpikeCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
