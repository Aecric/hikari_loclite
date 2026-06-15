#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_static_stats.py — ZUPT 标定工具 (1/2): 静止噪声地板测量

用途:
  机器人**完全静止**时跑本节点, 测量滑窗内 gyro/acc 的标准差 (std).
  这个 std 就是 ZUPT 静止检测的噪声地板 —— 静止阈值要设在它之上留余量。

输出:
  - 实时 (默认 2Hz): 当前滑窗的 gyro/acc 每轴 std + 向量模 std。
  - 退出时 (Ctrl-C): 全程 min/max/mean 汇总 + 建议阈值 (= 全程 max * margin)。

用法:
  source /opt/ros/jazzy/setup.bash   # 或 humble
  python3 imu_static_stats.py
  # 改 topic / 窗口:
  python3 imu_static_stats.py --ros-args -p imu_topic:=/livox/imu -p window_sec:=0.5

参数:
  imu_topic   (string, 默认 /livox/imu)  IMU 话题
  window_sec  (double, 默认 0.5)         滑窗时长 (秒)
  print_hz    (double, 默认 2.0)         实时打印频率
  margin      (double, 默认 1.5)         建议阈值 = 全程 max std * margin
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ImuStaticStats(Node):
    def __init__(self):
        super().__init__("imu_static_stats")
        self.imu_topic = self.declare_parameter("imu_topic", "/livox/imu").value
        self.window_sec = float(self.declare_parameter("window_sec", 0.5).value)
        self.print_hz = float(self.declare_parameter("print_hz", 2.0).value)
        self.margin = float(self.declare_parameter("margin", 1.5).value)

        # 滑窗: (t, gx,gy,gz, ax,ay,az)
        self.buf = deque()
        self.last_print_t = 0.0
        self.use_wall = False  # header stamp 为 0 时退化到墙钟

        # 全程统计 (对"滑窗 std"再求 min/max/累加)
        self.n_windows = 0
        self.gyro_axis_max = np.zeros(3)
        self.acc_axis_max = np.zeros(3)
        self.gyro_norm_max = 0.0
        self.acc_norm_max = 0.0
        self.gyro_norm_sum = 0.0
        self.acc_norm_sum = 0.0
        self.gyro_norm_min = math.inf
        self.acc_norm_min = math.inf
        self.sample_count = 0
        self.first_t = None
        self.last_t = None

        self.sub = self.create_subscription(Imu, self.imu_topic, self.on_imu, 200)
        self.get_logger().info(
            "imu_static_stats 已启动 | topic=%s window=%.2fs | 请保持机器人静止"
            % (self.imu_topic, self.window_sec)
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
        if self.first_t is None:
            self.first_t = t
        self.last_t = t
        g = msg.angular_velocity
        a = msg.linear_acceleration
        self.buf.append((t, g.x, g.y, g.z, a.x, a.y, a.z))
        self.sample_count += 1

        # 丢弃窗外样本
        while self.buf and (t - self.buf[0][0]) > self.window_sec:
            self.buf.popleft()
        if len(self.buf) < 3:
            return

        arr = np.array(self.buf, dtype=float)
        gyro = arr[:, 1:4]
        acc = arr[:, 4:7]
        gyro_std = gyro.std(axis=0)                       # 每轴 std
        acc_std = acc.std(axis=0)
        gyro_norm_std = np.linalg.norm(gyro, axis=1).std()  # 向量模的 std
        acc_norm_std = np.linalg.norm(acc, axis=1).std()

        # 累计全程统计
        self.n_windows += 1
        self.gyro_axis_max = np.maximum(self.gyro_axis_max, gyro_std)
        self.acc_axis_max = np.maximum(self.acc_axis_max, acc_std)
        self.gyro_norm_max = max(self.gyro_norm_max, gyro_norm_std)
        self.acc_norm_max = max(self.acc_norm_max, acc_norm_std)
        self.gyro_norm_min = min(self.gyro_norm_min, gyro_norm_std)
        self.acc_norm_min = min(self.acc_norm_min, acc_norm_std)
        self.gyro_norm_sum += gyro_norm_std
        self.acc_norm_sum += acc_norm_std

        # 实时打印
        if (t - self.last_print_t) >= (1.0 / max(self.print_hz, 0.1)):
            self.last_print_t = t
            self.get_logger().info(
                "win=%d | gyro_std[xyz]=%.5f %.5f %.5f norm=%.5f rad/s | "
                "acc_std[xyz]=%.4f %.4f %.4f norm=%.4f m/s^2"
                % (
                    len(self.buf),
                    gyro_std[0], gyro_std[1], gyro_std[2], gyro_norm_std,
                    acc_std[0], acc_std[1], acc_std[2], acc_norm_std,
                )
            )

    def print_summary(self):
        if self.n_windows == 0:
            self.get_logger().warn("没有收到足够 IMU 数据, 无法汇总。检查 topic 是否正确。")
            return
        dur = 0.0
        if self.first_t is not None and self.last_t is not None:
            dur = self.last_t - self.first_t
        rate = self.sample_count / dur if dur > 0 else float("nan")
        gyro_norm_mean = self.gyro_norm_sum / self.n_windows
        acc_norm_mean = self.acc_norm_sum / self.n_windows
        print("\n" + "=" * 64)
        print("ZUPT 静止噪声地板 汇总 (假设全程静止)")
        print("=" * 64)
        print("样本数=%d  时长=%.1fs  估计 IMU 速率=%.1f Hz  滑窗=%.2fs"
              % (self.sample_count, dur, rate, self.window_sec))
        print("-" * 64)
        print("gyro 模 std (rad/s):  min=%.6f  mean=%.6f  MAX=%.6f"
              % (self.gyro_norm_min, gyro_norm_mean, self.gyro_norm_max))
        print("gyro 每轴 std MAX:    x=%.6f y=%.6f z=%.6f"
              % (self.gyro_axis_max[0], self.gyro_axis_max[1], self.gyro_axis_max[2]))
        print("acc  模 std (m/s^2):  min=%.5f  mean=%.5f  MAX=%.5f"
              % (self.acc_norm_min, acc_norm_mean, self.acc_norm_max))
        print("acc  每轴 std MAX:    x=%.5f y=%.5f z=%.5f"
              % (self.acc_axis_max[0], self.acc_axis_max[1], self.acc_axis_max[2]))
        print("-" * 64)
        print("建议静止阈值 (= 全程 MAX * margin=%.2f, 取向量模口径):" % self.margin)
        print("  static_gyro_std_thres: %.6f" % (self.gyro_norm_max * self.margin))
        print("  static_acc_std_thres:  %.5f" % (self.acc_norm_max * self.margin))
        print("  static_window:         %.2f   # 秒, 与本次测量窗口一致" % self.window_sec)
        print("=" * 64)
        print("下一步: 用 zupt_detector_sim.py 带上述阈值, 站立<->慢走交替验证是否误触/漏触。")


def main():
    rclpy.init()
    node = ImuStaticStats()
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
