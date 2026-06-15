#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zupt_detector_sim.py — ZUPT 标定工具 (2/2): 静止检测器在线仿真

用途:
  用候选参数在 Python 里**复刻 C++ StaticDetector 逻辑**(滑窗 std + 非对称迟滞 + warmup),
  订阅 IMU 实时打印「当前 static? 已持续多久 + 状态翻转」。
  机器人**站立<->慢走交替**时跑, 不重编 C++ 就能判断:
    - 阈值是否太松(慢走被误判静止)或太紧(静止时漏判);
    - park_enter_frames / park_exit_frames 迟滞是否合适。
  调到「静止稳定进 STATIC、一动立刻出 STATIC、慢走不误进」即为合格, 把参数填进 yaml。

用法:
  python3 zupt_detector_sim.py --ros-args \
    -p gyro_std_thres:=0.01 -p acc_std_thres:=0.15 \
    -p window_sec:=0.5 -p park_enter_frames:=10 -p park_exit_frames:=3

参数 (与 C++ fast_lio.zupt_* 一一对应):
  imu_topic          (string, /livox/imu)
  window_sec         (double, 0.5)   滑窗时长
  gyro_std_thres     (double, 0.01)  gyro 模 std 静止阈值 (rad/s) — 用 imu_static_stats 的建议值
  acc_std_thres      (double, 0.15)  acc 模 std 静止阈值 (m/s^2)
  park_enter_frames  (int,    10)    连续 raw_static 多少帧才进 STATIC (进慢)
  park_exit_frames   (int,    3)     连续 raw_moving 多少帧才出 STATIC (出快)
  warmup_frames      (int,    50)    启动前丢弃帧数 (等滑窗填满/IMU 稳定)
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ZuptDetectorSim(Node):
    def __init__(self):
        super().__init__("zupt_detector_sim")
        self.imu_topic = self.declare_parameter("imu_topic", "/livox/imu").value
        self.window_sec = float(self.declare_parameter("window_sec", 0.5).value)
        self.gyro_thres = float(self.declare_parameter("gyro_std_thres", 0.01).value)
        self.acc_thres = float(self.declare_parameter("acc_std_thres", 0.15).value)
        self.enter_frames = int(self.declare_parameter("park_enter_frames", 10).value)
        self.exit_frames = int(self.declare_parameter("park_exit_frames", 3).value)
        self.warmup_frames = int(self.declare_parameter("warmup_frames", 50).value)

        self.buf = deque()        # (t, gx..gz, ax..az)
        self.use_wall = False
        self.n_seen = 0
        self.enter_cnt = 0
        self.exit_cnt = 0
        self.is_static = False
        self.state_since_t = None
        self.last_status_t = 0.0
        # 统计
        self.transitions = 0
        self.static_total = 0.0
        self.moving_total = 0.0
        self.last_t = None

        self.sub = self.create_subscription(Imu, self.imu_topic, self.on_imu, 200)
        self.get_logger().info(
            "zupt_detector_sim 已启动 | topic=%s window=%.2fs\n"
            "  阈值 gyro_std<%.5f & acc_std<%.4f | 迟滞 enter=%d exit=%d | warmup=%d\n"
            "  请站立<->慢走交替, 观察 STATIC/MOVING 翻转是否合理"
            % (self.imu_topic, self.window_sec, self.gyro_thres, self.acc_thres,
               self.enter_frames, self.exit_frames, self.warmup_frames)
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
        if self.last_t is not None:
            dt = t - self.last_t
            if dt > 0 and dt < 1.0:
                if self.is_static:
                    self.static_total += dt
                else:
                    self.moving_total += dt
        self.last_t = t

        g = msg.angular_velocity
        a = msg.linear_acceleration
        self.buf.append((t, g.x, g.y, g.z, a.x, a.y, a.z))
        while self.buf and (t - self.buf[0][0]) > self.window_sec:
            self.buf.popleft()

        self.n_seen += 1
        if self.n_seen <= self.warmup_frames or len(self.buf) < 3:
            return

        arr = np.array(self.buf, dtype=float)
        gyro_norm_std = np.linalg.norm(arr[:, 1:4], axis=1).std()
        acc_norm_std = np.linalg.norm(arr[:, 4:7], axis=1).std()
        raw_static = (gyro_norm_std < self.gyro_thres) and (acc_norm_std < self.acc_thres)

        # 非对称迟滞
        if raw_static:
            self.enter_cnt += 1
            self.exit_cnt = 0
            if not self.is_static and self.enter_cnt >= self.enter_frames:
                self.flip(True, t)
        else:
            self.exit_cnt += 1
            self.enter_cnt = 0
            if self.is_static and self.exit_cnt >= self.exit_frames:
                self.flip(False, t)

        # 实时状态行 (5Hz)
        if (t - self.last_status_t) >= 0.2:
            self.last_status_t = t
            held = (t - self.state_since_t) if self.state_since_t else 0.0
            print(
                "\r[%-7s %5.1fs] raw=%-7s gyro_std=%.5f acc_std=%.4f "
                "(enter %d/%d, exit %d/%d)   "
                % (
                    "STATIC" if self.is_static else "MOVING", held,
                    "static" if raw_static else "moving",
                    gyro_norm_std, acc_norm_std,
                    self.enter_cnt, self.enter_frames,
                    self.exit_cnt, self.exit_frames,
                ),
                end="", flush=True,
            )

    def flip(self, to_static, t):
        held = (t - self.state_since_t) if self.state_since_t else 0.0
        if self.state_since_t is not None:
            print()  # 结束 \r 行
            self.get_logger().info(
                ">>> 翻转: %s -> %s (前态持续 %.2fs)"
                % ("STATIC" if self.is_static else "MOVING",
                   "STATIC" if to_static else "MOVING", held)
            )
        self.is_static = to_static
        self.state_since_t = t
        self.transitions += 1

    def print_summary(self):
        print("\n" + "=" * 60)
        print("ZUPT 检测器仿真 汇总")
        print("=" * 60)
        print("参数: gyro_std<%.5f & acc_std<%.4f, enter=%d exit=%d window=%.2fs"
              % (self.gyro_thres, self.acc_thres,
                 self.enter_frames, self.exit_frames, self.window_sec))
        print("状态翻转次数=%d" % self.transitions)
        print("累计 STATIC=%.1fs  MOVING=%.1fs" % (self.static_total, self.moving_total))
        print("-" * 60)
        print("判读: 静止时应稳定停在 STATIC 不抖; 一开始动应在 ~exit*帧间隔内退出;")
        print("      慢走若误进 STATIC -> 调大 thres 或 enter_frames。参数合适即填入 yaml。")
        print("=" * 60)


def main():
    rclpy.init()
    node = ZuptDetectorSim()
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
