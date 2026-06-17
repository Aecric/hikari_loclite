#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sanity_gate_calib.py — sanity_gate 物理运动合理性门控阈值标定工具

用途:
  给 hikari_loclite 的 sanity_gate.{max_speed_mps, max_accel_mps2} 标定阈值。
  在真机上做**受控运动**(正常最大速度 + 最急加/减速 + 转向), 本节点采集:
    - 速度:  订阅定位 odom (默认 /hikari_loc/odom)。优先读 twist.linear (ESKF 速度,
             模长 = 门控用的 vel_.norm(), 最准最同域); 旧构建 odom 无 twist 时自动回退
             差分 pose.position。
    - 加速度: 订阅 IMU (默认 /livox/imu) linear_acceleration, 启动静止窗估重力向量后逐样本去重力。
  跑完 Ctrl-C, 取汇总里的「推荐阈值」(= p99.5 百分位 * margin), 填进 yaml。

  门控语义: 机器人本体物理上不可能超过的速度/加速度 → 判定 LIO 跟踪发散 → LOST。
  阈值要设在「真实运动 p99.5」之上留余量, 既拦发散又不误伤正常运动。

输出:
  - 实时 (默认 2Hz): 当前 speed / accel(imu) + 全程峰值 + 样本计数。
  - 退出 (Ctrl-C): speed(twist/posediff) / accel_imu / accel(twist Δv/dt) 各组
                   min/mean/p50/p95/p99/p99.5/max + 推荐 max_speed_mps / max_accel_mps2
                   + 可直接粘贴的 yaml 片段。

用法:
  source /opt/ros/jazzy/setup.bash        # 或 humble
  # 1) 先让机器人**静止** ~2s (估重力), 再开始受控运动
  python3 sanity_gate_calib.py
  # 改话题 / 参数:
  python3 sanity_gate_calib.py --ros-args \
      -p imu_topic:=/livox/imu -p odom_topic:=/hikari_loc/odom \
      -p velocity_source:=auto -p static_calib_sec:=2.0 -p percentile:=99.5 -p margin:=1.5

参数:
  imu_topic        (string, /livox/imu)        IMU 话题 (sensor_msgs/Imu)
  odom_topic       (string, /hikari_loc/odom)  定位 odom 话题 (nav_msgs/Odometry)
  velocity_source  (string, auto)              auto | twist | pose_diff
                                                 auto: twist 有运动则用 twist, 否则回退 pose_diff
  static_calib_sec (double, 2.0)               启动静止窗时长 (秒), 估重力向量
  static_acc_std_warn (double, 0.3)            静止窗 acc std 超此值告警 (m/s^2, 未真静止)
  max_odom_gap_sec (double, 0.5)               odom 相邻帧间隔超此值则跳过 (防陈旧位姿假峰)
  motion_eps_mps   (double, 0.05)              判定「twist 是否真在动」的速度下限 (auto 用)
  percentile       (double, 99.5)              推荐阈值取的百分位
  margin           (double, 1.5)               推荐阈值 = 百分位值 * margin
  print_hz         (double, 2.0)               实时打印频率

依赖: rclpy (ROS2 jazzy/humble) + numpy。不进 colcon 包构建, 直接 python3 跑。
"""

import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def summarize(name, samples, unit):
    """打印一组样本的分布统计; 返回 numpy 数组 (空则 None)。"""
    if len(samples) == 0:
        print(f"  {name:<14}: (无样本)")
        return None
    a = np.asarray(samples, dtype=float)
    print(f"  {name:<14}: n={a.size:<6} "
          f"min={a.min():.3f} mean={a.mean():.3f} p50={np.percentile(a, 50):.3f} "
          f"p95={np.percentile(a, 95):.3f} p99={np.percentile(a, 99):.3f} "
          f"p99.5={np.percentile(a, 99.5):.3f} max={a.max():.3f}  [{unit}]")
    return a


class SanityGateCalib(Node):
    def __init__(self):
        super().__init__('sanity_gate_calib')

        self.imu_topic = self.declare_parameter('imu_topic', '/livox/imu').value
        self.odom_topic = self.declare_parameter('odom_topic', '/hikari_loc/odom').value
        self.velocity_source = str(self.declare_parameter('velocity_source', 'auto').value)
        self.static_calib_sec = float(self.declare_parameter('static_calib_sec', 2.0).value)
        self.static_acc_std_warn = float(self.declare_parameter('static_acc_std_warn', 0.3).value)
        self.max_odom_gap_sec = float(self.declare_parameter('max_odom_gap_sec', 0.5).value)
        self.motion_eps_mps = float(self.declare_parameter('motion_eps_mps', 0.05).value)
        self.percentile = float(self.declare_parameter('percentile', 99.5).value)
        self.margin = float(self.declare_parameter('margin', 1.5).value)
        self.print_hz = float(self.declare_parameter('print_hz', 2.0).value)

        # --- 重力估计 (启动静止窗) ---
        self.g_vec = None                 # 估出的重力向量 (m/s^2, IMU 系)
        self.grav_samples = []            # 静止窗内 accel 向量
        self.grav_window_start = None     # 静止窗起始 stamp

        # --- 采样缓冲 ---
        self.accel_imu = []               # IMU 去重力后线加速度模长 (m/s^2)
        self.speed_twist = []             # odom twist.linear 模长 (m/s) —— ESKF 速度, 与门控同域
        self.speed_posediff = []          # 差分 pose.position 速度 (m/s) —— 回退/对照
        self.accel_twist = []             # twist 速度再差分 |Δv|/dt (m/s^2) —— 门控内部实际量
        self.last_pose = None             # (t, pos np.array)
        self.last_twist_speed = None      # (t, v scalar)

        # --- 实时峰值 ---
        self.peak_speed = 0.0
        self.peak_accel_imu = 0.0
        self.cur_speed = 0.0
        self.cur_accel_imu = 0.0

        self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.print_timer = self.create_timer(1.0 / max(self.print_hz, 0.1), self.on_print)

        print('=' * 78)
        print('sanity_gate_calib — 阈值标定 (Ctrl-C 结束并打印汇总)')
        print(f'  IMU : {self.imu_topic}   (加速度, 去重力)')
        print(f'  odom: {self.odom_topic}   (速度, 来源={self.velocity_source})')
        print(f'  推荐阈值 = p{self.percentile} * margin({self.margin})')
        print(f'  >> 请先让机器人静止 {self.static_calib_sec:.1f}s 估重力, 再开始受控运动 <<')
        print('=' * 78)

    # ---------- IMU: 加速度 (去重力) ----------
    def on_imu(self, msg: Imu):
        t = stamp_to_sec(msg.header.stamp)
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        a = np.array([msg.linear_acceleration.x,
                      msg.linear_acceleration.y,
                      msg.linear_acceleration.z], dtype=float)

        # 静止窗估重力
        if self.g_vec is None:
            if self.grav_window_start is None:
                self.grav_window_start = t
            self.grav_samples.append(a)
            if t - self.grav_window_start >= self.static_calib_sec and len(self.grav_samples) >= 10:
                arr = np.asarray(self.grav_samples)
                self.g_vec = arr.mean(axis=0)
                acc_std = float(np.linalg.norm(arr.std(axis=0)))
                print(f'[重力估计] g_vec=[{self.g_vec[0]:.3f}, {self.g_vec[1]:.3f}, '
                      f'{self.g_vec[2]:.3f}] |g|={np.linalg.norm(self.g_vec):.3f} m/s^2, '
                      f'静止窗 acc_std={acc_std:.3f} (n={arr.shape[0]})')
                if acc_std > self.static_acc_std_warn:
                    print(f'[告警] 静止窗 acc_std={acc_std:.3f} > {self.static_acc_std_warn:.3f}, '
                          f'机器人可能未真静止 → 重力估计偏差, 建议重跑')
                print('-' * 78)
            return

        lin = a - self.g_vec
        mag = float(np.linalg.norm(lin))
        self.accel_imu.append(mag)
        self.cur_accel_imu = mag
        if mag > self.peak_accel_imu:
            self.peak_accel_imu = mag

    # ---------- odom: 速度 (优先 twist, 回退差分位置) ----------
    def on_odom(self, msg: Odometry):
        t = stamp_to_sec(msg.header.stamp)
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        # twist.linear 模长 = ESKF 速度 (b54fae6 起 odom 填 twist; 旧构建恒 0)
        tw = msg.twist.twist.linear
        v_twist = math.sqrt(tw.x * tw.x + tw.y * tw.y + tw.z * tw.z)
        self.speed_twist.append(v_twist)
        # twist 速度再差分 → Δv/dt (门控内部实际计算的量)
        if self.last_twist_speed is not None:
            tv0, v0 = self.last_twist_speed
            dtv = t - tv0
            if 0.0 < dtv <= self.max_odom_gap_sec:
                self.accel_twist.append(abs(v_twist - v0) / dtv)
        self.last_twist_speed = (t, v_twist)

        # 差分 pose.position (回退源 / 对照)
        p = msg.pose.pose.position
        pos = np.array([p.x, p.y, p.z], dtype=float)
        v_pd = None
        if self.last_pose is not None:
            t0, pos0 = self.last_pose
            dt = t - t0
            if 0.0 < dt <= self.max_odom_gap_sec:
                v_pd = float(np.linalg.norm(pos - pos0)) / dt
                self.speed_posediff.append(v_pd)
        self.last_pose = (t, pos)

        # 实时显示用的当前速度: 按来源选 (auto 先用 twist)
        cur = v_twist if self.velocity_source != 'pose_diff' else v_pd
        if cur is not None:
            self.cur_speed = cur
            if cur > self.peak_speed:
                self.peak_speed = cur

    # ---------- 实时打印 ----------
    def on_print(self):
        if self.g_vec is None:
            print(f'\r[估重力中] 静止采样 {len(self.grav_samples)} 帧 ...', end='', flush=True)
            return
        print(f'\rspeed={self.cur_speed:5.2f} m/s (peak {self.peak_speed:5.2f}) | '
              f'accel={self.cur_accel_imu:5.2f} m/s^2 (peak {self.peak_accel_imu:5.2f}) | '
              f'n_spd={len(self.speed_twist)} n_acc={len(self.accel_imu)}   ',
              end='', flush=True)

    # ---------- 选定速度源 ----------
    def _choose_speed(self):
        """返回 (源名, 样本数组)。auto: twist 有运动则用 twist, 否则回退 pose_diff。"""
        tw = np.asarray(self.speed_twist, dtype=float) if self.speed_twist else None
        pd = np.asarray(self.speed_posediff, dtype=float) if self.speed_posediff else None
        if self.velocity_source == 'twist':
            return 'twist', tw
        if self.velocity_source == 'pose_diff':
            return 'pose_diff', pd
        # auto
        tw_moves = tw is not None and tw.max() > self.motion_eps_mps
        if tw_moves:
            return 'twist', tw
        pd_moves = pd is not None and pd.max() > self.motion_eps_mps
        if pd_moves:
            print(f'[提示] twist 全程 < {self.motion_eps_mps} m/s (旧构建无 twist?) → 回退 pose_diff')
            return 'pose_diff', pd
        return 'twist', tw  # 都没动, 随便返回 twist 让上层判空

    # ---------- 退出汇总 ----------
    def report(self):
        print('\n' + '=' * 78)
        print('汇总 (受控运动全程统计)')
        print('-' * 78)
        summarize('speed(twist)', self.speed_twist, 'm/s')
        summarize('speed(posediff)', self.speed_posediff, 'm/s')
        acc_i = summarize('accel(imu)', self.accel_imu, 'm/s^2')
        acc_t = summarize('accel(twistΔv)', self.accel_twist, 'm/s^2')

        src, spd = self._choose_speed()
        if spd is None or len(spd) == 0 or acc_i is None:
            print('-' * 78)
            print('[警告] 速度或加速度无样本, 无法推荐阈值。检查话题是否有数据 / 是否做了运动。')
            print('=' * 78)
            return

        p = self.percentile
        rec_speed = float(np.percentile(spd, p)) * self.margin
        rec_accel = float(np.percentile(acc_i, p)) * self.margin

        print('-' * 78)
        print(f'推荐阈值 (p{p} * margin{self.margin}, 速度源={src}):')
        print(f'  max_speed_mps  = {rec_speed:.2f}   '
              f'(p{p} speed={np.percentile(spd, p):.3f}, peak={spd.max():.3f})')
        print(f'  max_accel_mps2 = {rec_accel:.2f}   '
              f'(p{p} accel_imu={np.percentile(acc_i, p):.3f}, peak={acc_i.max():.3f})')
        if acc_t is not None:
            print(f'  [交叉参考] 门控内部用 Δv/dt(twist) p{p}={np.percentile(acc_t, p):.3f}, '
                  f'peak={acc_t.max():.3f} m/s^2 — 与门控同域; IMU 比力偏大且更噪, 故 IMU 阈值偏保守')
        print('-' * 78)
        print('可粘贴到 config/loclite_livox.yaml:')
        print('sanity_gate:')
        print('  enabled: true')
        print(f'  max_speed_mps: {rec_speed:.2f}')
        print(f'  max_accel_mps2: {rec_accel:.2f}')
        print('=' * 78)


def main():
    rclpy.init()
    node = SanityGateCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
