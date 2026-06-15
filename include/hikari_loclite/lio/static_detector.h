#pragma once

#ifndef HIKARI_LOCLITE_STATIC_DETECTOR_H
#define HIKARI_LOCLITE_STATIC_DETECTOR_H

#include "common/eigen_types.h"
#include "common/imu.h"

#include <cmath>
#include <deque>

namespace hikari::loclite {

// StaticDetector — ZUPT 静止检测器 (Phase 1, header-only, 仿 imu_filter.h 风格).
//
// 关键约束: 逻辑必须与 scripts/zupt_calib/zupt_detector_sim.py 完全一致, 这样用户用
// 那个 python 节点标定出的参数能 1:1 搬进来:
//   - 按 IMU 时间戳维护 window_sec 时间滑窗 (deque), 每个 IMU 样本算窗内 gyro-模 std
//     与 acc-模 std (总体标准差 ddof=0, 与 numpy .std() 一致);
//   - raw_static = (gyro_norm_std < gyro_std_thres) && (acc_norm_std < acc_std_thres);
//   - 非对称迟滞计数器以 *IMU 样本* 为单位: 连续 raw_static >= park_enter_frames 进静止 (进慢),
//     连续 raw_moving >= park_exit_frames 出静止 (出快);
//   - 前 warmup_frames 个样本只填窗不判定 (等滑窗填满 / IMU 稳定).
//
// 单位说明: park_enter_frames / park_exit_frames / warmup_frames 的单位是 *IMU 样本数*
// (非 lidar 帧), 需用 zupt_detector_sim.py 在线标定后填进 yaml.
class StaticDetector {
   public:
    struct Config {
        double gyro_std_thres = 0.01;  // rad/s, gyro 模 std 静止阈值 [待 imu_static_stats.py 标定]
        double acc_std_thres = 0.15;   // m/s^2, acc 模 std 静止阈值 [待 imu_static_stats.py 标定]
        double window_sec = 0.5;       // 静止检测滑窗时长 (秒), 与标定时一致
        int park_enter_frames = 10;    // 连续静止多少 IMU 样本才进泊车 (进慢) [zupt_detector_sim.py 标定]
        int park_exit_frames = 3;      // 连续运动多少 IMU 样本才出泊车 (出快) [同上]
        int warmup_frames = 50;        // 启动丢弃 IMU 样本数
    };

    StaticDetector() = default;
    explicit StaticDetector(const Config& cfg) : config_(cfg) {}

    void SetConfig(const Config& cfg) { config_ = cfg; }
    const Config& GetConfig() const { return config_; }

    // 按 IMU 时间戳推进滑窗 + 迟滞判定. 每个 IMU 样本调一次.
    void AddImu(const IMU& imu) {
        const double t = imu.timestamp;
        buf_.push_back(imu);
        while (!buf_.empty() && (t - buf_.front().timestamp) > config_.window_sec) {
            buf_.pop_front();
        }

        ++n_seen_;
        // 前 warmup_frames 个样本只填窗不判定; 窗内样本不足 3 也不判定.
        if (n_seen_ <= config_.warmup_frames || buf_.size() < 3) {
            return;
        }

        const double gyro_norm_std = NormStd(buf_, /*is_gyro=*/true);
        const double acc_norm_std = NormStd(buf_, /*is_gyro=*/false);
        const bool raw_static =
            (gyro_norm_std < config_.gyro_std_thres) && (acc_norm_std < config_.acc_std_thres);

        // 非对称迟滞 (与 python 逐分支一致).
        if (raw_static) {
            ++enter_cnt_;
            exit_cnt_ = 0;
            if (!is_static_ && enter_cnt_ >= config_.park_enter_frames) {
                is_static_ = true;
            }
        } else {
            ++exit_cnt_;
            enter_cnt_ = 0;
            if (is_static_ && exit_cnt_ >= config_.park_exit_frames) {
                is_static_ = false;
            }
        }
    }

    bool IsStatic() const { return is_static_; }

    // 清空窗与计数器, 位姿/速度域中断时调 (例如 ResetToMapPose).
    void Reset() {
        buf_.clear();
        n_seen_ = 0;
        enter_cnt_ = 0;
        exit_cnt_ = 0;
        is_static_ = false;
    }

   private:
    // 窗内每个样本 gyro/acc 三轴模的总体标准差 (ddof=0, 与 numpy .std() 一致).
    static double NormStd(const std::deque<IMU>& buf, bool is_gyro) {
        const size_t n = buf.size();
        if (n == 0) return 0.0;
        double mean = 0.0;
        for (const auto& s : buf) {
            const Vec3d& v = is_gyro ? s.angular_velocity : s.linear_acceleration;
            mean += v.norm();
        }
        mean /= static_cast<double>(n);
        double var = 0.0;
        for (const auto& s : buf) {
            const Vec3d& v = is_gyro ? s.angular_velocity : s.linear_acceleration;
            const double d = v.norm() - mean;
            var += d * d;
        }
        var /= static_cast<double>(n);
        return std::sqrt(var);
    }

    Config config_;
    std::deque<IMU> buf_;  // (时间戳, gyro, acc) 滑窗
    int n_seen_ = 0;
    int enter_cnt_ = 0;
    int exit_cnt_ = 0;
    bool is_static_ = false;
};

}  // namespace hikari::loclite

#endif
