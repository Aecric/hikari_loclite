#pragma once

#ifndef HIKARI_LOCLITE_IMU_CONSISTENCY_GATE_HPP
#define HIKARI_LOCLITE_IMU_CONSISTENCY_GATE_HPP

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>

#include "common/eigen_types.h"

namespace hikari::loclite {

class ImuConsistencyGate {
   public:
    struct Options {
        bool enabled = false;
        double window_sec = 1.0;
        double min_check_interval_sec = 0.2;
        double max_imu_gap_sec = 0.2;
        double loc_static_speed_mps = 0.03;
        double loc_static_dist_m = 0.03;
        double loc_static_yaw_rad = 0.03;
        double loc_moving_dist_m = 0.15;
        double loc_moving_yaw_rad = 0.10;
        double gyro_dynamic_radps = 0.08;
        double gyro_yaw_delta_rad = 0.08;
        double acc_dynamic_mps2 = 0.20;
        double acc_window_energy_mps2 = 0.50;
        int fail_windows = 3;
        double log_rate_hz = 1.0;
    };

    struct Result {
        bool checked = false;
        bool valid_window = false;
        bool fail = false;
        bool triggered = false;
        std::string reason;
        int fail_count = 0;

        double imu_window_sec = 0.0;
        double loc_window_sec = 0.0;
        double gyro_mean_radps = 0.0;
        double gyro_yaw_delta_rad = 0.0;
        double acc_norm_std_mps2 = 0.0;
        double acc_norm_delta_mean_mps2 = 0.0;
        double loc_dist_m = 0.0;
        double loc_yaw_delta_rad = 0.0;
        double loc_max_speed_mps = 0.0;
        bool imu_static = false;
        bool imu_dynamic = false;
        bool loc_static = false;
        bool loc_moving = false;
    };

    void SetOptions(const Options& options) {
        options_ = options;
        if (options_.fail_windows < 1) {
            options_.fail_windows = 1;
        }
        if (options_.window_sec <= 0.0) {
            options_.window_sec = 1.0;
        }
        if (options_.min_check_interval_sec < 0.0) {
            options_.min_check_interval_sec = 0.0;
        }
        if (options_.max_imu_gap_sec <= 0.0) {
            options_.max_imu_gap_sec = 0.2;
        }
        Reset();
    }

    const Options& GetOptions() const { return options_; }

    void AddImu(double timestamp, const Vec3d& gyro, const Vec3d& acc) {
        if (!options_.enabled || !std::isfinite(timestamp) || timestamp <= 0.0) {
            return;
        }
        if (!imu_buf_.empty()) {
            const double dt = timestamp - imu_buf_.back().timestamp;
            if (dt <= 0.0 || dt > options_.max_imu_gap_sec) {
                ResetImu();
            }
        }

        imu_buf_.push_back({timestamp, gyro, acc});
        PruneImu(timestamp);
    }

    Result ObserveLocalization(double timestamp, const Vec3d& position, double yaw, double speed_mps) {
        Result result;
        result.fail_count = fail_count_;
        if (!options_.enabled || !std::isfinite(timestamp) || timestamp <= 0.0) {
            return result;
        }
        if (last_check_ts_ >= 0.0 && timestamp - last_check_ts_ < options_.min_check_interval_sec) {
            return result;
        }
        last_check_ts_ = timestamp;
        result.checked = true;

        if (!loc_buf_.empty() && timestamp <= loc_buf_.back().timestamp) {
            ResetLocalization();
        }
        loc_buf_.push_back({timestamp, position, WrapAngle(yaw), std::max(0.0, speed_mps)});
        PruneLocalization(timestamp);

        if (!BuildMetrics(result)) {
            fail_count_ = 0;
            result.fail_count = fail_count_;
            return result;
        }

        result.valid_window = true;
        if (result.imu_static && result.loc_moving) {
            result.fail = true;
            result.reason = "imu_consistency_static_loc_moving";
        } else if (result.imu_dynamic && result.loc_static) {
            result.fail = true;
            result.reason = "imu_consistency_imu_moving_loc_static";
        }

        if (result.fail) {
            ++fail_count_;
        } else {
            fail_count_ = 0;
        }
        result.fail_count = fail_count_;
        result.triggered = result.fail && fail_count_ >= options_.fail_windows;
        return result;
    }

    void Reset() {
        ResetImu();
        ResetLocalization();
    }

    void ResetLocalization() {
        loc_buf_.clear();
        fail_count_ = 0;
        last_check_ts_ = -1.0;
    }

   private:
    struct ImuSample {
        double timestamp = 0.0;
        Vec3d gyro = Vec3d::Zero();
        Vec3d acc = Vec3d::Zero();
    };

    struct LocSample {
        double timestamp = 0.0;
        Vec3d position = Vec3d::Zero();
        double yaw = 0.0;
        double speed_mps = 0.0;
    };

    void ResetImu() { imu_buf_.clear(); }

    void PruneImu(double now) {
        while (!imu_buf_.empty() && now - imu_buf_.front().timestamp > options_.window_sec) {
            imu_buf_.pop_front();
        }
    }

    void PruneLocalization(double now) {
        while (!loc_buf_.empty() && now - loc_buf_.front().timestamp > options_.window_sec) {
            loc_buf_.pop_front();
        }
    }

    static double WrapAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

    static double YawDelta(double from, double to) { return WrapAngle(to - from); }

    bool BuildMetrics(Result& result) const {
        if (imu_buf_.size() < 3 || loc_buf_.size() < 2) {
            return false;
        }

        result.imu_window_sec = imu_buf_.back().timestamp - imu_buf_.front().timestamp;
        result.loc_window_sec = loc_buf_.back().timestamp - loc_buf_.front().timestamp;
        if (result.imu_window_sec < options_.window_sec * 0.8 ||
            result.loc_window_sec < options_.window_sec * 0.8) {
            return false;
        }

        double gyro_norm_sum = 0.0;
        double acc_norm_sum = 0.0;
        double acc_delta_sum = 0.0;
        double yaw_integral = 0.0;
        for (size_t i = 0; i < imu_buf_.size(); ++i) {
            gyro_norm_sum += imu_buf_[i].gyro.norm();
            acc_norm_sum += imu_buf_[i].acc.norm();
            if (i > 0) {
                const double dt = imu_buf_[i].timestamp - imu_buf_[i - 1].timestamp;
                if (dt > 0.0) {
                    yaw_integral += 0.5 * (imu_buf_[i - 1].gyro.z() + imu_buf_[i].gyro.z()) * dt;
                }
                acc_delta_sum += std::abs(imu_buf_[i].acc.norm() - imu_buf_[i - 1].acc.norm());
            }
        }

        const double n = static_cast<double>(imu_buf_.size());
        const double acc_norm_mean = acc_norm_sum / n;
        double acc_var = 0.0;
        for (const auto& sample : imu_buf_) {
            const double d = sample.acc.norm() - acc_norm_mean;
            acc_var += d * d;
        }
        acc_var /= n;

        result.gyro_mean_radps = gyro_norm_sum / n;
        result.gyro_yaw_delta_rad = std::abs(yaw_integral);
        result.acc_norm_std_mps2 = std::sqrt(acc_var);
        result.acc_norm_delta_mean_mps2 = acc_delta_sum / static_cast<double>(imu_buf_.size() - 1);

        const auto& first = loc_buf_.front();
        const auto& last = loc_buf_.back();
        result.loc_dist_m = (last.position - first.position).norm();
        result.loc_yaw_delta_rad = std::abs(YawDelta(first.yaw, last.yaw));
        result.loc_max_speed_mps = 0.0;
        for (const auto& sample : loc_buf_) {
            result.loc_max_speed_mps = std::max(result.loc_max_speed_mps, sample.speed_mps);
        }

        result.imu_static = result.gyro_mean_radps <= options_.gyro_dynamic_radps &&
                            result.gyro_yaw_delta_rad <= options_.gyro_yaw_delta_rad &&
                            result.acc_norm_std_mps2 <= options_.acc_dynamic_mps2 &&
                            result.acc_norm_delta_mean_mps2 <= options_.acc_window_energy_mps2;
        result.imu_dynamic = result.gyro_mean_radps > options_.gyro_dynamic_radps ||
                             result.gyro_yaw_delta_rad > options_.gyro_yaw_delta_rad ||
                             result.acc_norm_std_mps2 > options_.acc_dynamic_mps2 ||
                             result.acc_norm_delta_mean_mps2 > options_.acc_window_energy_mps2;
        result.loc_static = result.loc_dist_m <= options_.loc_static_dist_m &&
                            result.loc_yaw_delta_rad <= options_.loc_static_yaw_rad &&
                            result.loc_max_speed_mps <= options_.loc_static_speed_mps;
        result.loc_moving = result.loc_dist_m >= options_.loc_moving_dist_m ||
                            result.loc_yaw_delta_rad >= options_.loc_moving_yaw_rad ||
                            result.loc_max_speed_mps > options_.loc_static_speed_mps;
        return true;
    }

    Options options_;
    std::deque<ImuSample> imu_buf_;
    std::deque<LocSample> loc_buf_;
    int fail_count_ = 0;
    double last_check_ts_ = -1.0;
};

}  // namespace hikari::loclite

#endif
