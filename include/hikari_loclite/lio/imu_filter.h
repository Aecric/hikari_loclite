#pragma once

#ifndef HIKARI_LOCLITE_IMU_FILTER_H
#define HIKARI_LOCLITE_IMU_FILTER_H

#include "common/eigen_types.h"
#include "common/imu.h"
#include "log.h"

#include <cstdint>

namespace hikari::loclite {

struct FilterResult {
    IMU imu;
    double cov_scale = 1.0;
};

class IMUFilter {
   public:
    struct Config {
        bool enabled = true;
        double acc_norm_min = 4.0;
        double acc_norm_max = 18.0;
        double acc_delta_max = 8.0;
        double acc_clamp_norm_max = 25.0;
        double acc_cov_scale_on_spike = 10.0;
        double baseline_alpha = 0.01;
        double spike_log_rate_hz = 1.0;
    };

    struct DiagCounters {
        uint64_t total = 0;
        uint64_t soft_spike = 0;
        uint64_t hard_clamp = 0;
        uint64_t cov_inflated = 0;
    };

    enum class SpikeClass { Normal, SoftSpike, HardSpike };

    IMUFilter() = default;
    explicit IMUFilter(const Config& cfg) : config_(cfg) {}

    void SetConfig(const Config& cfg) { config_ = cfg; }
    const Config& GetConfig() const { return config_; }

    DiagCounters GetAndResetCounters() {
        auto c = counters_;
        counters_ = {};
        return c;
    }

    FilterResult Filter(const IMU& raw) {
        FilterResult result;
        result.imu = raw;

        if (!initialized_) {
            acc_norm_mean_ = raw.linear_acceleration.norm();
            prev_acc_ = raw.linear_acceleration;
            prev_acc_norm_ = acc_norm_mean_;
            has_prev_ = true;
            initialized_ = true;
            counters_.total++;
            return result;
        }

        double acc_norm = raw.linear_acceleration.norm();
        double acc_delta = has_prev_ ? (raw.linear_acceleration - prev_acc_).norm() : 0.0;

        SpikeClass cls = classifySpike(acc_norm, acc_delta);

        if (cls == SpikeClass::HardSpike) {
            Vec3d dir = raw.linear_acceleration.normalized();
            if (dir.norm() > 0.5) {
                result.imu.linear_acceleration = dir * config_.acc_clamp_norm_max;
            }
            result.cov_scale = config_.acc_cov_scale_on_spike;
            counters_.hard_clamp++;
            counters_.cov_inflated++;
        } else if (cls == SpikeClass::SoftSpike) {
            result.cov_scale = config_.acc_cov_scale_on_spike;
            counters_.soft_spike++;
            counters_.cov_inflated++;
        }

        updateBaseline(acc_norm);
        prev_acc_ = raw.linear_acceleration;
        prev_acc_norm_ = acc_norm;
        has_prev_ = true;
        counters_.total++;

        maybeLogDiag(raw.timestamp);
        return result;
    }

   private:
    Config config_;
    DiagCounters counters_;

    double acc_norm_mean_ = 9.81;
    bool initialized_ = false;

    Vec3d prev_acc_ = Vec3d::Zero();
    double prev_acc_norm_ = 0.0;
    bool has_prev_ = false;

    double last_log_time_ = 0.0;

    SpikeClass classifySpike(double acc_norm, double acc_delta) const {
        if (acc_norm > config_.acc_clamp_norm_max) {
            return SpikeClass::HardSpike;
        }
        if (acc_norm > config_.acc_norm_max || acc_norm < config_.acc_norm_min || acc_delta > config_.acc_delta_max) {
            return SpikeClass::SoftSpike;
        }
        return SpikeClass::Normal;
    }

    void updateBaseline(double acc_norm) {
        acc_norm_mean_ = (1.0 - config_.baseline_alpha) * acc_norm_mean_ + config_.baseline_alpha * acc_norm;
    }

    void maybeLogDiag(double now) {
        if (config_.spike_log_rate_hz <= 0.0) return;
        double interval = 1.0 / config_.spike_log_rate_hz;
        if (now - last_log_time_ < interval) return;
        last_log_time_ = now;

        double total = static_cast<double>(counters_.total);
        if (total == 0) return;

        LOG(INFO) << "IMUFilter diag: total=" << counters_.total
                  << " soft_spike=" << counters_.soft_spike
                  << "(" << (100.0 * counters_.soft_spike / total) << "%)"
                  << " hard_clamp=" << counters_.hard_clamp
                  << "(" << (100.0 * counters_.hard_clamp / total) << "%)"
                  << " cov_inflated=" << counters_.cov_inflated;

        counters_ = {};
    }
};

}  // namespace hikari::loclite

#endif
