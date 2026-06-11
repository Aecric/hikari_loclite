#pragma once

#ifndef HIKARI_LOCLITE_STABILITY_GATE_HPP
#define HIKARI_LOCLITE_STABILITY_GATE_HPP

#include <cmath>
#include <deque>
#include <utility>

#include "common/eigen_types.h"
#include "log.h"

namespace hikari::loclite {

/// 轻量稳定门控 (lightning Localization::ApplyStabilityGate 的精简版, localization.cpp:1543 /
/// localization.h StabilityGate 结构):
/// /initialpose 经 NDT 验证通过后不直接放行 Good, 先进入 Initializing; 滑窗 (timestamp, SE3)
/// 内位姿抖动持续低于阈值且窗口覆盖 >= window_sec 才放行; NDT 置信度 (TP) >= conf_upper_thres
/// 视为强可信验证, 提前放行.
///
/// 与 lightning 的语义差异: lightning 的 stability_gate_conf_upper_thres 是"高分伪匹配守门"
/// (conf 过高反而不放行); 本包无级联 NDT / 覆盖率指标, 轻量版按任务拍板改为"高 TP 提前放行",
/// 伪匹配防护交给 NdtCorrector::Validate 的 delta 门限兜底.
///
/// 线程安全: 无内部锁, 由 LocLiteNode 在 mutex_ 内串行调用.
class StabilityGate {
   public:
    struct Options {
        bool enabled = true;
        double trans_thres = 0.1;       // 窗口内最大平移抖动 (m)
        double rot_thres_deg = 4.0;     // 窗口内最大旋转抖动 (deg)
        double window_sec = 3.0;        // 滑动窗口长度 (s)
        // TP >= 此值提前放行; <=0 关闭提前放行. TP 量纲同 ndt.min_confidence
        // (真匹配通常 1.5~5, 见 lightning default_livox.yaml:136), 默认 3.0 = 真匹配区间中上水平.
        double conf_upper_thres = 3.0;
    };

    void Init(const Options& options) { options_ = options; }
    const Options& GetOptions() const { return options_; }

    /// 重新武装: /initialpose 验证通过后调用, 清滑窗重新累计 (lightning re-arm 同款)
    void Reset() {
        armed_ = true;
        buf_.clear();
    }

    bool Armed() const { return armed_; }

    /// 喂入一帧 (timestamp, 输出位姿, 最近一次 NDT TP). 返回 true 表示放行 (内部自动 disarm);
    /// 已放行后恒返回 true, 直到下一次 Reset().
    bool Observe(double t, const SE3& pose, double conf) {
        if (!armed_) {
            return true;
        }

        // 提前放行: NDT 验证 TP 已达强可信水平
        if (options_.conf_upper_thres > 0.0 && conf >= options_.conf_upper_thres) {
            armed_ = false;
            buf_.clear();
            LOG(INFO) << "[StabilityGate] early release: TP=" << conf
                      << " >= " << options_.conf_upper_thres;
            return true;
        }

        // 时间戳回退 (bag 回放/重启) → 清缓冲重新累计, 避免窗口长度算错 (lightning 同款处理)
        if (!buf_.empty() && t < buf_.back().first) {
            buf_.clear();
        }
        buf_.emplace_back(t, pose);
        while (!buf_.empty() && t - buf_.front().first > options_.window_sec) {
            buf_.pop_front();
        }

        // 窗口未覆盖足够时长 → 继续等 (0.8 系数照 lightning, 容忍帧率抖动)
        if (buf_.size() < 2 || (t - buf_.front().first) < options_.window_sec * 0.8) {
            return false;
        }

        // 取窗口内相对首帧的最大 Δt / Δr
        const SE3& p0 = buf_.front().second;
        const double rot_thres_rad = options_.rot_thres_deg * M_PI / 180.0;
        double max_dt = 0.0;
        double max_dr = 0.0;
        for (const auto& sample : buf_) {
            const double dt = (sample.second.translation() - p0.translation()).norm();
            const double dr = (sample.second.so3() * p0.so3().inverse()).log().norm();
            if (dt > max_dt) max_dt = dt;
            if (dr > max_dr) max_dr = dr;
        }

        if (max_dt <= options_.trans_thres && max_dr <= rot_thres_rad) {
            armed_ = false;
            buf_.clear();
            LOG(INFO) << "[StabilityGate] released: dt_max=" << max_dt
                      << "m, dr_max=" << max_dr * 180.0 / M_PI
                      << "deg, window=" << options_.window_sec << "s";
            return true;
        }
        return false;
    }

   private:
    Options options_;
    bool armed_ = true;
    std::deque<std::pair<double, SE3>> buf_;
};

}  // namespace hikari::loclite

#endif
