#pragma once

#ifndef HIKARI_LOCLITE_POSE_SMOOTHER_HPP
#define HIKARI_LOCLITE_POSE_SMOOTHER_HPP

#include "common/eigen_types.h"

namespace hikari::loclite {

class LitePoseSmoother {
   public:
    /// 四个跳变门限 (Phase1 改为可配, 缺省即新放宽后的值):
    /// max_correction 默认从 0.3m/5° 放宽到 0.5m/8° —— TP 门已正确标定 (ndt.min_confidence=1.0)
    /// 且有 inlier 兜底, GOOD 态中等漂移可被 NDT 校正拉回; 大漂移 (>0.5m) 不是 GOOD 微调的职责,
    /// 交后续 Phase 的状态机降级 + SC 重定位处理.
    struct Options {
        double max_output_jump_trans_m = 0.5;
        double max_output_jump_rot_deg = 15.0;
        double max_correction_trans_m = 0.5;
        double max_correction_rot_deg = 8.0;
    };

    /// 写入四个门限 (由 LocLiteNode::Init 读 yaml smoother.* 后调用; 不调则用 Options 默认值)
    void Init(const Options& options) {
        max_output_jump_trans_m_ = options.max_output_jump_trans_m;
        max_output_jump_rot_deg_ = options.max_output_jump_rot_deg;
        max_correction_trans_m_ = options.max_correction_trans_m;
        max_correction_rot_deg_ = options.max_correction_rot_deg;
    }

    SE3 UpdateFastLioPose(const SE3& pose) {
        if (!initialized_) {
            last_pose_ = pose;
            initialized_ = true;
            return pose;
        }

        const SE3 delta = last_pose_.inverse() * pose;
        const double dt = delta.translation().norm();
        const double dr = delta.so3().log().norm() * 180.0 / M_PI;
        if (dt > max_output_jump_trans_m_ || dr > max_output_jump_rot_deg_) {
            return last_pose_;
        }

        last_pose_ = pose;
        return last_pose_;
    }

    SE3 ApplyNdtCorrection(const SE3& fast_pose, const SE3& ndt_pose, double gain) {
        const SE3 delta = fast_pose.inverse() * ndt_pose;
        const double dt = delta.translation().norm();
        const double dr = delta.so3().log().norm() * 180.0 / M_PI;
        if (dt > max_correction_trans_m_ || dr > max_correction_rot_deg_) {
            return fast_pose;
        }

        Eigen::Vector3d t = fast_pose.translation() * (1.0 - gain) + ndt_pose.translation() * gain;
        SO3 r = fast_pose.so3() * SO3::exp(delta.so3().log() * gain);
        return SE3(r, t);
    }

    void Reset() { initialized_ = false; }

   private:
    bool initialized_ = false;
    SE3 last_pose_;
    // 默认值与 Options 默认对齐 (Phase1: max_correction 0.3/5 → 0.5/8); 实际由 Init(Options) 覆盖
    double max_output_jump_trans_m_ = 0.5;
    double max_output_jump_rot_deg_ = 15.0;
    double max_correction_trans_m_ = 0.5;
    double max_correction_rot_deg_ = 8.0;
};

}  // namespace hikari::loclite

#endif
