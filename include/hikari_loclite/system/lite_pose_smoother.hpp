#pragma once

#ifndef HIKARI_LOCLITE_POSE_SMOOTHER_HPP
#define HIKARI_LOCLITE_POSE_SMOOTHER_HPP

#include "common/eigen_types.h"

namespace hikari::loclite {

class LitePoseSmoother {
   public:
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
    double max_output_jump_trans_m_ = 0.5;
    double max_output_jump_rot_deg_ = 15.0;
    double max_correction_trans_m_ = 0.3;
    double max_correction_rot_deg_ = 5.0;
};

}  // namespace hikari::loclite

#endif
