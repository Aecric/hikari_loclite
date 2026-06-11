#pragma once

#ifndef HIKARI_LOCLITE_NAV_STATE_H
#define HIKARI_LOCLITE_NAV_STATE_H

#include "common/eigen_types.h"

#include <iomanip>
#include <iostream>
#include <vector>

namespace hikari::loclite {

struct NavState {
    constexpr static int dim = 12;
    constexpr static int full_dim = 12;
    constexpr static int kBlockDim = 3;
    constexpr static int kPosIdx = 0;
    constexpr static int kRotIdx = 3;
    constexpr static int kVelIdx = 6;
    constexpr static int kBgIdx = 9;

    using VectState = Eigen::Matrix<double, dim, 1>;
    using FullVectState = Eigen::Matrix<double, full_dim, 1>;

    NavState() = default;

    bool operator<(const NavState& other) { return timestamp_ < other.timestamp_; }

    FullVectState ToState() {
        FullVectState ret;
        ret.block<kBlockDim, 1>(kPosIdx, 0) = pos_;
        ret.block<kBlockDim, 1>(kRotIdx, 0) = rot_.log();
        ret.block<kBlockDim, 1>(kVelIdx, 0) = vel_;
        ret.block<kBlockDim, 1>(kBgIdx, 0) = bg_;
        return ret;
    }

    void FromVectState(const FullVectState& state) {
        pos_ = state.block<kBlockDim, 1>(kPosIdx, 0);
        rot_ = SO3::exp(state.block<kBlockDim, 1>(kRotIdx, 0));
        vel_ = state.block<kBlockDim, 1>(kVelIdx, 0);
        bg_ = state.block<kBlockDim, 1>(kBgIdx, 0);
    }

    inline FullVectState get_f(const Vec3d& gyro, const Vec3d& acce) const {
        FullVectState res = FullVectState::Zero();
        Vec3d omega = gyro - bg_;
        Vec3d a_inertial = rot_ * acce;
        for (int i = 0; i < 3; i++) {
            res(i) = vel_[i];
            res(i + kRotIdx) = omega[i];
            res(i + kVelIdx) = a_inertial[i] + grav_[i];
        }
        return res;
    }

    inline Eigen::Matrix<double, full_dim, dim> df_dx(const Vec3d& acce) const {
        Eigen::Matrix<double, full_dim, dim> cov = Eigen::Matrix<double, full_dim, dim>::Zero();
        cov.block<kBlockDim, kBlockDim>(kPosIdx, kVelIdx) = Mat3d::Identity();
        Vec3d acc = acce;
        cov.block<kBlockDim, kBlockDim>(kVelIdx, kRotIdx) = -rot_.matrix() * SO3::hat(acc);
        cov.block<kBlockDim, kBlockDim>(kRotIdx, kBgIdx) = -Eigen::Matrix3d::Identity();
        return cov;
    }

    inline Eigen::Matrix<double, full_dim, 12> df_dw() const {
        Eigen::Matrix<double, full_dim, 12> cov = Eigen::Matrix<double, full_dim, 12>::Zero();
        cov.block<kBlockDim, kBlockDim>(kVelIdx, 3) = -rot_.matrix();
        cov.block<kBlockDim, kBlockDim>(kRotIdx, 0) = -Eigen::Matrix3d::Identity();
        cov.block<kBlockDim, kBlockDim>(kBgIdx, 6) = Eigen::Matrix3d::Identity();
        return cov;
    }

    void oplus(const FullVectState& vec, double dt) {
        timestamp_ += dt;
        pos_ += vec.middleRows(kPosIdx, kBlockDim) * dt;
        rot_ = rot_ * SO3::exp(vec.middleRows(kRotIdx, kBlockDim) * dt);
        bg_ += vec.middleRows(kBgIdx, kBlockDim) * dt;
    }

    VectState boxminus(const NavState& other) {
        VectState result;
        result.block<kBlockDim, 1>(kPosIdx, 0) = pos_ - other.pos_;
        result.block<kBlockDim, 1>(kRotIdx, 0) = (other.rot_.inverse() * rot_).log();
        result.block<kBlockDim, 1>(kVelIdx, 0) = vel_ - other.vel_;
        result.block<kBlockDim, 1>(kBgIdx, 0) = bg_ - other.bg_;
        return result;
    }

    NavState boxplus(const VectState& dx) {
        NavState ret;
        ret.timestamp_ = timestamp_;
        ret.pos_ = pos_ + dx.middleRows(kPosIdx, kBlockDim);
        ret.rot_ = rot_ * SO3::exp(dx.middleRows(kRotIdx, kBlockDim));
        ret.vel_ = vel_ + dx.middleRows(kVelIdx, kBlockDim);
        ret.bg_ = bg_ + dx.middleRows(kBgIdx, kBlockDim);
        ret.grav_ = grav_;
        return ret;
    }

    struct MetaInfo {
        MetaInfo(int idx, int vdim, int dof) : idx_(idx), dim_(vdim), dof_(dof) {}
        int idx_ = 0;
        int dim_ = 0;
        int dof_ = 0;
    };

    static const std::vector<MetaInfo> vect_states_;
    static const std::vector<MetaInfo> SO3_states_;

    friend inline std::ostream& operator<<(std::ostream& os, const NavState& s) {
        os << std::setprecision(18) << s.pos_.transpose() << " " << s.rot_.unit_quaternion().coeffs().transpose() << " "
           << s.vel_.transpose() << " " << s.bg_.transpose() << " " << s.grav_.transpose();
        return os;
    }

    inline SE3 GetPose() const { return SE3(rot_, pos_); }
    inline SO3 GetRot() const { return rot_; }
    inline void SetPose(const SE3& pose) {
        rot_ = pose.so3();
        pos_ = pose.translation();
    }

    inline Vec3d Getba() const { return Vec3d::Zero(); }
    inline Vec3d Getbg() const { return bg_; }
    inline Vec3d GetVel() const { return vel_; }
    void SetVel(const Vec3d& v) { vel_ = v; }

    double timestamp_ = 0.0;
    double confidence_ = 0.0;
    bool pose_is_ok_ = true;
    bool lidar_odom_reliable_ = true;
    bool is_parking_ = false;

    Vec3d pos_ = Vec3d::Zero();
    SO3 rot_;
    Vec3d vel_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();
    Vec3d grav_ = Vec3d(0.0, 0.0, -9.81);

    Eigen::Matrix<double, 6, 6> pose_cov_ = Eigen::Matrix<double, 6, 6>::Zero();
    bool pose_cov_valid_ = false;
};

}  // namespace hikari::loclite

#endif
