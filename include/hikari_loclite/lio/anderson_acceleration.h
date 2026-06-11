#pragma once

#ifndef HIKARI_LOCLITE_ANDERSON_ACCELERATION_H
#define HIKARI_LOCLITE_ANDERSON_ACCELERATION_H

#include <algorithm>
#include <cassert>
#include <vector>

#include "common/eigen_types.h"
#include "log.h"

namespace hikari::loclite {

template <typename S, int D, int m>
class AndersonAcceleration {
   public:
    using Scalar = S;
    using Vec = Eigen::Matrix<S, D, 1>;
    using MatDM = Eigen::Matrix<S, D, m>;
    using MatDD = Eigen::Matrix<S, D, D>;

    Vec compute(const Vec& g) {
        assert(iter_ >= 0);
        Vec G = g;
        current_F_ = G - current_u_;

        if (iter_ == 0) {
            prev_dF_.col(0) = -current_F_;
            prev_dG_.col(0) = -G;
            current_u_ = G;
        } else {
            prev_dF_.col(col_idx_) += current_F_;
            prev_dG_.col(col_idx_) += G;

            Scalar eps = 1e-14;
            Scalar scale = std::max(eps, prev_dF_.col(col_idx_).norm());
            dF_scale_(col_idx_) = scale;
            prev_dF_.col(col_idx_) /= scale;

            int m_k = std::min(m, iter_);

            if (m_k == 1) {
                theta_(0) = 0;
                Scalar dF_sqrnorm = prev_dF_.col(col_idx_).squaredNorm();
                M_(0, 0) = dF_sqrnorm;
                Scalar dF_norm = std::sqrt(dF_sqrnorm);
                if (dF_norm > eps) {
                    theta_(0) = (prev_dF_.col(col_idx_) / dF_norm).dot(current_F_ / dF_norm);
                }
            } else {
                VecXd new_inner_prod = (prev_dF_.col(col_idx_).transpose() * prev_dF_.block(0, 0, D, m_k)).transpose();
                M_.block(col_idx_, 0, 1, m_k) = new_inner_prod.transpose();
                M_.block(0, col_idx_, m_k, 1) = new_inner_prod;
                cod_.compute(M_.block(0, 0, m_k, m_k));
                theta_.head(m_k) = cod_.solve(prev_dF_.block(0, 0, D, m_k).transpose() * current_F_);
            }

            current_u_ =
                G - prev_dG_.block(0, 0, D, m_k) * ((theta_.head(m_k).array() / dF_scale_.head(m_k).array()).matrix());
            col_idx_ = (col_idx_ + 1) % m;
            prev_dF_.col(col_idx_) = -current_F_;
            prev_dG_.col(col_idx_) = -G;
        }

        if (!current_u_.allFinite()) {
            LOG(WARNING) << "AA produced non-finite output, falling back to raw update";
            current_u_ = g;
            iter_ = 0;
            col_idx_ = 0;
        }

        iter_++;
        return current_u_;
    }

    void reset(const Vec& u) {
        iter_ = 0;
        col_idx_ = 0;
        current_u_ = u;
    }

    void init(const Vec& u0) {
        current_u_.setZero();
        current_F_.setZero();
        prev_dG_.setZero();
        prev_dF_.setZero();
        M_.setZero();
        theta_.setZero();
        dF_scale_.setZero();
        current_u_ = u0;
        iter_ = 0;
        col_idx_ = 0;
    }

   private:
    Vec current_u_ = Vec::Zero();
    Vec current_F_ = Vec::Zero();
    MatDM prev_dG_ = MatDM::Zero();
    MatDM prev_dF_ = MatDM::Zero();
    MatDD M_ = MatDD::Zero();
    Vec theta_ = Vec::Zero();
    Vec dF_scale_ = Vec::Zero();
    Eigen::CompleteOrthogonalDecomposition<MatXd> cod_;
    int iter_ = 0;
    int col_idx_ = -1;
};

}  // namespace hikari::loclite

#endif
