#pragma once

#ifndef HIKARI_LOCLITE_NDT_DEGENERACY_CHECK_HPP
#define HIKARI_LOCLITE_NDT_DEGENERACY_CHECK_HPP

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace hikari::loclite {

/// NDT Hessian 退化检测结果 (从 lightning gating_features.h 提取, 简化: 世界系分析, 无 R_body)
struct DegeneracyResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // --- 平移退化 ---
    Eigen::Vector3d trans_eigenvalues   = Eigen::Vector3d::Zero();
    Eigen::Matrix3d trans_eigenvectors  = Eigen::Matrix3d::Identity();
    double trans_lambda_min             = 0.0;
    int    trans_min_idx                = 0;
    Eigen::Vector3d trans_degen_dir     = Eigen::Vector3d::Zero();
    double trans_condition_ratio        = 0.0;  // lambda_max / lambda_min, 越大退化越严重

    // --- 旋转退化 ---
    Eigen::Vector3d rot_eigenvalues     = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot_eigenvectors    = Eigen::Matrix3d::Identity();
    double rot_lambda_min               = 0.0;
    int    rot_min_idx                  = 0;
    Eigen::Vector3d rot_degen_dir       = Eigen::Vector3d::Zero();
    double rot_condition_ratio          = 0.0;

    // --- 综合判定 ---
    bool trans_degenerated = false;
    bool rot_degenerated   = false;
    bool is_degenerated    = false;  // trans || rot
};

/**
 * @brief 退化检测 (Degeneracy Eigen-Check)
 *
 * 从 NDT 优化收敛后的 6x6 Hessian 矩阵中, 提取平移和旋转的约束强度.
 *
 * 数学原理:
 *   NDT 配准的目标函数对参数向量 p=[tx,ty,tz,rx,ry,rz] 求二阶导得到 6x6 Hessian H.
 *   H 的对角块:
 *     H[0:3, 0:3] -> 平移维 Hessian (反映地图对平移的约束力)
 *     H[3:6, 3:6] -> 旋转维 Hessian (反映地图对旋转的约束力)
 *   对各块做特征值分解:
 *     - lambda_min 很小 -> 该方向上地图对 NDT 几乎没有约束 (退化)
 *     - lambda_max / lambda_min 很大 -> 约束各向异性严重 (长走廊、平面)
 *
 * @param hessian_6x6      NDT 最后一次迭代的 6x6 Hessian (pclomp::NDT::getHessionMatrix())
 * @param ratio_threshold  lambda_max / lambda_min 超过此值判定退化 (典型值 50~100)
 * @param min_ev_threshold lambda_min 低于此绝对值也判定退化 (典型值 5~20)
 * @return DegeneracyResult
 */
inline DegeneracyResult DegeneracyEigenCheck(
    const Eigen::Matrix<double, 6, 6>& hessian_6x6,
    double ratio_threshold  = 50.0,
    double min_ev_threshold = 10.0)
{
    DegeneracyResult result;

    // ---- 平移 Hessian 分析 ----
    // 提取左上角 3x3 块: 平移参数 [tx, ty, tz] 的二阶导
    const Eigen::Matrix3d H_trans = hessian_6x6.block<3, 3>(0, 0);

    // 自伴随矩阵特征值分解 (Hessian 理论上半正定, 数值上可能有微负)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_trans(H_trans);
    // cwiseAbs: 对数值噪声的鲁棒处理
    result.trans_eigenvalues  = solver_trans.eigenvalues().cwiseAbs();
    result.trans_eigenvectors = solver_trans.eigenvectors();

    const double max_ev_t = result.trans_eigenvalues.maxCoeff();
    result.trans_eigenvalues.minCoeff(&result.trans_min_idx);
    result.trans_lambda_min = result.trans_eigenvalues(result.trans_min_idx);
    result.trans_degen_dir  = result.trans_eigenvectors.col(result.trans_min_idx);

    // 条件数: lambda_max / lambda_min, 表征约束的各向异性程度
    result.trans_condition_ratio =
        (result.trans_lambda_min > 1e-12) ? (max_ev_t / result.trans_lambda_min) : 1e6;

    result.trans_degenerated =
        (result.trans_condition_ratio > ratio_threshold) ||
        (result.trans_lambda_min < min_ev_threshold);

    // ---- 旋转 Hessian 分析 ----
    // 提取右下角 3x3 块: 旋转参数 [rx, ry, rz] 的二阶导
    const Eigen::Matrix3d H_rot = hessian_6x6.block<3, 3>(3, 3);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_rot(H_rot);
    result.rot_eigenvalues  = solver_rot.eigenvalues().cwiseAbs();
    result.rot_eigenvectors = solver_rot.eigenvectors();

    const double max_ev_r = result.rot_eigenvalues.maxCoeff();
    result.rot_eigenvalues.minCoeff(&result.rot_min_idx);
    result.rot_lambda_min = result.rot_eigenvalues(result.rot_min_idx);
    result.rot_degen_dir  = result.rot_eigenvectors.col(result.rot_min_idx);

    result.rot_condition_ratio =
        (result.rot_lambda_min > 1e-12) ? (max_ev_r / result.rot_lambda_min) : 1e6;

    result.rot_degenerated =
        (result.rot_condition_ratio > ratio_threshold) ||
        (result.rot_lambda_min < min_ev_threshold);

    // ---- 综合 ----
    result.is_degenerated = result.trans_degenerated || result.rot_degenerated;

    return result;
}

}  // namespace hikari::loclite

#endif
