#pragma once

#ifndef HIKARI_LOCLITE_NDT_CORRECTOR_HPP
#define HIKARI_LOCLITE_NDT_CORRECTOR_HPP

#include <memory>
#include <string>

#include <pcl/filters/voxel_grid.h>

#include "common/eigen_types.h"
#include "common/point_def.h"
#include "ndt/ndt_omp.h"

namespace hikari::loclite {

struct NdtResult {
    bool valid = false;
    SE3 pose;
    /// pclomp getTransformationProbability() (TP): "已匹配点的平均 Gaussian 密度".
    /// 量纲参考 lightning (default_livox.yaml:136): 真匹配通常落在 [1.5, 5], >6 多为致密几何伪匹配.
    double confidence = 0.0;
    double inlier_ratio = 0.0;  // 预留: 覆盖率指标 (用户拍板本版不做, 恒 0)
    double delta_trans_m = 0.0;  // 收敛位姿相对 guess 的平移差 (m)
    double delta_rot_deg = 0.0;  // 收敛位姿相对 guess 的旋转差 (deg)
};

/// NDT 校正/验证器: 低频漂移校正 (Good 态) 与外部候选位姿验证 (/initialpose, 后续 SC).
///
/// 线程安全说明: 内部复用同一 pclomp NDT 实例 (target 体素协方差只在 SetMap 建一次, 后续
/// Align 复用), align 过程会改写实例内部状态, 因此 Align/Validate 均为非 const 且非线程安全.
/// 调用方 (LocLiteNode) 已在 mutex_ 内串行调用, 本类不持有内部锁, 也不开线程.
class NdtCorrector {
   public:
    using Ptr = std::shared_ptr<NdtCorrector>;
    using NdtType = pclomp::NormalDistributionsTransform<PointType, PointType>;

    bool Init(const std::string& yaml_path);

    /// 保存 target 点云并一次性建体素协方差 (fixed_map 已按 fixed_map.voxel_leaf 降采样).
    /// 再次调用会整体重建 (本包 pclomp 的 AddTarget/ComputeTargetGrids 是一次性累积式建格).
    bool SetMap(const CloudPtr& map);

    /// 以 guess 为初值配准 (scan 为 lidar 系去畸变点云, 内部按 ndt.source_leaf 降采样).
    /// valid 仅表示"收敛且有输出"; TP / delta 门限判定在 Validate (或由调用方自行把关).
    NdtResult Align(const CloudPtr& scan, const SE3& guess);

    /// 候选位姿验证: Align(scan, candidate) 后做门限判定,
    /// valid = 收敛 且 TP >= ndt.min_confidence 且 delta <= max_delta_trans_m / max_delta_rot_deg.
    NdtResult Validate(const SE3& candidate_pose, const CloudPtr& scan);

    /// TP 下限 (供调用方在 Good 态校正等场景复用同一口径)
    double MinConfidence() const { return min_confidence_; }

   private:
    /// 按当前参数新建并配置 pclomp 实例 (SetMap 时调用, 保证 target 无残留)
    NdtType::Ptr MakeConfiguredNdt() const;

    int threads_ = 1;
    double resolution_ = 1.0;
    double source_leaf_ = 0.5;      // ndt.source_leaf: 源点云降采样 leaf (m)
    double min_confidence_ = 1.5;   // ndt.min_confidence: TP 下限 (标定依据见 yaml 注释)
    double max_delta_trans_m_ = 1.0;
    double max_delta_rot_deg_ = 10.0;

    NdtType::Ptr ndt_;  // 复用实例; 体素协方差只在 SetMap 建一次
    pcl::VoxelGrid<PointType> voxel_source_;
};

}  // namespace hikari::loclite

#endif
