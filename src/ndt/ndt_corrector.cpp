#include "ndt/ndt_corrector.hpp"
#include "log.h"

#include <yaml-cpp/yaml.h>

#include <cmath>

namespace hikari::loclite {

namespace {

/// 降采样后源点数下限: 低于该值说明 scan 过稀 (盲区/遮挡/空旷), NDT 结果不可信
constexpr int kMinSourcePoints = 50;

/// NDT 输出的 4x4 矩阵转 SE3: 旋转块可能有数值误差, 经四元数归一化后再构造
SE3 Mat4fToSE3(const Eigen::Matrix4f& m) {
    const Mat4d md = m.cast<double>();
    Eigen::Quaterniond q(md.block<3, 3>(0, 0));
    q.normalize();
    return SE3(q, Vec3d(md.block<3, 1>(0, 3)));
}

}  // namespace

bool NdtCorrector::Init(const std::string& yaml_path) {
    try {
        auto yaml = YAML::LoadFile(yaml_path);
        if (yaml["ndt"]) {
            auto ndt = yaml["ndt"];
            threads_ = ndt["threads"].as<int>(1);
            resolution_ = ndt["resolution"].as<double>(1.0);
            source_leaf_ = ndt["source_leaf"].as<double>(0.5);
            // min_confidence 是 TP (TransformationProbability) 量纲.
            // 默认 1.5 标定依据: lightning lidar_loc.h tracking_conf_thres_=1.5 /
            // default_livox.yaml multi_scale_bad_conf_thres=1.5, 真匹配 TP 通常落在 [1.5, 5].
            min_confidence_ = ndt["min_confidence"].as<double>(1.5);
            max_delta_trans_m_ = ndt["max_delta_trans_m"].as<double>(1.0);
            max_delta_rot_deg_ = ndt["max_delta_rot_deg"].as<double>(10.0);
        }
    } catch (...) {
        LOG(ERROR) << "NdtCorrector: failed to load ndt config from " << yaml_path;
        return false;
    }
    voxel_source_.setLeafSize(source_leaf_, source_leaf_, source_leaf_);
    LOG(INFO) << "NdtCorrector: resolution=" << resolution_ << ", threads=" << threads_
              << ", source_leaf=" << source_leaf_ << ", min_confidence(TP)=" << min_confidence_
              << ", max_delta=" << max_delta_trans_m_ << "m/" << max_delta_rot_deg_ << "deg";
    return true;
}

NdtCorrector::NdtType::Ptr NdtCorrector::MakeConfiguredNdt() const {
    // 配准参数照 lightning lidar_loc.cc:1743 的 local_ndt (step=0.1, eps=0.01, iter<=10, DIRECT7).
    // threads 默认 1: 嵌入式上避免与 LIO 主链路抢核 (lightning P1 setNumThreads(1) 同款考量).
    NdtType::Ptr ndt(new NdtType());
    ndt->setResolution(static_cast<float>(resolution_));
    ndt->setNumThreads(threads_);
    ndt->setStepSize(0.1);
    ndt->setTransformationEpsilon(0.01);
    ndt->setMaximumIterations(10);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    return ndt;
}

bool NdtCorrector::SetMap(const CloudPtr& map) {
    if (!map || map->empty()) {
        LOG(ERROR) << "NdtCorrector: SetMap called with empty map";
        return false;
    }
    // 每次换图都重建实例: 本包 pclomp 的 setInputTarget 被改为空实现, target 通过
    // AddTarget + ComputeTargetGrids 一次性累积建格 (ComputeTargetGrids 后再 AddTarget
    // 会污染均值/协方差), 新实例保证无残留. 体素协方差只在这里建一次, 后续 Align 复用.
    // 构造时 force_no_recompute_=true, align 内不会对 target 重建 KdTree, 无额外开销.
    ndt_ = MakeConfiguredNdt();
    ndt_->AddTarget(map);
    ndt_->ComputeTargetGrids();
    LOG(INFO) << "NdtCorrector: map set with " << map->size() << " points, target voxel grids built";
    return true;
}

NdtResult NdtCorrector::Align(const CloudPtr& scan, const SE3& guess) {
    NdtResult result;
    if (!scan || scan->empty()) {
        return result;
    }
    if (!ndt_) {
        LOG(ERROR) << "NdtCorrector: Align called before SetMap";
        return result;
    }

    // 源点云降采样: scan 为 lidar 系去畸变点云, 0.5m leaf 控制单次 Align 的 CPU 预算
    CloudPtr filtered(new PointCloudType);
    voxel_source_.setInputCloud(scan);
    voxel_source_.filter(*filtered);
    if (static_cast<int>(filtered->size()) < kMinSourcePoints) {
        LOG(WARNING) << "NdtCorrector: too few source points after downsample (" << filtered->size()
                     << " < " << kMinSourcePoints << "), skip align";
        return result;
    }

    ndt_->setInputSource(filtered);
    CloudPtr aligned(new PointCloudType);
    ndt_->align(*aligned, guess.matrix().cast<float>());

    if (!ndt_->hasConverged()) {
        return result;
    }

    result.pose = Mat4fToSE3(ndt_->getFinalTransformation());
    result.confidence = ndt_->getTransformationProbability();
    const SE3 delta = guess.inverse() * result.pose;
    result.delta_trans_m = delta.translation().norm();
    result.delta_rot_deg = delta.so3().log().norm() * 180.0 / M_PI;
    // Align 的 valid 仅表示"收敛且有输出"; TP/delta 门限在 Validate (或调用方) 判定
    result.valid = true;
    return result;
}

NdtResult NdtCorrector::Validate(const SE3& candidate_pose, const CloudPtr& scan) {
    NdtResult result = Align(scan, candidate_pose);
    if (!result.valid) {
        LOG(WARNING) << "NdtCorrector: validate rejected (ndt not converged, or empty/sparse scan)";
        return result;
    }

    // 门限判定 (用户拍板: 仅 TP + delta, 覆盖率指标留作后续扩展).
    // 拒绝日志带 TP/delta 数值, 便于现场标定阈值.
    const bool conf_ok = result.confidence >= min_confidence_;
    const bool delta_ok =
        result.delta_trans_m <= max_delta_trans_m_ && result.delta_rot_deg <= max_delta_rot_deg_;
    if (!conf_ok || !delta_ok) {
        LOG(WARNING) << "NdtCorrector: validate rejected: TP=" << result.confidence
                     << " (min=" << min_confidence_ << "), delta=" << result.delta_trans_m << "m/"
                     << result.delta_rot_deg << "deg (max=" << max_delta_trans_m_ << "m/"
                     << max_delta_rot_deg_ << "deg)";
        result.valid = false;
        return result;
    }

    LOG(INFO) << "NdtCorrector: validate accepted: TP=" << result.confidence
              << ", delta=" << result.delta_trans_m << "m/" << result.delta_rot_deg << "deg";
    return result;
}

}  // namespace hikari::loclite
