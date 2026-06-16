#include "reloc/kiss_global_matcher.hpp"

#include "log.h"

#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <exception>
#include <utility>

#ifdef USE_KISS_MATCHER
#include "kiss_matcher/KISSMatcher.hpp"
#endif

namespace hikari::loclite {

KissGlobalMatcher::KissGlobalMatcher(Config cfg) : config_(std::move(cfg)) {
    LOG(INFO) << "[KISS] init: voxel=" << config_.kiss_voxel_size
              << "m, tgt_pre_voxel=" << config_.target_pre_voxel
              << "m, max_tgt_pts=" << config_.max_target_pts
              << ", min_rot_inl=" << config_.min_rotation_inliers
              << ", min_final_inl=" << config_.min_final_inliers;
}

void KissGlobalMatcher::SetTarget(const CloudPtr& map_cloud) {
    target_vec_.clear();
    target_ready_ = false;

    if (!map_cloud || map_cloud->empty()) {
        LOG(WARNING) << "[KISS] SetTarget: map_cloud 空, target 不可用";
        return;
    }

    // 预下采样固定地图 (避免每次 estimate 都喂全分辨率点); <=0 关掉.
    CloudPtr ds;
    if (config_.target_pre_voxel > 0.0) {
        const float leaf = static_cast<float>(config_.target_pre_voxel);
        pcl::VoxelGrid<PointType> vg;
        vg.setLeafSize(leaf, leaf, leaf);
        vg.setInputCloud(map_cloud);
        ds = std::make_shared<PointCloudType>();
        vg.filter(*ds);
        LOG(INFO) << "[KISS] SetTarget 预下采样 leaf=" << config_.target_pre_voxel
                  << "m: " << map_cloud->size() << " -> " << ds->size() << " 点";
    } else {
        ds = map_cloud;
    }

    // 点数护栏: 预下采样后仍超 max_target_pts 直接拒收 (大图分块为后续 task).
    if (config_.max_target_pts > 0 && ds->size() > config_.max_target_pts) {
        LOG(WARNING) << "[KISS] SetTarget: target 点数 " << ds->size()
                     << " > max_target_pts " << config_.max_target_pts
                     << ", target 不可用 (大图需分块)";
        return;
    }

    // 转 vector<Vec3f>, 跳过非有限点.
    target_vec_.reserve(ds->size());
    for (const auto& p : ds->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        target_vec_.emplace_back(p.x, p.y, p.z);
    }

    if (target_vec_.size() < 1000) {
        LOG(WARNING) << "[KISS] SetTarget: 有效目标点 <1000 (" << target_vec_.size()
                     << "), target 不可用";
        target_vec_.clear();
        return;
    }

    target_ready_ = true;
    LOG(INFO) << "[KISS] SetTarget ready: " << target_vec_.size() << " 点";
}

KissGlobalMatcher::KissResult KissGlobalMatcher::MatchGlobal(
    const CloudPtr& query_level) const {
    KissResult result;

#ifndef USE_KISS_MATCHER
    (void)query_level;
    LOG_EVERY_N(WARNING, 20) << "[KISS] support was not built; returning no match";
    return result;
#else
    if (!target_ready_) {
        LOG(WARNING) << "[KISS] MatchGlobal: target 未就绪 (先 SetTarget)";
        return result;
    }
    if (!query_level || query_level->empty()) {
        LOG(WARNING) << "[KISS] MatchGlobal: query_level 空";
        return result;
    }

    // query 转 vector<Vec3f>, 跳过非有限点 (手抄循环, 照 lightning wrapper).
    std::vector<Eigen::Vector3f> src_vec;
    src_vec.reserve(query_level->size());
    for (const auto& p : query_level->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        src_vec.emplace_back(p.x, p.y, p.z);
    }
    if (src_vec.size() < 1000) {
        LOG(WARNING) << "[KISS] MatchGlobal: query 有效点 <1000 (" << src_vec.size() << ")";
        return result;
    }

    kiss_matcher::KISSMatcherConfig cfg(config_.kiss_voxel_size);
    kiss_matcher::KISSMatcher matcher(cfg);
    kiss_matcher::RegistrationSolution sol;
    try {
        sol = matcher.estimate(src_vec, target_vec_);
    } catch (const std::exception& e) {
        LOG(ERROR) << "[KISS] estimate 抛异常: " << e.what();
        return result;
    }

    result.rotation_inliers = matcher.getNumRotationInliers();
    result.final_inliers = matcher.getNumFinalInliers();

    if (!sol.valid) {
        LOG(WARNING) << "[KISS] solver invalid: rot_inl=" << result.rotation_inliers
                     << ", final_inl=" << result.final_inliers
                     << ", src=" << src_vec.size() << ", tgt=" << target_vec_.size();
        return result;
    }
    if (result.rotation_inliers < config_.min_rotation_inliers ||
        result.final_inliers < config_.min_final_inliers) {
        LOG(WARNING) << "[KISS] inliers 不达标: rot=" << result.rotation_inliers
                     << " (min=" << config_.min_rotation_inliers
                     << "), final=" << result.final_inliers
                     << " (min=" << config_.min_final_inliers
                     << "), src=" << src_vec.size() << ", tgt=" << target_vec_.size();
        return result;
    }

    // sol.rotation 是 raw Matrix3d, 可能非严格正交; 直接构 SO3 会触发 Sophus 断言.
    // 走 Quaternion 归一化路径 (照 lightning wrapper:195-198).
    Eigen::Quaterniond q(sol.rotation);
    q.normalize();
    result.pose = SE3(SO3(q), sol.translation);
    result.valid = true;

    LOG(INFO) << "[KISS] OK: t=[" << result.pose.translation().transpose() << "]"
              << ", rot_inl=" << result.rotation_inliers
              << ", final_inl=" << result.final_inliers
              << ", src=" << src_vec.size() << ", tgt=" << target_vec_.size();
    return result;
#endif
}

}  // namespace hikari::loclite
