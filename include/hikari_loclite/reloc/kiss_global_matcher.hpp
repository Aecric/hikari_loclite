#pragma once

#ifndef HIKARI_LOCLITE_KISS_GLOBAL_MATCHER_HPP
#define HIKARI_LOCLITE_KISS_GLOBAL_MATCHER_HPP

#include <cstddef>
#include <vector>

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace hikari::loclite {

/// KISS-Matcher (MIT-SPARK) 全局点云配准封装 (无需初值的粗 6DOF 重定位).
///
/// 用途: 冷启动 / 手动重定位时, 把累积+重力对齐后的查询云直接对固定地图做全局配准,
/// 输出一个 T_map_query 粗解. 跟 SC ring-key 极坐标判别相比, KISS 用真实 3D 特征对应 +
/// GNC 鲁棒求解, 走廊等对称场景不会退化成 SC 那种"收向原点"伪命中.
///
/// 整类被 USE_KISS_MATCHER 守卫: 未编 KISS 时 MatchGlobal 始终返回无效结果 (降级),
/// 不破坏构建. 本类是纯库 (无 rclcpp 句柄), 日志走 log.h 的 LOG() shim.
///
/// 移植自 lightning src/core/localization/kiss_matcher_wrapper.{h,cc} 的 RunKiss 内核;
/// 差异: 命名空间 hikari::loclite; 不自加载 global.pcd (target 由调用方喂固定图);
/// 只保留整图 MatchGlobal (小图无需 crop).
class KissGlobalMatcher {
   public:
    struct Config {
        // KISS-Matcher 主体素 (FPFH 内部下采样). 0.3m 是论文/默认推荐.
        float kiss_voxel_size = 0.3f;

        // target (固定地图) 缓存前的预下采样 leaf (m); <=0 关掉预下采样.
        double target_pre_voxel = 0.2;

        // target 点数护栏: 预下采样后仍 > 此值则标记 target 不可用并告警 (大图需分块, 暂不支持).
        size_t max_target_pts = 800000;

        // GNC rotation inliers 下限 — 低于此值视为失败.
        size_t min_rotation_inliers = 50;

        // COTE 最终 (translation) inliers 下限.
        size_t min_final_inliers = 30;
    };

    struct KissResult {
        bool valid = false;
        SE3 pose;  // T_map_query (粗 6DOF)
        size_t rotation_inliers = 0;
        size_t final_inliers = 0;
    };

    KissGlobalMatcher() = default;
    explicit KissGlobalMatcher(Config cfg);

    /// 设置/复用 target 固定地图 (典型: FastLioFixedMap::FixedMapCloud()).
    /// 按 target_pre_voxel 体素降采样后转为 vector<Vec3f> 缓存; 点数超 max_target_pts
    /// 则告警并标记 target 不可用. 重复调用会替换缓存.
    void SetTarget(const CloudPtr& map_cloud);

    /// target 是否已就绪 (已 SetTarget 且通过点数护栏).
    bool TargetReady() const { return target_ready_; }
    size_t TargetPts() const { return target_vec_.size(); }

    /// 整图全局配准: query (已重力对齐的累积云) 对缓存 target 跑 KISS estimate + inlier 闸.
    /// 返回 KissResult{valid=false} 表示 target 未就绪 / query 空 / solver 无解 / inliers
    /// 不达标 / 未编 KISS. 不修改任何外部状态.
    KissResult MatchGlobal(const CloudPtr& query_level) const;

    const Config& GetConfig() const { return config_; }

   private:
    Config config_;
    std::vector<Eigen::Vector3f> target_vec_;  // 预下采样后的固定地图 (map 系)
    bool target_ready_ = false;
};

}  // namespace hikari::loclite

#endif  // HIKARI_LOCLITE_KISS_GLOBAL_MATCHER_HPP
