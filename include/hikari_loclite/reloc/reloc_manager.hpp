#pragma once

#ifndef HIKARI_LOCLITE_RELOC_MANAGER_HPP
#define HIKARI_LOCLITE_RELOC_MANAGER_HPP

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/eigen_types.h"
#include "common/point_def.h"
#include "lio/scan_context.h"

namespace hikari::loclite {

/// SC 重定位候选.
struct RelocCandidate {
    bool valid = false;
    SE3 pose;           // T_map_lidar (gravity-compensated)
    double score = 0.0; // SC distance (lower is better)
    int kf_id = -1;
    float yaw_diff_rad = 0.0f;
    std::string source; // "sc" or "manual_sc"
};

/// SC debug info for publishing.
struct ScDebugInfo {
    std::vector<ScanContextManager::Candidate> candidates;
    RelocCandidate best;
    float roll_err_deg = 0.0f;
    float pitch_err_deg = 0.0f;
    bool gravity_passed = false;
};

/// 轻量重定位管理器: bounded SC query for init / LOST recovery.
/// Good 态 disarm, 不运行后台线程.
class RelocManager {
   public:
    using Ptr = std::shared_ptr<RelocManager>;

    bool Init(const std::string& yaml_path, const std::string& map_dir);

    /// Arm with a wall-clock timestamp for runtime limit enforcement.
    void Arm(const char* reason, double current_time = -1.0);
    void Disarm(const char* reason);
    bool Armed() const { return armed_.load(std::memory_order_acquire); }
    const char* ArmReason() const { return reason_.c_str(); }

    bool AutoOnInit() const { return auto_on_init_; }
    bool AutoOnLost() const { return auto_on_lost_; }
    bool ScEnabled() const { return sc_enabled_; }
    double ScCooldownSec() const { return sc_cooldown_sec_; }
    double MaxRuntimeSec() const { return max_runtime_sec_; }
    bool DisableAfterGood() const { return disable_after_good_; }

    /// SC 候选验证专用 delta 门限 (传给 NdtCorrector::Validate 覆盖 ndt.max_delta_*):
    /// SC sector 分辨率 360/60=6° 量化误差 + 关键帧间距使正确候选 delta 常超 ndt.* 门限.
    double ScMaxDeltaTransM() const { return sc_max_delta_trans_m_; }
    double ScMaxDeltaRotDeg() const { return sc_max_delta_rot_deg_; }

    /// Try one bounded SC query. Returns invalid candidate if disarmed, no scan,
    /// on cooldown, exceeded max runtime, or no valid match found. Does NOT do NDT validation.
    /// @param current_time  wall-clock seconds for runtime limit check (pass node->now().seconds())
    RelocCandidate TryRelocalize(const CloudPtr& scan, const SE3& current_imu_pose, double current_time);

    /// Request a manual SC attempt (bypasses blackout, cooldown, and runtime limit).
    RelocCandidate ManualRelocalize(const CloudPtr& scan, const SE3& current_imu_pose);

    /// armed 期间逐帧调用: 体素降采样后的 deskewed scan (lidar 系) + 对应 LIO lidar 位姿入环形缓冲.
    /// 每帧开销仅为降采样 + 入队; 合并/重力对齐/查询只在 SC 尝试节奏 (cooldown) 内发生.
    void AccumulateScan(const CloudPtr& scan, const SE3& lidar_pose);

    /// 清空滚动累积缓冲. Arm/Disarm 内部已调用; LIO ResetToMapPose 后相对位姿断裂时也需调用.
    void ClearAccumulation();
    int AccumulatedFrames() const { return static_cast<int>(accum_buffer_.size()); }
    int ScAccumFrames() const { return sc_accum_frames_; }

    /// 最近一次 SC 查询实际使用的点云 (level 系, 合并 + 重力对齐后), 供 debug 发布; 可能为空.
    CloudPtr LastQueryCloud() const { return last_query_cloud_; }

    /// Get debug info from the last query (for publishing SC debug topics).
    const ScDebugInfo& LastDebugInfo() const { return debug_; }

    /// Look up the map-frame pose for a keyframe ID. Returns identity if not found.
    SE3 KfPose(int kf_id) const;

   private:
    bool LoadPoses(const std::string& path);
    SE3 KfIdToMapPose(int kf_id, float yaw_diff_rad, const SE3& current_imu_pose) const;
    RelocCandidate RunScanContextOnce(const CloudPtr& scan, const SE3& current_imu_pose, bool manual);
    /// 把缓冲内各帧用相对 LIO 位姿统一到最新帧 body 系, 合并 + 体素降采样.
    CloudPtr BuildAccumulatedQueryCloud() const;
    /// 重力对齐: 从 R_world_body 提取 yaw, R_body_level = R_yaw_only^T * R_world_body,
    /// 把查询点云旋转到水平 (level) 系 (公式同 lightning TryScanContextRelocalization).
    CloudPtr GravityAlignCloud(const CloudPtr& cloud, const SE3& current_imu_pose) const;

    /// 滚动累积单帧: 降采样副本 + 对应 LIO 位姿.
    struct AccumFrame {
        CloudPtr cloud;  // 体素降采样后的 deskewed scan (lidar/body 系)
        SE3 pose;        // T_map_lidar (LIO; 全局可能漂移, 短时局部自洽, 仅用其相对位姿拼接)
    };

    ScanContextManager sc_manager_;
    std::unordered_map<int, SE3> kf_poses_;  // kf_id -> T_map_lidar

    std::atomic<bool> armed_{false};
    std::string reason_;
    double arm_ts_ = -1.0;  // wall-clock seconds when armed, for max_runtime_sec_ check
    bool auto_on_init_ = true;
    bool auto_on_lost_ = true;
    bool disable_after_good_ = true;
    bool sc_enabled_ = true;
    int sc_top_k_ = 1;
    double sc_cooldown_sec_ = 5.0;
    double max_runtime_sec_ = 10.0;

    // Gravity check thresholds
    double gravity_roll_thres_deg_ = 30.0;
    double gravity_pitch_thres_deg_ = 30.0;

    // SC 候选验证专用 delta 门限 (仅 SC 用, /initialpose 验证仍走 ndt.max_delta_*)
    double sc_max_delta_trans_m_ = 2.0;
    double sc_max_delta_rot_deg_ = 15.0;

    // SC 查询点云滚动累积 (armed 期间逐帧维护; 调用方与节点主链路同锁, 无需额外加锁)
    std::deque<AccumFrame> accum_buffer_;
    int sc_accum_frames_ = 20;          // 查询前所需累积帧数; <=1 时退化为单帧查询
    double sc_accum_voxel_leaf_ = 0.1;  // 入队前 + 合并后体素降采样 leaf (m); <=0 关闭降采样
    // 发散守门: 与上一入队帧相对平移超此值不入队 (LIO 发散时相对位姿拼接会糊掉查询云); <=0 关闭
    double sc_accum_max_rel_trans_m_ = 1.0;
    int accum_reject_count_ = 0;        // 连续被发散守门拒绝的帧数 (节流日志 + 入队成功时清零)
    CloudPtr last_query_cloud_;         // 最近一次实际送入 QueryTopK 的点云 (level 系)

    ScDebugInfo debug_;
};

}  // namespace hikari::loclite

#endif
