#pragma once

#ifndef HIKARI_LOCLITE_RELOC_MANAGER_HPP
#define HIKARI_LOCLITE_RELOC_MANAGER_HPP

#include <atomic>
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

    /// Try one bounded SC query. Returns invalid candidate if disarmed, no scan,
    /// on cooldown, exceeded max runtime, or no valid match found. Does NOT do NDT validation.
    /// @param current_time  wall-clock seconds for runtime limit check (pass node->now().seconds())
    RelocCandidate TryRelocalize(const CloudPtr& scan, const SE3& current_imu_pose, double current_time);

    /// Request a manual SC attempt (bypasses blackout, cooldown, and runtime limit).
    RelocCandidate ManualRelocalize(const CloudPtr& scan, const SE3& current_imu_pose);

    /// Get debug info from the last query (for publishing SC debug topics).
    const ScDebugInfo& LastDebugInfo() const { return debug_; }

    /// Look up the map-frame pose for a keyframe ID. Returns identity if not found.
    SE3 KfPose(int kf_id) const;

   private:
    bool LoadPoses(const std::string& path);
    SE3 KfIdToMapPose(int kf_id, float yaw_diff_rad, const SE3& current_imu_pose) const;
    RelocCandidate RunScanContextOnce(const CloudPtr& scan, const SE3& current_imu_pose, bool manual);

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

    ScDebugInfo debug_;
};

}  // namespace hikari::loclite

#endif
