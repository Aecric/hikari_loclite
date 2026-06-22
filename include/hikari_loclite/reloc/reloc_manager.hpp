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
#include "ndt/ndt_corrector.hpp"
#include "reloc/kiss_global_matcher.hpp"

namespace hikari::loclite {

/// 重定位后端选择 (reloc.reloc_backend): 默认 KISS-Matcher 全局配准,
/// SC (Scan Context) 保留可切回做 A/B / 回退.
enum class RelocBackend { Kiss, Sc };

/// 重定位候选 (两个 backend 共用; SC 专有字段对 KISS 无意义, 保持默认).
struct RelocCandidate {
    bool valid = false;
    SE3 pose;           // T_map_lidar (gravity-compensated; KISS 为 yaw 微扫后最佳 NDT 解)
    double score = 0.0; // SC: SC distance (lower is better); KISS: final_inliers (higher is better)
    int kf_id = -1;     // SC only; KISS 恒为 -1
    float yaw_diff_rad = 0.0f;  // SC only
    std::string source; // "sc" / "manual_sc" / "kiss" / "manual_kiss"
};

/// SC debug info for publishing.
struct ScDebugInfo {
    std::vector<ScanContextManager::Candidate> candidates;
    RelocCandidate best;
    float roll_err_deg = 0.0f;
    float pitch_err_deg = 0.0f;
    bool gravity_passed = false;
};

/// 轻量重定位管理器: bounded KISS/SC query for init / LOST recovery.
/// Good 态 disarm, 不运行常驻后台线程.
class RelocManager {
   public:
    using Ptr = std::shared_ptr<RelocManager>;

    bool Init(const std::string& yaml_path, const std::string& map_dir);

    /// Arm with a wall-clock timestamp for runtime limit enforcement.
    void Arm(const char* reason, double current_time = -1.0);
    void Disarm(const char* reason);
    bool Armed() const { return armed_.load(std::memory_order_acquire); }
    const char* ArmReason() const { return reason_.c_str(); }

    RelocBackend Backend() const { return backend_; }
    bool BackendIsKiss() const { return backend_ == RelocBackend::Kiss; }
    bool BackendIsSc() const { return backend_ == RelocBackend::Sc; }

    bool AutoOnInit() const { return auto_on_init_; }
    bool AutoOnLost() const { return auto_on_lost_; }
    bool ScEnabled() const { return sc_enabled_; }
    /// 当前 backend 是否可用于重定位 (backend=kiss → kiss target 就绪; backend=sc → SC 已启用).
    /// 节点用此 backend-agnostic 谓词替代旧的 ScEnabled() 守卫来 arm/accumulate/trigger.
    bool RelocReady() const {
        return backend_ == RelocBackend::Kiss ? kiss_.TargetReady() : sc_enabled_;
    }
    double RelocCooldownSec() const { return reloc_cooldown_sec_; }
    /// 兼容旧调用点命名 (语义同 RelocCooldownSec).
    double ScCooldownSec() const { return RelocCooldownSec(); }
    double MaxRuntimeSec() const { return max_runtime_sec_; }
    /// armed 已持续多久 (wall 秒); 未 armed 或无 arm_ts 返回 -1. 供异步 KISS 路径 (绕过 TryRelocalize
    /// 内置的 max_runtime 检查) 自行判定是否超 MaxRuntimeSec 而 Disarm.
    double ArmedElapsed(double current_time) const {
        return (Armed() && arm_ts_ > 0.0 && current_time > 0.0) ? current_time - arm_ts_ : -1.0;
    }
    bool DisableAfterGood() const { return disable_after_good_; }

    /// 候选 NDT 验证专用 delta 门限 (传给 NdtCorrector::Validate 覆盖 ndt.max_delta_*):
    /// SC sector 分辨率 360/60=6° 量化误差 + 关键帧间距使正确候选 delta 常超 ndt.* 门限;
    /// KISS yaw 微扫起点与最终 NDT 解之间同样需放宽 delta. 两 backend 共用.
    double RelocMaxDeltaTransM() const { return reloc_max_delta_trans_m_; }
    double RelocMaxDeltaRotDeg() const { return reloc_max_delta_rot_deg_; }
    /// 兼容旧调用点命名 (语义同 RelocMaxDelta*).
    double ScMaxDeltaTransM() const { return reloc_max_delta_trans_m_; }
    double ScMaxDeltaRotDeg() const { return reloc_max_delta_rot_deg_; }

    /// 注入 KISS target 固定地图 (节点在固定图加载后调用; 仅 backend=kiss 时有意义).
    /// 转调 KissGlobalMatcher::SetTarget (预降采样 + 点数护栏 + 缓存 vector<Vec3f>).
    void SetKissTarget(const CloudPtr& map_cloud);
    bool KissTargetReady() const { return kiss_.TargetReady(); }

    /// Try one bounded relocalization (按 backend 分派 KISS / SC). Returns invalid candidate
    /// if disarmed, no scan, on cooldown, exceeded max runtime, or no valid match found.
    /// KISS 路径内含 yaw 微扫 NDT 精修/验证 (需 ndt); SC 路径不做 NDT (节点侧再验).
    /// @param current_time  wall-clock seconds for runtime limit check (pass node->now().seconds())
    /// @param ndt           NDT 校正器 (KISS yaw 微扫复用其 Validate; SC backend 可传 nullptr)
    RelocCandidate TryRelocalize(const CloudPtr& scan, const SE3& current_imu_pose, double current_time,
                                 NdtCorrector* ndt);

    /// Request a manual relocalization attempt (bypasses blackout, cooldown, and runtime limit).
    RelocCandidate ManualRelocalize(const CloudPtr& scan, const SE3& current_imu_pose, NdtCorrector* ndt);

    /// 异步 KISS 重定位的「主线程」一半 (KISS backend 专用): 在调用方持锁的前提下读 accum_buffer_
    /// 构造 level 系查询点云快照 (合并 + 重力对齐), 顺带更新 last_query_cloud_ (debug 用).
    /// 返回 nullptr 表示帧数不足 (调用方据此不消耗 cooldown) 或查询云为空; manual=true 时帧数不足回退单帧.
    /// 与 MatchKissOnSnapshot 配对, 把读共享缓冲 (主线程) 与跑重活 (worker) 隔开.
    CloudPtr PrepareKissQueryLevel(const CloudPtr& scan, const SE3& current_imu_pose, bool manual);

    /// 异步 KISS 重定位的「worker 线程」一半 (KISS backend 专用): 在 PrepareKissQueryLevel 产出的
    /// query_level 快照上跑 KISS 全局配准 + yaw 微扫 (用 caller 独占的 ndt 实例做 Validate).
    /// **线程安全约定**: 只读 kiss_/yaw_refine_*/reloc_max_delta_* 等 init 后不变的成员, 不写任何成员;
    /// 配合"同一时刻至多一个在飞 worker + 独立 ndt 实例"的调用约束, 可安全地在工作线程调用.
    /// scan 为派发时快照的当前帧去畸变云 (yaw 微扫 NDT Validate 用).
    RelocCandidate MatchKissOnSnapshot(const CloudPtr& query_level, const CloudPtr& scan,
                                       const SE3& current_imu_pose, NdtCorrector* ndt, bool manual) const;

    /// armed 期间逐帧调用: 体素降采样后的 deskewed scan (lidar 系) + 对应 LIO lidar 位姿入环形缓冲.
    /// 每帧开销仅为降采样 + 入队; 合并/重力对齐/查询只在重定位尝试节奏 (cooldown) 内发生.
    void AccumulateScan(const CloudPtr& scan, const SE3& lidar_pose);

    /// 清空滚动累积缓冲. Arm/Disarm 内部已调用; LIO ResetToMapPose 后相对位姿断裂时也需调用.
    void ClearAccumulation();
    int AccumulatedFrames() const { return static_cast<int>(accum_buffer_.size()); }
    int QueryAccumFrames() const { return query_accum_frames_; }
    /// 兼容旧调用点命名 (语义同 QueryAccumFrames).
    int ScAccumFrames() const { return QueryAccumFrames(); }

    /// 最近一次 KISS/SC 查询实际使用的点云 (level 系, 合并 + 重力对齐后), 供 debug 发布; 可能为空.
    CloudPtr LastQueryCloud() const { return last_query_cloud_; }

    /// Get debug info from the last query (for publishing SC debug topics).
    const ScDebugInfo& LastDebugInfo() const { return debug_; }

    /// Look up the map-frame pose for a keyframe ID. Returns identity if not found.
    SE3 KfPose(int kf_id) const;

   private:
    bool LoadPoses(const std::string& path);
    SE3 KfIdToMapPose(int kf_id, float yaw_diff_rad, const SE3& current_imu_pose) const;
    RelocCandidate RunScanContextOnce(const CloudPtr& scan, const SE3& current_imu_pose, bool manual);
    /// KISS 全局配准重定位: 累积 query → 重力对齐 → KISS estimate (无需初值的粗 6DOF) →
    /// inlier 闸 → yaw 微扫 (±range@step, 逐起点 NdtCorrector::Validate, 取 TP 最高的有效解).
    /// yaw 微扫的 NDT 既是精修也是候选二次闸 (与 SC 路径一致的接受逻辑). 返回的 pose 是最佳 NDT 解.
    RelocCandidate RunKissOnce(const CloudPtr& scan, const SE3& current_imu_pose, bool manual,
                               NdtCorrector* ndt);
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
    KissGlobalMatcher kiss_;                 // KISS-Matcher 全局配准 (默认 backend)

    RelocBackend backend_ = RelocBackend::Kiss;  // reloc.reloc_backend (默认 kiss)

    std::atomic<bool> armed_{false};
    std::string reason_;
    double arm_ts_ = -1.0;  // wall-clock seconds when armed, for max_runtime_sec_ check
    bool auto_on_init_ = true;
    bool auto_on_lost_ = true;
    bool disable_after_good_ = true;
    bool sc_enabled_ = true;
    int sc_top_k_ = 1;
    double reloc_cooldown_sec_ = 5.0;
    double max_runtime_sec_ = 10.0;

    // Gravity check thresholds (SC backend only)
    double gravity_roll_thres_deg_ = 30.0;
    double gravity_pitch_thres_deg_ = 30.0;

    // 候选 NDT 验证专用 delta 门限 (两 backend 共用, /initialpose 验证仍走 ndt.max_delta_*)
    double reloc_max_delta_trans_m_ = 2.0;
    double reloc_max_delta_rot_deg_ = 15.0;

    // KISS yaw 微扫 (退化场景保险丝): KISS 粗解 yaw 上叠加 [-range, +range] @ step, 逐起点
    // NDT 验证取 TP 最高的有效解. ±9°@3° = {-9,-6,-3,0,3,6,9}.
    double yaw_refine_range_deg_ = 9.0;
    double yaw_refine_step_deg_ = 3.0;

    // KISS/SC 查询点云滚动累积 (armed 期间逐帧维护; 调用方与节点主链路同锁, 无需额外加锁)
    std::deque<AccumFrame> accum_buffer_;
    int query_accum_frames_ = 20;          // 查询前所需累积帧数; <=1 时退化为单帧查询
    double query_accum_voxel_leaf_ = 0.1;  // 入队前 + 合并后体素降采样 leaf (m); <=0 关闭降采样
    // 发散守门: 与上一入队帧相对平移超此值不入队 (LIO 发散时相对位姿拼接会糊掉查询云); <=0 关闭
    double query_accum_max_rel_trans_m_ = 1.0;
    int accum_reject_count_ = 0;        // 连续被发散守门拒绝的帧数 (节流日志 + 入队成功时清零)
    CloudPtr last_query_cloud_;         // 最近一次实际送入 backend 查询的点云 (level 系)

    ScDebugInfo debug_;
};

}  // namespace hikari::loclite

#endif
