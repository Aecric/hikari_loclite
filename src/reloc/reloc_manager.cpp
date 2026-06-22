#include "reloc/reloc_manager.hpp"
#include "log.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace hikari::loclite {

bool RelocManager::Init(const std::string& yaml_path, const std::string& map_dir) {
    try {
        auto yaml = YAML::LoadFile(yaml_path);
        if (yaml["reloc"]) {
            auto reloc = yaml["reloc"];

            // backend 选择器 (默认 kiss): kiss=KISS-Matcher 全局配准, sc=旧 Scan Context 回退.
            const std::string backend_str = reloc["reloc_backend"].as<std::string>("kiss");
            if (backend_str == "sc") {
                backend_ = RelocBackend::Sc;
            } else {
                backend_ = RelocBackend::Kiss;
                if (backend_str != "kiss") {
                    LOG(WARNING) << "[RelocManager] unknown reloc_backend '" << backend_str
                                 << "', defaulting to kiss";
                }
            }

            auto_on_init_ = reloc["auto_on_init"].as<bool>(true);
            auto_on_lost_ = reloc["auto_on_lost"].as<bool>(true);
            disable_after_good_ = reloc["disable_after_good"].as<bool>(true);
            sc_enabled_ = reloc["sc_enabled"].as<bool>(true);
            sc_top_k_ = reloc["sc_top_k"].as<int>(1);
            const double legacy_cooldown_sec = reloc["sc_cooldown_sec"].as<double>(5.0);
            reloc_cooldown_sec_ = reloc["reloc_cooldown_sec"].as<double>(legacy_cooldown_sec);
            max_runtime_sec_ = reloc["max_runtime_sec"].as<double>(10.0);
            gravity_roll_thres_deg_ = reloc["gravity_roll_thres_deg"].as<double>(30.0);
            gravity_pitch_thres_deg_ = reloc["gravity_pitch_thres_deg"].as<double>(30.0);

            // 查询点云滚动累积: 单帧太稀, 倾斜安装下 KISS/SC 查询都不稳定. 两 backend 共用累积.
            // 新键 query_accum_*; 旧键 sc_accum_* 保留兼容读取 (旧键存在时作为后备默认).
            const int legacy_accum_frames = reloc["sc_accum_frames"].as<int>(20);
            const double legacy_accum_voxel_leaf = reloc["sc_accum_voxel_leaf"].as<double>(0.1);
            const double legacy_accum_max_rel_trans_m =
                reloc["sc_accum_max_rel_trans_m"].as<double>(1.0);
            query_accum_frames_ = reloc["query_accum_frames"].as<int>(legacy_accum_frames);
            query_accum_voxel_leaf_ =
                reloc["query_accum_voxel_leaf"].as<double>(legacy_accum_voxel_leaf);
            // 累积发散守门 (C3): LOST 后 LIO 速度积分跑飞时相对位姿不可用于拼接
            query_accum_max_rel_trans_m_ =
                reloc["query_accum_max_rel_trans_m"].as<double>(legacy_accum_max_rel_trans_m);

            // 候选 NDT 验证专用 delta 门限 (C1): ndt.max_delta_* 按 /initialpose 量纲设定,
            // 对 SC/KISS 候选过紧 (sector 6° 量化 + 关键帧间距; KISS yaw 微扫起点偏移). 两 backend 共用.
            // 新键 reloc_max_delta_*; 旧键 sc_max_delta_* 保留兼容读取 (旧键存在时作为后备默认).
            const double legacy_dt = reloc["sc_max_delta_trans_m"].as<double>(2.0);
            const double legacy_dr = reloc["sc_max_delta_rot_deg"].as<double>(15.0);
            reloc_max_delta_trans_m_ = reloc["reloc_max_delta_trans_m"].as<double>(legacy_dt);
            reloc_max_delta_rot_deg_ = reloc["reloc_max_delta_rot_deg"].as<double>(legacy_dr);

            // KISS-Matcher 参数 (R4): backend=kiss 时填 KissGlobalMatcher::Config.
            KissGlobalMatcher::Config kiss_cfg;
            kiss_cfg.kiss_voxel_size = reloc["kiss_voxel_size"].as<float>(0.3f);
            kiss_cfg.target_pre_voxel = reloc["target_pre_voxel"].as<double>(0.2);
            kiss_cfg.max_target_pts = reloc["max_target_pts"].as<size_t>(800000);
            kiss_cfg.min_rotation_inliers = reloc["min_rotation_inliers"].as<size_t>(50);
            kiss_cfg.min_final_inliers = reloc["min_final_inliers"].as<size_t>(30);
            kiss_ = KissGlobalMatcher(kiss_cfg);
            // yaw 微扫范围/步长 (退化场景保险丝)
            yaw_refine_range_deg_ = reloc["yaw_refine_range_deg"].as<double>(9.0);
            yaw_refine_step_deg_ = reloc["yaw_refine_step_deg"].as<double>(3.0);

            // SC database and poses: prefer map_dir, fallback to explicit YAML paths
            std::string sc_db_path = reloc["sc_database"].as<std::string>("");
            std::string poses_path = reloc["poses_txt"].as<std::string>("");

            if (!map_dir.empty()) {
                if (sc_db_path.empty()) sc_db_path = map_dir + "/sc_database.bin";
                if (poses_path.empty()) poses_path = map_dir + "/poses.txt";
            }

            // backend=kiss: 不加载 SC database / poses.txt (省 IO/CPU); KISS target 由节点 SetKissTarget 注入.
            if (backend_ == RelocBackend::Kiss) {
                LOG(INFO) << "[RelocManager] backend=kiss: SC database/poses NOT loaded "
                             "(KISS target will be injected by node)";
            } else {
                // backend=sc: 维持现有 SC 加载流程.
                if (!sc_enabled_) {
                    LOG(INFO) << "[RelocManager] SC disabled by config";
                    return true;
                }

                // Load poses first (needed to map kf_id -> map pose)
                if (!poses_path.empty()) {
                    if (!LoadPoses(poses_path)) {
                        LOG(WARNING) << "[RelocManager] failed to load poses: " << poses_path
                                     << " (SC candidates will have no map pose)";
                    }
                } else {
                    LOG(WARNING) << "[RelocManager] no poses_txt configured";
                }

                // Load SC database
                if (!sc_db_path.empty()) {
                    ScanContextManager::Options sc_options;
                    // Load SC options from yaml if present
                    if (reloc["sc_pc_num_ring"]) sc_options.pc_num_ring = reloc["sc_pc_num_ring"].as<int>(20);
                    if (reloc["sc_pc_num_sector"]) sc_options.pc_num_sector = reloc["sc_pc_num_sector"].as<int>(60);
                    if (reloc["sc_dist_thres"]) sc_options.sc_dist_thres = reloc["sc_dist_thres"].as<double>(0.13);
                    if (reloc["sc_pc_max_radius"]) sc_options.pc_max_radius = reloc["sc_pc_max_radius"].as<double>(80.0);
                    if (reloc["sc_lidar_height"]) sc_options.lidar_height = reloc["sc_lidar_height"].as<double>(2.0);
                    if (reloc["sc_num_candidates_from_tree"])
                        sc_options.num_candidates_from_tree = reloc["sc_num_candidates_from_tree"].as<int>(10);

                    sc_manager_.SetOptions(sc_options);
                    if (!sc_manager_.LoadDatabase(sc_db_path)) {
                        LOG(WARNING) << "[RelocManager] failed to load SC database: " << sc_db_path
                                     << " (SC relocalization unavailable)";
                        sc_enabled_ = false;
                    } else {
                        LOG(INFO) << "[RelocManager] SC database loaded: " << sc_db_path
                                  << " (" << sc_manager_.GetScanContextCount() << " entries)";
                    }
                } else {
                    LOG(WARNING) << "[RelocManager] no sc_database path configured";
                    sc_enabled_ = false;
                }
            }
        }
    } catch (const std::exception& e) {
        LOG(ERROR) << "[RelocManager] failed to parse config: " << e.what();
        return false;
    }

    LOG(INFO) << "[RelocManager] init: backend=" << (backend_ == RelocBackend::Kiss ? "kiss" : "sc")
              << ", auto_on_init=" << auto_on_init_
              << ", auto_on_lost=" << auto_on_lost_
              << ", disable_after_good=" << disable_after_good_
              << ", sc_enabled=" << sc_enabled_
              << ", sc_top_k=" << sc_top_k_
              << ", reloc_cooldown_sec=" << reloc_cooldown_sec_
              << ", query_accum_frames=" << query_accum_frames_
              << ", query_accum_voxel_leaf=" << query_accum_voxel_leaf_
              << ", query_accum_max_rel_trans_m=" << query_accum_max_rel_trans_m_
              << ", reloc_max_delta=" << reloc_max_delta_trans_m_ << "m/" << reloc_max_delta_rot_deg_ << "deg"
              << ", yaw_refine=±" << yaw_refine_range_deg_ << "@" << yaw_refine_step_deg_ << "deg"
              << ", poses=" << kf_poses_.size();
    return true;
}

void RelocManager::SetKissTarget(const CloudPtr& map_cloud) { kiss_.SetTarget(map_cloud); }

bool RelocManager::LoadPoses(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        LOG(ERROR) << "[RelocManager] cannot open poses file: " << path;
        return false;
    }

    kf_poses_.clear();
    std::string line;
    int loaded = 0;
    while (std::getline(ifs, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        int kf_id;
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        if (!(iss >> kf_id >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            continue;
        }

        Quatd q(qw, qx, qy, qz);
        q.normalize();
        SE3 pose(SO3(q), Vec3d(tx, ty, tz));
        kf_poses_[kf_id] = pose;
        ++loaded;
    }

    LOG(INFO) << "[RelocManager] loaded " << loaded << " keyframe poses from " << path;
    return loaded > 0;
}

void RelocManager::Arm(const char* reason, double current_time) {
    armed_.store(true, std::memory_order_release);
    reason_ = reason ? reason : "";
    arm_ts_ = current_time;
    // 重新 arm 即重新累积: 旧缓冲的 LIO 位姿域可能已断裂 (历经 Good 校正 / 重置)
    ClearAccumulation();
}

void RelocManager::Disarm(const char* reason) {
    armed_.store(false, std::memory_order_release);
    reason_ = reason ? reason : "";
    arm_ts_ = -1.0;
    // disarm/Good 时清空缓冲, 不在非 armed 期间持有点云副本
    ClearAccumulation();
}

void RelocManager::AccumulateScan(const CloudPtr& scan, const SE3& lidar_pose) {
    // 累积缓冲两 backend 共用 (KISS query 与 SC query 都用累积 + 重力对齐云).
    if (!RelocReady() || query_accum_frames_ <= 1) return;
    if (!scan || scan->empty()) return;

    // 发散守门 (C3): LOST 后 LIO 速度积分可能跑飞 (实测每帧位移 ~5m), 此时"短时局部自洽"
    // 假设失效, 按相对位姿拼接会糊掉查询云. 与上一入队帧相对平移超限则不入队;
    // 不清空缓冲 — 连续超限说明 LIO 已不可用于拼接, 保留旧帧, 环形 pop 自然换血.
    if (!accum_buffer_.empty() && query_accum_max_rel_trans_m_ > 0.0) {
        const double rel_trans =
            (accum_buffer_.back().pose.inverse() * lidar_pose).translation().norm();
        if (rel_trans > query_accum_max_rel_trans_m_) {
            ++accum_reject_count_;
            // 节流: 每个发散片段首帧 + 之后每 20 帧打一条
            if (accum_reject_count_ == 1 || accum_reject_count_ % 20 == 0) {
                LOG(WARNING) << "[RelocManager] accum frame rejected: rel_trans=" << rel_trans
                             << " m > query_accum_max_rel_trans_m=" << query_accum_max_rel_trans_m_
                             << " m (LIO likely diverged, rejected=" << accum_reject_count_
                             << " consecutive)";
            }
            return;
        }
        accum_reject_count_ = 0;
    }

    // 每帧开销仅为降采样 + 入队 (嵌入式预算内); 合并/对齐/查询在重定位尝试节奏发生
    AccumFrame frame;
    frame.pose = lidar_pose;
    frame.cloud.reset(new PointCloudType());
    if (query_accum_voxel_leaf_ > 0.0) {
        pcl::VoxelGrid<PointType> voxel;
        const float leaf = static_cast<float>(query_accum_voxel_leaf_);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.setInputCloud(scan);
        voxel.filter(*frame.cloud);
    } else {
        *frame.cloud = *scan;
    }

    accum_buffer_.push_back(std::move(frame));
    while (static_cast<int>(accum_buffer_.size()) > query_accum_frames_) {
        accum_buffer_.pop_front();
    }
}

void RelocManager::ClearAccumulation() {
    accum_buffer_.clear();
    accum_reject_count_ = 0;
}

CloudPtr RelocManager::BuildAccumulatedQueryCloud() const {
    CloudPtr merged(new PointCloudType());
    if (accum_buffer_.empty()) return merged;

    // 统一到最新帧 body 系: p_latest = (T_w_latest^-1 * T_w_i) * p_i.
    // LIO 全局可能漂移, 但短时局部自洽, 相对位姿可用于拼接.
    const SE3 T_latest_inv = accum_buffer_.back().pose.inverse();
    for (const auto& f : accum_buffer_) {
        const SE3 T_rel = T_latest_inv * f.pose;
        CloudPtr transformed(new PointCloudType());
        pcl::transformPointCloud(*f.cloud, *transformed, T_rel.matrix().cast<float>());
        *merged += *transformed;
    }

    if (query_accum_voxel_leaf_ <= 0.0) return merged;
    // 合并后再降采样一次, 控制 QueryTopK 输入规模
    CloudPtr filtered(new PointCloudType());
    pcl::VoxelGrid<PointType> voxel;
    const float leaf = static_cast<float>(query_accum_voxel_leaf_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(merged);
    voxel.filter(*filtered);
    return filtered;
}

CloudPtr RelocManager::GravityAlignCloud(const CloudPtr& cloud, const SE3& current_imu_pose) const {
    // 重力对齐 (公式同 lightning localization.cpp TryScanContextRelocalization):
    // 从 R_world_body 提取 yaw, R_body_level = R_yaw_only^T * R_world_body,
    // 把点云从倾斜 body 系旋转到水平 (level) 系, 与建库侧 (水平系点云) 描述子域一致.
    const Mat3d R_world_body = current_imu_pose.so3().matrix();
    const double yaw = std::atan2(R_world_body(1, 0), R_world_body(0, 0));
    const Mat3d R_yaw_only = Eigen::AngleAxisd(yaw, Vec3d::UnitZ()).toRotationMatrix();
    const Mat3d R_body_level = R_yaw_only.transpose() * R_world_body;

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = R_body_level.cast<float>();
    CloudPtr aligned(new PointCloudType());
    pcl::transformPointCloud(*cloud, *aligned, T);
    return aligned;
}

SE3 RelocManager::KfIdToMapPose(int kf_id, float yaw_diff_rad, const SE3& current_imu_pose) const {
    auto it = kf_poses_.find(kf_id);
    if (it == kf_poses_.end()) {
        return SE3();
    }

    // The keyframe pose is T_map_lidar at the keyframe.
    // SC gives yaw_diff_rad: the yaw rotation to align current scan to the keyframe.
    // We want the current scan to be placed at the keyframe location with the yaw correction applied.
    //
    // Strategy: take the keyframe's map position and yaw, apply the SC yaw correction,
    // then use current LIO roll/pitch for gravity compensation.
    const SE3& kf_pose = it->second;
    const Vec3d kf_pos = kf_pose.translation();

    // Extract keyframe yaw
    const Mat3d R_kf = kf_pose.so3().matrix();
    const double kf_yaw = std::atan2(R_kf(1, 0), R_kf(0, 0));

    // Apply SC yaw correction
    const double corrected_yaw = kf_yaw + static_cast<double>(yaw_diff_rad);

    // Extract current roll/pitch from LIO (gravity-aligned)
    const Mat3d R_current = current_imu_pose.so3().matrix();
    const double current_roll = std::atan2(R_current(2, 1), R_current(2, 2));
    const double current_pitch = std::asin(-std::clamp(R_current(2, 0), -1.0, 1.0));

    // Construct T_map_lidar with corrected yaw + current roll/pitch
    const Eigen::AngleAxisd yaw_aa(corrected_yaw, Vec3d::UnitZ());
    const Eigen::AngleAxisd pitch_aa(current_pitch, Vec3d::UnitY());
    const Eigen::AngleAxisd roll_aa(current_roll, Vec3d::UnitX());
    const Mat3d R_map = (yaw_aa * pitch_aa * roll_aa).toRotationMatrix();

    return SE3(SO3(R_map), kf_pos);
}

RelocCandidate RelocManager::TryRelocalize(const CloudPtr& scan, const SE3& current_imu_pose,
                                           double current_time, NdtCorrector* ndt) {
    if (!Armed()) return {};
    if (!RelocReady()) return {};
    if (!scan || scan->empty()) return {};

    // Check max runtime: disarm if relocalization has been trying too long
    if (max_runtime_sec_ > 0.0 && arm_ts_ > 0.0 && current_time > 0.0 &&
        current_time - arm_ts_ > max_runtime_sec_) {
        LOG(WARNING) << "[RelocManager] max runtime exceeded (" << max_runtime_sec_ << " s), disarming";
        Disarm("max_runtime_exceeded");
        return {};
    }

    if (backend_ == RelocBackend::Kiss) {
        return RunKissOnce(scan, current_imu_pose, false, ndt);
    }
    return RunScanContextOnce(scan, current_imu_pose, false);
}

RelocCandidate RelocManager::ManualRelocalize(const CloudPtr& scan, const SE3& current_imu_pose,
                                              NdtCorrector* ndt) {
    if (!RelocReady()) {
        LOG(WARNING) << "[RelocManager] manual relocalize: backend not ready ("
                     << (backend_ == RelocBackend::Kiss ? "KISS target missing" : "SC not enabled") << ")";
        return {};
    }
    if (!scan || scan->empty()) {
        LOG(WARNING) << "[RelocManager] manual relocalize: empty scan";
        return {};
    }

    LOG(INFO) << "[RelocManager] manual relocalize requested (bypasses blackout/cooldown), backend="
              << (backend_ == RelocBackend::Kiss ? "kiss" : "sc");
    if (backend_ == RelocBackend::Kiss) {
        return RunKissOnce(scan, current_imu_pose, true, ndt);
    }
    return RunScanContextOnce(scan, current_imu_pose, true);
}

RelocCandidate RelocManager::RunScanContextOnce(const CloudPtr& scan, const SE3& current_imu_pose, bool manual) {
    RelocCandidate result;
    debug_ = ScDebugInfo{};
    last_query_cloud_.reset();

    // Step 1: 构造查询点云 — 优先滚动累积合并 (单帧太稀, 倾斜安装下 SC 描述子不稳定)
    CloudPtr query_cloud;
    const int accum_frames = AccumulatedFrames();
    if (query_accum_frames_ > 1) {
        if (accum_frames >= query_accum_frames_) {
            query_cloud = BuildAccumulatedQueryCloud();
        } else if (manual) {
            // 手动触发不等累积: 用户显式授权, 缓冲不足时退回单帧立即尝试
            LOG(WARNING) << "[SC-Reloc] manual SC with insufficient accumulation (frames=" << accum_frames
                         << "/" << query_accum_frames_ << "), falling back to single scan";
            query_cloud = scan;
        } else {
            // 帧数不足: 跳过本次尝试, 等下个 cooldown 周期 (缓冲由逐帧 AccumulateScan 继续填充)
            LOG(INFO) << "[SC-Reloc] accumulating frames=" << accum_frames << "/" << query_accum_frames_
                      << ", skipping this attempt";
            return result;
        }
    } else {
        query_cloud = scan;
    }
    if (!query_cloud || query_cloud->empty()) {
        LOG(WARNING) << "[SC-Reloc] SC query cloud empty, skip";
        return result;
    }

    // Step 2: 重力对齐到水平 (level) 系后再查询, 与建库域一致
    CloudPtr cloud_level = GravityAlignCloud(query_cloud, current_imu_pose);
    last_query_cloud_ = cloud_level;

    LOG(INFO) << "[SC-Reloc] SC query: accum_frames=" << accum_frames
              << ", query_points=" << cloud_level->size()
              << ", top_k=" << sc_top_k_ << (manual ? " (manual)" : "");

    auto candidates = sc_manager_.QueryTopK(cloud_level, sc_top_k_);
    debug_.candidates = candidates;

    if (candidates.empty()) {
        LOG(INFO) << "[RelocManager] SC query: no candidates found";
        return result;
    }

    // Try candidates in order (best SC distance first, already sorted)
    for (const auto& c : candidates) {
        if (c.kf_id < 0) continue;

        RelocCandidate cand;
        cand.kf_id = c.kf_id;
        cand.yaw_diff_rad = c.yaw_diff_rad;
        cand.score = c.sc_dist;
        cand.source = manual ? "manual_sc" : "sc";

        // Gravity check: compare SC candidate orientation with current LIO
        cand.pose = KfIdToMapPose(c.kf_id, c.yaw_diff_rad, current_imu_pose);

        // Compute gravity alignment error
        const Mat3d R_cand = cand.pose.so3().matrix();
        const Mat3d R_curr = current_imu_pose.so3().matrix();
        const double cand_roll = std::atan2(R_cand(2, 1), R_cand(2, 2));
        const double cand_pitch = std::asin(-std::clamp(R_cand(2, 0), -1.0, 1.0));
        const double curr_roll = std::atan2(R_curr(2, 1), R_curr(2, 2));
        const double curr_pitch = std::asin(-std::clamp(R_curr(2, 0), -1.0, 1.0));

        const float roll_err = static_cast<float>(std::abs(cand_roll - curr_roll) * 180.0 / M_PI);
        const float pitch_err = static_cast<float>(std::abs(cand_pitch - curr_pitch) * 180.0 / M_PI);
        const bool gravity_ok = (roll_err <= gravity_roll_thres_deg_) && (pitch_err <= gravity_pitch_thres_deg_);

        // Store gravity check for the first (best) candidate
        if (&c == &candidates[0]) {
            debug_.roll_err_deg = roll_err;
            debug_.pitch_err_deg = pitch_err;
            debug_.gravity_passed = gravity_ok;
        }

        if (!gravity_ok) {
            LOG(WARNING) << "[RelocManager] SC candidate kf_id=" << c.kf_id
                         << " gravity check failed: roll_err=" << roll_err
                         << " deg, pitch_err=" << pitch_err << " deg";
            continue;
        }

        cand.valid = true;
        result = cand;
        debug_.best = cand;

        LOG(INFO) << "[RelocManager] SC candidate: kf_id=" << c.kf_id
                  << ", sc_dist=" << c.sc_dist
                  << ", yaw_diff=" << c.yaw_diff_rad * 180.0 / M_PI << " deg"
                  << ", pos=[" << cand.pose.translation().x() << ", "
                  << cand.pose.translation().y() << ", "
                  << cand.pose.translation().z() << "]"
                  << (manual ? " (manual)" : "");
        break;  // Take the first valid candidate
    }

    if (!result.valid) {
        LOG(INFO) << "[RelocManager] SC: no valid candidate after gravity check ("
                  << candidates.size() << " candidates)";
    }

    return result;
}

CloudPtr RelocManager::PrepareKissQueryLevel(const CloudPtr& scan, const SE3& current_imu_pose,
                                             bool manual) {
    // 异步 KISS 的「主线程」一半: 读 accum_buffer_ (与逐帧 AccumulateScan 同锁) 构造 level 系查询云快照.
    // 真正的重活 (MatchGlobal + yaw 微扫) 交给 MatchKissOnSnapshot 在工作线程跑, 不碰这里的共享缓冲.
    debug_ = ScDebugInfo{};
    last_query_cloud_.reset();

    if (!kiss_.TargetReady()) {
        LOG_EVERY_T(WARNING, 5.0) << "[KISS-Reloc] target not ready (SetKissTarget first), skip";
        return nullptr;
    }

    // Step 1: 构造查询点云 — 复用共享滚动累积 (单帧太稀, 倾斜安装下特征不稳定)
    CloudPtr query_cloud;
    const int accum_frames = AccumulatedFrames();
    if (query_accum_frames_ > 1) {
        if (accum_frames >= query_accum_frames_) {
            query_cloud = BuildAccumulatedQueryCloud();
        } else if (manual) {
            LOG(WARNING) << "[KISS-Reloc] manual KISS with insufficient accumulation (frames=" << accum_frames
                         << "/" << query_accum_frames_ << "), falling back to single scan";
            query_cloud = scan;
        } else {
            LOG(INFO) << "[KISS-Reloc] accumulating frames=" << accum_frames << "/" << query_accum_frames_
                      << ", skipping this attempt";
            return nullptr;
        }
    } else {
        query_cloud = scan;
    }
    if (!query_cloud || query_cloud->empty()) {
        LOG(WARNING) << "[KISS-Reloc] query cloud empty, skip";
        return nullptr;
    }

    // Step 2: 重力对齐到水平 (level) 系 (去 roll/pitch 倾斜); KISS target 是固定地图 (map 系),
    // KISS 全局配准本身无需初值, 输出 T_map_query_level. 与 SC 路径同样的 level 化 query.
    CloudPtr cloud_level = GravityAlignCloud(query_cloud, current_imu_pose);
    last_query_cloud_ = cloud_level;

    LOG(INFO) << "[KISS-Reloc] query prepared: accum_frames=" << accum_frames
              << ", query_points=" << cloud_level->size() << (manual ? " (manual)" : "");
    return cloud_level;
}

RelocCandidate RelocManager::RunKissOnce(const CloudPtr& scan, const SE3& current_imu_pose, bool manual,
                                         NdtCorrector* ndt) {
    // 同步组合 (保留供 TryRelocalize/ManualRelocalize 的旧调用面); 在线/离线节点改走
    // PrepareKissQueryLevel + 工作线程 MatchKissOnSnapshot 的异步路径, 不在 spin 线程上阻塞.
    CloudPtr query_level = PrepareKissQueryLevel(scan, current_imu_pose, manual);
    if (!query_level) return {};
    return MatchKissOnSnapshot(query_level, scan, current_imu_pose, ndt, manual);
}

RelocCandidate RelocManager::MatchKissOnSnapshot(const CloudPtr& query_level, const CloudPtr& scan,
                                                 const SE3& current_imu_pose, NdtCorrector* ndt,
                                                 bool manual) const {
    // 异步 KISS 的「worker 线程」一半: 只读 kiss_/yaw_refine_*/reloc_max_delta_*, 用 caller 独占的 ndt 做
    // yaw 微扫 Validate, 不写任何成员. 配合"至多一个在飞 worker + 独立 ndt 实例"约束, 工作线程调用安全.
    RelocCandidate result;
    result.kf_id = -1;
    if (!query_level || query_level->empty()) return result;
    if (!ndt) {
        LOG(ERROR) << "[KISS-Reloc] NDT corrector null, cannot refine/validate, skip";
        return result;
    }

    // Step 3: KISS estimate (无需初值的粗 6DOF) + inlier 闸 (在 wrapper 内做)
    const auto kiss_res = kiss_.MatchGlobal(query_level);
    if (!kiss_res.valid) {
        LOG(INFO) << "[KISS-Reloc] KISS estimate invalid (no match / inliers below gate)";
        return result;
    }
    // KISS 解 T_map_level 把 level 系 query 对齐到 map (p_map = T_map_level * p_level).
    // 但 NDT 验证喂的是原始去畸变 scan (lidar/body 系), 而 query 是 cloud_level = R_body_level * scan_body.
    // 故需把 T_map_level 复合回 lidar 系: T_map_lidar = T_map_level * SE3(R_body_level, 0).
    // R_body_level 取自 GravityAlignCloud 同款公式 (用同一 current_imu_pose, 保证域一致).
    const Mat3d R_world_body = current_imu_pose.so3().matrix();
    const double yaw_wb = std::atan2(R_world_body(1, 0), R_world_body(0, 0));
    const Mat3d R_yaw_only = Eigen::AngleAxisd(yaw_wb, Vec3d::UnitZ()).toRotationMatrix();
    const Mat3d R_body_level = R_yaw_only.transpose() * R_world_body;
    const SE3 T_level_lidar(SO3(R_body_level), Vec3d::Zero());
    const SE3 T_map_lidar = kiss_res.pose * T_level_lidar;
    LOG(INFO) << "[KISS-Reloc] coarse 6DOF (T_map_lidar): pos=[" << T_map_lidar.translation().x() << ", "
              << T_map_lidar.translation().y() << ", " << T_map_lidar.translation().z()
              << "], rot_inl=" << kiss_res.rotation_inliers << ", final_inl=" << kiss_res.final_inliers;

    // Step 4: yaw 微扫 (退化场景保险丝) — 在 KISS 粗解 yaw 上叠加 [-range,+range]@step,
    // 逐起点构 SE3 调 NdtCorrector::Validate (既精修也是候选二次闸), 取 valid && TP 最高的.
    // 验证用的 scan 是当前帧去畸变云 (lidar 系), 与 SC 路径一致 (NDT Validate 内部做 lidar->map 配准).
    NdtResult best_ndt;
    bool found = false;
    const int n_step = (yaw_refine_step_deg_ > 0.0)
                           ? static_cast<int>(std::round(yaw_refine_range_deg_ / yaw_refine_step_deg_))
                           : 0;
    for (int i = -n_step; i <= n_step; ++i) {
        const double d_rad = static_cast<double>(i) * yaw_refine_step_deg_ * M_PI / 180.0;
        // 在 map 系 z 轴 (重力轴) 上叠加 yaw 偏移: T = Rz(d) * T_map_lidar (绕地图原点 z 旋转位置+朝向)
        const SE3 T_yaw(SO3(Eigen::AngleAxisd(d_rad, Vec3d::UnitZ()).toRotationMatrix()), Vec3d::Zero());
        const SE3 guess = T_yaw * T_map_lidar;

        const NdtResult ndt_res =
            ndt->Validate(guess, scan, reloc_max_delta_trans_m_, reloc_max_delta_rot_deg_);
        if (!ndt_res.valid) continue;
        if (!found || ndt_res.confidence > best_ndt.confidence) {
            best_ndt = ndt_res;
            found = true;
        }
    }

    if (!found) {
        LOG(INFO) << "[KISS-Reloc] yaw refine: no NDT-valid pose across ±" << yaw_refine_range_deg_
                  << "@" << yaw_refine_step_deg_ << " deg sweep (KISS coarse rejected)";
        return result;
    }

    // 接受: pose = 最佳 NDT 解 (yaw 微扫的 NDT 已是精修 + 二次闸).
    result.valid = true;
    result.pose = best_ndt.pose;
    result.score = static_cast<double>(kiss_res.final_inliers);
    result.kf_id = -1;
    result.source = manual ? "manual_kiss" : "kiss";

    LOG(INFO) << "[KISS-Reloc] accepted: pos=[" << result.pose.translation().x() << ", "
              << result.pose.translation().y() << ", " << result.pose.translation().z()
              << "], TP=" << best_ndt.confidence << ", final_inl=" << kiss_res.final_inliers
              << (manual ? " (manual)" : "");
    return result;
}

SE3 RelocManager::KfPose(int kf_id) const {
    auto it = kf_poses_.find(kf_id);
    if (it == kf_poses_.end()) return SE3();
    return it->second;
}

}  // namespace hikari::loclite
