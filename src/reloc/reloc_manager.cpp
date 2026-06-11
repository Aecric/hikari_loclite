#include "reloc/reloc_manager.hpp"
#include "log.h"

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
            auto_on_init_ = reloc["auto_on_init"].as<bool>(true);
            auto_on_lost_ = reloc["auto_on_lost"].as<bool>(true);
            disable_after_good_ = reloc["disable_after_good"].as<bool>(true);
            sc_enabled_ = reloc["sc_enabled"].as<bool>(true);
            sc_top_k_ = reloc["sc_top_k"].as<int>(1);
            sc_cooldown_sec_ = reloc["sc_cooldown_sec"].as<double>(5.0);
            max_runtime_sec_ = reloc["max_runtime_sec"].as<double>(10.0);
            gravity_roll_thres_deg_ = reloc["gravity_roll_thres_deg"].as<double>(30.0);
            gravity_pitch_thres_deg_ = reloc["gravity_pitch_thres_deg"].as<double>(30.0);

            // SC database and poses: prefer map_dir, fallback to explicit YAML paths
            std::string sc_db_path = reloc["sc_database"].as<std::string>("");
            std::string poses_path = reloc["poses_txt"].as<std::string>("");

            if (!map_dir.empty()) {
                if (sc_db_path.empty()) sc_db_path = map_dir + "/sc_database.bin";
                if (poses_path.empty()) poses_path = map_dir + "/poses.txt";
            }

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
    } catch (const std::exception& e) {
        LOG(ERROR) << "[RelocManager] failed to parse config: " << e.what();
        return false;
    }

    LOG(INFO) << "[RelocManager] init: auto_on_init=" << auto_on_init_
              << ", auto_on_lost=" << auto_on_lost_
              << ", disable_after_good=" << disable_after_good_
              << ", sc_enabled=" << sc_enabled_
              << ", sc_top_k=" << sc_top_k_
              << ", sc_cooldown_sec=" << sc_cooldown_sec_
              << ", poses=" << kf_poses_.size();
    return true;
}

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
}

void RelocManager::Disarm(const char* reason) {
    armed_.store(false, std::memory_order_release);
    reason_ = reason ? reason : "";
    arm_ts_ = -1.0;
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

RelocCandidate RelocManager::TryRelocalize(const CloudPtr& scan, const SE3& current_imu_pose, double current_time) {
    if (!Armed()) return {};
    if (!sc_enabled_) return {};
    if (!scan || scan->empty()) return {};

    // Check max runtime: disarm if SC has been trying too long
    if (max_runtime_sec_ > 0.0 && arm_ts_ > 0.0 && current_time > 0.0 &&
        current_time - arm_ts_ > max_runtime_sec_) {
        LOG(WARNING) << "[RelocManager] SC max runtime exceeded (" << max_runtime_sec_ << " s), disarming";
        Disarm("max_runtime_exceeded");
        return {};
    }

    return RunScanContextOnce(scan, current_imu_pose, false);
}

RelocCandidate RelocManager::ManualRelocalize(const CloudPtr& scan, const SE3& current_imu_pose) {
    if (!sc_enabled_) {
        LOG(WARNING) << "[RelocManager] manual SC relocalize: SC not enabled";
        return {};
    }
    if (!scan || scan->empty()) {
        LOG(WARNING) << "[RelocManager] manual SC relocalize: empty scan";
        return {};
    }

    LOG(INFO) << "[RelocManager] manual SC relocalize requested (bypasses blackout/cooldown)";
    return RunScanContextOnce(scan, current_imu_pose, true);
}

RelocCandidate RelocManager::RunScanContextOnce(const CloudPtr& scan, const SE3& current_imu_pose, bool manual) {
    RelocCandidate result;
    debug_ = ScDebugInfo{};

    auto candidates = sc_manager_.QueryTopK(scan, sc_top_k_);
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

SE3 RelocManager::KfPose(int kf_id) const {
    auto it = kf_poses_.find(kf_id);
    if (it == kf_poses_.end()) return SE3();
    return it->second;
}

}  // namespace hikari::loclite
