#include "lio/fast_lio_fixed_map.hpp"
#include "math/loclite_math.hpp"
#include "log.h"

#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <execution>

namespace hikari::loclite {

bool FastLioFixedMap::Init(const std::string& yaml_path) {
    LOG(INFO) << "FastLioFixedMap: init from " << yaml_path;

    preprocess_ = std::make_shared<PointCloudPreprocess>();
    p_imu_ = std::make_shared<ImuProcess>();

    try {
        auto yaml = YAML::LoadFile(yaml_path);

        // Lidar type
        int lidar_type = yaml["fast_lio"]["lidar_type"].as<int>(1);
        if (lidar_type == 1) preprocess_->SetLidarType(LidarType::AVIA);
        else if (lidar_type == 2) preprocess_->SetLidarType(LidarType::VELO32);
        else if (lidar_type == 3) preprocess_->SetLidarType(LidarType::OUST64);
        else if (lidar_type == 4) preprocess_->SetLidarType(LidarType::ROBOSENSE);

        preprocess_->Blind() = yaml["fast_lio"]["blind"].as<double>(0.5);
        preprocess_->TimeScale() = yaml["fast_lio"]["time_scale"].as<double>(1e-3);
        preprocess_->PointFilterNum() = yaml["fast_lio"]["point_filter_num"].as<int>(1);
        preprocess_->NumScans() = yaml["fast_lio"]["scan_line"].as<int>(6);
        if (yaml["fast_lio"]["max_range"]) {
            preprocess_->MaxRange() = yaml["fast_lio"]["max_range"].as<double>();
        }

        // Extrinsic
        const std::vector<double> default_extrin_t{0, 0, 0};
        const std::vector<double> default_extrin_r{1, 0, 0, 0, 1, 0, 0, 0, 1};
        auto extrinT = yaml["fast_lio"]["extrinsic_T"].as<std::vector<double>>(default_extrin_t);
        auto extrinR = yaml["fast_lio"]["extrinsic_R"].as<std::vector<double>>(default_extrin_r);
        offset_t_lidar_fixed_ = math::VecFromArray<double>(extrinT);
        offset_R_lidar_fixed_ = math::MatFromArray<double>(extrinR);

        // IMU
        double gyr_cov = yaml["fast_lio"]["gyr_cov"].as<double>(0.01);
        double acc_cov = yaml["fast_lio"]["acc_cov"].as<double>(0.01);
        double b_gyr_cov = yaml["fast_lio"]["b_gyr_cov"].as<double>(0.0001);
        double b_acc_cov = yaml["fast_lio"]["b_acc_cov"].as<double>(0.0001);
        bool use_imu_filter = yaml["fast_lio"]["imu_filter"].as<bool>(false);
        p_imu_->SetExtrinsic(offset_t_lidar_fixed_, offset_R_lidar_fixed_);
        p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
        p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
        p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));
        p_imu_->SetUseIMUFilter(use_imu_filter);

        // iVox
        ivox_options_.resolution_ = yaml["fast_lio"]["ivox_grid_resolution"].as<float>(0.2);
        int nearby_type = yaml["fast_lio"]["ivox_nearby_type"].as<int>(18);
        if (nearby_type == 0) ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
        else if (nearby_type == 6) ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
        else if (nearby_type == 18) ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        else if (nearby_type == 26) ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
        else ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;

        // Scan filter
        double filter_size_scan = yaml["fast_lio"]["filter_size_scan"].as<double>(0.2);
        filter_size_map_min_ = yaml["fast_lio"]["filter_size_map"].as<double>(0.2);
        min_pts_ = yaml["fast_lio"]["min_pts"].as<int>(300);
        esti_plane_threshold_ = yaml["fast_lio"]["esti_plane_threshold"].as<float>(fasterlio::ESTI_PLANE_THRESHOLD);
        max_lidar_residual_mean_ = yaml["fast_lio"]["max_lidar_residual_mean"].as<double>(0.05);
        voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

        // ZUPT 零速更新 (Phase 1): 默认值用 Phase 0 标定结果, 缺键走默认.
        auto fl = yaml["fast_lio"];
        zupt_enabled_ = fl["zupt_enabled"].as<bool>(true);
        zupt_vel_gate_ = fl["zupt_vel_gate"].as<double>(0.05);
        zupt_vel_cov_ = fl["zupt_vel_cov"].as<double>(1.0e-3);
        StaticDetector::Config sd_cfg;
        sd_cfg.gyro_std_enter_thres = fl["static_gyro_std_enter_thres"].as<double>(0.010);
        sd_cfg.gyro_std_exit_thres = fl["static_gyro_std_exit_thres"].as<double>(0.025);
        sd_cfg.acc_std_thres = fl["static_acc_std_thres"].as<double>(0.008);
        sd_cfg.window_sec = fl["static_window_sec"].as<double>(0.5);
        sd_cfg.park_enter_frames = fl["zupt_park_enter_frames"].as<int>(10);
        sd_cfg.park_exit_frames = fl["zupt_park_exit_frames"].as<int>(3);
        sd_cfg.warmup_frames = fl["zupt_warmup_frames"].as<int>(50);
        static_detector_.SetConfig(sd_cfg);
        LOG(INFO) << "FastLioFixedMap: ZUPT enabled=" << zupt_enabled_
                  << ", vel_gate=" << zupt_vel_gate_ << ", vel_cov=" << zupt_vel_cov_
                  << ", static[gyro_std enter<" << sd_cfg.gyro_std_enter_thres << " exit>"
                  << sd_cfg.gyro_std_exit_thres << ", acc_std<" << sd_cfg.acc_std_thres
                  << ", window=" << sd_cfg.window_sec << "s, enter=" << sd_cfg.park_enter_frames
                  << ", exit=" << sd_cfg.park_exit_frames << ", warmup=" << sd_cfg.warmup_frames << "]";

        // Fixed map config
        crop_radius_m_ = yaml["fixed_map"]["crop_radius_m"].as<double>(30.0);
        max_points_ = yaml["fixed_map"]["max_points"].as<int>(800000);

        if (yaml["roi"]) {
            const float height_max = yaml["roi"]["height_max"].as<float>(3.0f);
            const float height_min = yaml["roi"]["height_min"].as<float>(-3.0f);
            preprocess_->SetHeightROI(height_max, height_min);
            LOG(INFO) << "FastLioFixedMap: ROI height [" << height_min << ", " << height_max << "]";
        }

    } catch (const std::exception& e) {
        LOG(ERROR) << "FastLioFixedMap: failed to load config: " << e.what();
        return false;
    }

    // ESKF init
    ESKF::Options eskf_options;
    try {
        auto yaml = YAML::LoadFile(yaml_path);
        eskf_options.max_iterations_ = yaml["fast_lio"]["max_iteration"].as<int>(fasterlio::NUM_MAX_ITERATIONS);
    } catch (...) {
        eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    }
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, ESKF::state_dim_, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState& s, ESKF::CustomObservationModel& obs) { ObsModel(s, obs); };
    eskf_options.use_aa_ = false;
    kf_.Init(eskf_options);
    LOG(INFO) << "FastLioFixedMap: diagnostics enabled, max_lidar_residual_mean="
              << max_lidar_residual_mean_ << ", min_pts=" << min_pts_
              << ", esti_plane_threshold=" << esti_plane_threshold_;

    return true;
}

bool FastLioFixedMap::LoadFixedMapFromConfig(const std::string& yaml_path) {
    try {
        auto yaml = YAML::LoadFile(yaml_path);
        if (!yaml["fixed_map"] || !yaml["fixed_map"]["global_pcd"]) {
            LOG(ERROR) << "FastLioFixedMap: no fixed_map.global_pcd in config";
            return false;
        }
        std::string pcd_path = yaml["fixed_map"]["global_pcd"].as<std::string>();
        double voxel_leaf = yaml["fixed_map"]["voxel_leaf"].as<double>(0.2);
        return LoadFixedMap(pcd_path, voxel_leaf);
    } catch (const std::exception& e) {
        LOG(ERROR) << "FastLioFixedMap: failed to load map config: " << e.what();
        return false;
    }
}

bool FastLioFixedMap::LoadFixedMap(const std::string& pcd_path, double voxel_leaf) {
    LOG(INFO) << "FastLioFixedMap: loading map from " << pcd_path;

    CloudPtr raw(new PointCloudType);
    if (pcl::io::loadPCDFile<PointType>(pcd_path, *raw) != 0) {
        LOG(ERROR) << "FastLioFixedMap: failed to load PCD: " << pcd_path;
        return false;
    }

    LOG(INFO) << "FastLioFixedMap: raw map points: " << raw->size();

    // VoxelGrid filter
    CloudPtr filtered(new PointCloudType);
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
    voxel.setInputCloud(raw);
    voxel.filter(*filtered);

    // Limit max points
    if ((int)filtered->size() > max_points_) {
        LOG(WARNING) << "FastLioFixedMap: map too large (" << filtered->size() << "), limiting to " << max_points_;
        CloudPtr limited(new PointCloudType);
        limited->points.assign(filtered->points.begin(), filtered->points.begin() + max_points_);
        limited->width = limited->size();
        limited->height = 1;
        filtered = limited;
    }

    fixed_map_cloud_ = filtered;
    LOG(INFO) << "FastLioFixedMap: filtered map points: " << fixed_map_cloud_->size();

    // Build iVox from the full map (will be cropped on first tracking)
    fixed_ivox_ = std::make_shared<IVoxType>(ivox_options_);
    fixed_ivox_->AddPoints(fixed_map_cloud_->points);
    LOG(INFO) << "FastLioFixedMap: built iVox with " << fixed_ivox_->NumValidGrids() << " grids";

    return true;
}

bool FastLioFixedMap::RebuildLocalMapAround(const SE3& T_map_lidar) {
    if (!fixed_map_cloud_) return false;

    const Eigen::Vector3d center = T_map_lidar.translation();
    CloudPtr local(new PointCloudType);
    local->reserve(fixed_map_cloud_->size());

    const double r2 = crop_radius_m_ * crop_radius_m_;
    for (const auto& p : fixed_map_cloud_->points) {
        const Eigen::Vector3d q(p.x, p.y, p.z);
        if ((q - center).squaredNorm() <= r2) {
            local->push_back(p);
        }
    }

    IVoxType::Options opts = ivox_options_;
    fixed_ivox_ = std::make_shared<IVoxType>(opts);
    fixed_ivox_->AddPoints(local->points);

    LOG(INFO) << "FastLioFixedMap: rebuilt local map around [" << center.transpose()
              << "], points=" << local->size() << ", grids=" << fixed_ivox_->NumValidGrids();
    return true;
}

SE3 FastLioFixedMap::ImuPoseToLidarPose(const SE3& T_map_imu) const {
    const SE3 T_imu_lidar(SO3(offset_R_lidar_fixed_), offset_t_lidar_fixed_);
    return T_map_imu * T_imu_lidar;
}

SE3 FastLioFixedMap::LidarPoseToImuPose(const SE3& T_map_lidar) const {
    const SE3 T_imu_lidar(SO3(offset_R_lidar_fixed_), offset_t_lidar_fixed_);
    return T_map_lidar * T_imu_lidar.inverse();
}

bool FastLioFixedMap::ResetToMapPose(const SE3& T_map_lidar, bool reset_velocity) {
    LOG(INFO) << "FastLioFixedMap: reset to pose [" << T_map_lidar.translation().transpose() << "]";

    // Reset only the map pose. Keep IMU-estimated bias/gravity/time state so
    // propagation after relocalization does not restart with a wrong inertial model.
    NavState new_state = kf_.GetX();
    new_state.SetPose(LidarPoseToImuPose(T_map_lidar));
    if (reset_velocity) {
        new_state.vel_ = Vec3d::Zero();
    }
    kf_.ChangeX(new_state);

    // Reset covariance
    ESKF::CovType P = ESKF::CovType::Identity() * 0.01;
    kf_.ChangeP(P);

    state_point_ = new_state;

    // Rebuild local map around new pose
    RebuildLocalMapAround(T_map_lidar);
    tracking_enabled_ = true;

    // 速度/位姿域中断: 清空静止检测窗与迟滞计数, 重定位后从干净状态重新判定泊车态.
    static_detector_.Reset();

    // Do not reset the IMU processor here. ResetToMapPose is called after a
    // deskewed scan has already proven the candidate pose; forcing IMU init
    // again would freeze propagation for the init window and publish a static
    // pose. Keep the IMU time base and continue matching from the next frame.
    flg_first_scan_ = false;

    return true;
}

void FastLioFixedMap::AddImu(const sensor_msgs::msg::Imu::SharedPtr& msg) {
    auto imu = std::make_shared<IMU>();
    imu->timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    imu->linear_acceleration = Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if (imu->timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }
    last_timestamp_imu_ = imu->timestamp;
    imu_buffer_.emplace_back(imu);
}

void FastLioFixedMap::AddCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    CloudPtr cloud(new PointCloudType());
    preprocess_->Process(msg, cloud);
    lidar_buffer_.push_back(cloud);
    time_buffer_.push_back(timestamp);
    last_timestamp_lidar_ = timestamp;
}

void FastLioFixedMap::AddLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    CloudPtr cloud(new PointCloudType());
    preprocess_->Process(msg, cloud);
    lidar_buffer_.push_back(cloud);
    time_buffer_.push_back(timestamp);
    last_timestamp_lidar_ = timestamp;
}

bool FastLioFixedMap::SyncMeasurements() {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;

    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / double(1000);
            lidar_mean_scantime_ += (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;
            if ((lidar_end_time_ - measures_.lidar_begin_time_) > 5 * lo::lidar_time_interval) {
                lidar_end_time_ = measures_.lidar_begin_time_ + lo::lidar_time_interval;
                lidar_mean_scantime_ = lo::lidar_time_interval;
            }
        }
        lo::lidar_time_interval = lidar_mean_scantime_;
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) return false;

    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

bool FastLioFixedMap::RunOnce(NavState* state) {
    if (!SyncMeasurements()) return false;

    // IMU process + undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (!scan_undistort_ || scan_undistort_->empty()) return false;

    if (!tracking_enabled_) {
        last_lidar_update_valid_ = false;
        state_point_ = kf_.GetX();
        state_point_.timestamp_ = measures_.lidar_end_time_;
        *state = state_point_;
        return true;
    }

    if (flg_first_scan_) {
        // 固定地图不变量: 冷启动首帧只初始化 state/时间基准,
        // 绝不向 fixed_ivox_ 插入当前 scan 点 —— 固定地图永不被当前 scan 污染.
        // fixed_ivox_ 在 LoadFixedMap / RebuildLocalMapAround 时已由地图点建好,
        // ESKF 从下一帧起直接对固定地图匹配, 无需首帧建图起步.
        state_point_ = kf_.GetX();
        state_point_.timestamp_ = measures_.lidar_end_time_;
        first_lidar_time_ = measures_.lidar_end_time_;
        flg_first_scan_ = false;
        last_lidar_update_valid_ = false;

        *state = state_point_;
        return true;
    }

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    // Downsample
    last_raw_points_ = static_cast<int>(scan_undistort_->size());
    last_down_points_ = 0;
    last_effective_surface_points_ = 0;
    last_lidar_residual_mean_ = 0.0;
    last_lidar_residual_max_ = 0.0;

    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    int cur_pts = scan_down_body_->size();
    if (cur_pts < static_cast<int>(scan_undistort_->size()) * 0.1 || cur_pts < min_pts_) {
        auto fine_voxel = voxel_scan_;
        fine_voxel.setLeafSize(0.1, 0.1, 0.1);
        fine_voxel.setInputCloud(scan_undistort_);
        fine_voxel.filter(*scan_down_body_);
        cur_pts = scan_down_body_->size();
    }
    last_down_points_ = cur_pts;

    if (cur_pts < 5) {
        LOG_EVERY_N(WARNING, 20) << "FastLioFixedMap: too few downsampled points, raw="
                                 << scan_undistort_->size() << ", down=" << cur_pts;
        last_lidar_update_valid_ = false;
        state_point_ = kf_.GetX();
        state_point_.timestamp_ = measures_.lidar_end_time_;
        *state = state_point_;
        return true;
    }

    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, 1);
    plane_coef_.resize(cur_pts, Vec4f::Zero());

    // EKF update with lidar observation
    last_lidar_update_valid_ = false;
    const bool update_accepted = kf_.Update(ESKF::ObsType::LIDAR, 1.0);
    last_lidar_update_valid_ = last_lidar_update_valid_ && update_accepted;

    const bool residual_ok = last_lidar_residual_mean_ <= max_lidar_residual_mean_;
    if (!residual_ok) {
        last_lidar_update_valid_ = false;
        LOG_EVERY_N(WARNING, 10) << "FastLioFixedMap: lidar residual too high, median_sq="
                                 << last_lidar_residual_mean_ << " > " << max_lidar_residual_mean_
                                 << ", max_sq=" << last_lidar_residual_max_;
    }
    // 稳态诊断: 时间式限频 ~每 5s 一行, 与帧率解耦, 控制嵌入式日志预算 (异常走上方 LOG_EVERY_N 警告)
    LOG_EVERY_T(INFO, 5.0) << "FastLioFixedMap diag: raw=" << last_raw_points_
                          << ", down=" << last_down_points_
                          << ", eff=" << last_effective_surface_points_
                          << ", update_accepted=" << update_accepted
                          << ", residual_mean_sq=" << last_lidar_residual_mean_
                          << ", residual_max_sq=" << last_lidar_residual_max_
                          << ", final_res_ratio=" << kf_.GetFinalRes()
                          << ", quality_good=" << TrackingQualityGood()
                          << ", pose=[" << ImuPoseToLidarPose(kf_.GetX().GetPose()).translation().transpose()
                          << "]";

    // ==== ZUPT 零速更新 ====
    // 走廊沿轴几何不可观, NDT 在该方向同样退化拉不回; ZUPT 用零速运动模型钳死蠕动, 与几何观测正交.
    // 双门触发: StaticDetector.is_static (滑窗 IMU std + 非对称迟滞) && EKF 速度模 < zupt_vel_gate_.
    bool is_static = false;
    if (zupt_enabled_ && flg_EKF_inited_) {
        for (const auto& imu : measures_.imu_) {
            if (imu) static_detector_.AddImu(*imu);
        }
        is_static = static_detector_.IsStatic();
        if (is_static && kf_.GetX().GetVel().norm() < zupt_vel_gate_) {
            kf_.ZuptUpdate(zupt_vel_cov_);
            LOG_EVERY_N(INFO, 20) << "ZUPT applied, |v|=" << kf_.GetX().GetVel().norm();
        }
    }

    state_point_ = kf_.GetX();
    state_point_.timestamp_ = measures_.lidar_end_time_;
    state_point_.is_parking_ = is_static;

    *state = state_point_;
    return true;
}

void FastLioFixedMap::ObsModel(NavState& s, ESKF::CustomObservationModel& obs) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (int i = 0; i < cnt_pts; ++i) index[i] = i;

    Mat3f R_wl = (s.rot_.matrix() * offset_R_lidar_fixed_).cast<float>();
    Vec3f t_wl = (s.rot_ * offset_t_lidar_fixed_ + s.pos_).cast<float>();

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t& i) {
        PointType& point_body = scan_down_body_->points[i];
        PointType& point_world = scan_down_world_->points[i];

        Vec3f p_body = point_body.getVector3fMap();
        point_world.getVector3fMap() = R_wl * p_body + t_wl;
        point_world.intensity = point_body.intensity;

        auto& points_near = nearest_points_[i];
        points_near.clear();

        fixed_ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
        point_selected_surf_[i] = points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;

        if (point_selected_surf_[i]) {
            point_selected_surf_[i] = math::esti_plane(plane_coef_[i], points_near, esti_plane_threshold_);
        }

        if (point_selected_surf_[i]) {
            auto temp = point_world.getVector4fMap();
            temp[3] = 1.0;
            float pd2 = plane_coef_[i].dot(temp);
            if (p_body.norm() > 81 * pd2 * pd2) {
                point_selected_surf_[i] = true;
                residuals_[i] = pd2;
            } else {
                point_selected_surf_[i] = false;
            }
        }
    });

    int effect_feat_surf_ = 0;
    std::vector<Vec4f> corr_pts(cnt_pts);
    std::vector<Vec4f> corr_norm(cnt_pts);

    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm[effect_feat_surf_] = plane_coef_[i];
            corr_pts[effect_feat_surf_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts[effect_feat_surf_][3] = residuals_[i];
            effect_feat_surf_++;
        }
    }

    corr_pts.resize(effect_feat_surf_);
    corr_norm.resize(effect_feat_surf_);

    if (effect_feat_surf_ < 20) {
        obs.valid_ = false;
        last_effective_surface_points_ = effect_feat_surf_;
        LOG_EVERY_N(WARNING, 20) << "FastLioFixedMap: not enough effective surface points: "
                                 << effect_feat_surf_ << " / " << cnt_pts;
        return;
    }
    last_effective_surface_points_ = effect_feat_surf_;
    last_lidar_update_valid_ = true;

    const Mat3f off_R = offset_R_lidar_fixed_.cast<float>();
    const Vec3f off_t = offset_t_lidar_fixed_.cast<float>();
    const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

    obs.HTH_.setZero();
    obs.HTr_.setZero();

    std::vector<Mat6d> JTJ(effect_feat_surf_);
    std::vector<Vec6d> JTr(effect_feat_surf_);
    std::vector<double> res_sq(effect_feat_surf_);

    std::vector<size_t> surf_idx(effect_feat_surf_);
    for (int i = 0; i < effect_feat_surf_; i++) surf_idx[i] = i;

    std::for_each(std::execution::par_unseq, surf_idx.begin(), surf_idx.end(), [&](const size_t& i) {
        Vec3f point_this_be = corr_pts[i].head<3>();
        Vec3f point_this = off_R * point_this_be + off_t;
        Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);

        Vec3f norm_vec = corr_norm[i].head<3>();
        Vec3f C(Rt * norm_vec);
        Vec3f A(point_crossmat * C);

        Eigen::Matrix<double, 1, ESKF::pose_obs_dim_> J;
        J.setZero();
        J << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2];

        float res = -corr_pts[i][3];
        JTJ[i] = (J.transpose() * J).eval();
        JTr[i] = J.transpose() * res;
        res_sq[i] = res * res;
    });

    for (int i = 0; i < effect_feat_surf_; ++i) {
        obs.HTH_ += JTJ[i];
        obs.HTr_ += JTr[i];
    }

    if (!res_sq.empty()) {
        std::sort(res_sq.begin(), res_sq.end());
        obs.lidar_residual_mean_ = res_sq[res_sq.size() / 2];
        obs.lidar_residual_max_ = res_sq[res_sq.size() - 1];
        last_lidar_residual_mean_ = obs.lidar_residual_mean_;
        last_lidar_residual_max_ = obs.lidar_residual_max_;
    }
}

bool FastLioFixedMap::TrackingQualityGood() const {
    return tracking_enabled_ && last_lidar_update_valid_ && flg_EKF_inited_ && kf_.GetFinalRes() < 5.0 &&
           last_lidar_residual_mean_ <= max_lidar_residual_mean_;
}

}  // namespace hikari::loclite
