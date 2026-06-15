#pragma once

#ifndef HIKARI_LOCLITE_FAST_LIO_FIXED_MAP_HPP
#define HIKARI_LOCLITE_FAST_LIO_FIXED_MAP_HPP

#include <memory>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/measure_group.h"
#include "common/nav_state.h"
#include "common/options.h"
#include "common/point_def.h"
#include "ivox3d/ivox3d.h"
#include "lio/eskf.hpp"
#include "lio/imu_processing.hpp"
#include "lio/pointcloud_preprocess.h"
#include "lio/static_detector.h"

namespace hikari::loclite {

class FastLioFixedMap {
   public:
    using Ptr = std::shared_ptr<FastLioFixedMap>;
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    bool Init(const std::string& yaml_path);
    bool LoadFixedMapFromConfig(const std::string& yaml_path);
    bool LoadFixedMap(const std::string& pcd_path, double voxel_leaf);
    bool RebuildLocalMapAround(const SE3& T_map_lidar);
    bool ResetToMapPose(const SE3& T_map_lidar, bool reset_velocity = true);

    void AddImu(const sensor_msgs::msg::Imu::SharedPtr& imu);
    void AddCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void AddLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    bool RunOnce(NavState* state);

    CloudPtr LatestDeskewedCloud() const { return scan_undistort_; }
    NavState LatestState() const { return state_point_; }
    SE3 ImuPoseToLidarPose(const SE3& T_map_imu) const;
    SE3 LidarPoseToImuPose(const SE3& T_map_lidar) const;
    SE3 LatestPose() const { return ImuPoseToLidarPose(state_point_.GetPose()); }
    bool TrackingEnabled() const { return tracking_enabled_; }
    bool LastLidarUpdateValid() const { return last_lidar_update_valid_; }
    bool TrackingQualityGood() const;

    /// 已加载的固定地图点云 (降采样后), 供 NdtCorrector::SetMap 复用, 避免二次读盘
    CloudPtr FixedMapCloud() const { return fixed_map_cloud_; }

   private:
    bool SyncMeasurements();
    bool DeskewCurrentScan();
    bool MatchAgainstFixedMap(NavState* state);

    void ObsModel(NavState& s, ESKF::CustomObservationModel& obs);

    // Fixed map
    CloudPtr fixed_map_cloud_;
    std::shared_ptr<IVoxType> fixed_ivox_;
    double crop_radius_m_ = 30.0;
    int max_points_ = 800000;
    double filter_size_map_min_ = 0.2;
    int min_pts_ = 300;
    float esti_plane_threshold_ = fasterlio::ESTI_PLANE_THRESHOLD;
    double max_lidar_residual_mean_ = 0.05;

    // LIO components
    ESKF kf_;
    std::shared_ptr<PointCloudPreprocess> preprocess_;
    std::shared_ptr<ImuProcess> p_imu_;

    // ZUPT 零速更新 (Phase 1, 治走廊静止沿轴蠕动; 与 NDT 正交)
    StaticDetector static_detector_;
    bool zupt_enabled_ = true;
    double zupt_vel_gate_ = 0.05;    // m/s, EKF 速度模 < 此才允许 ZUPT (第一道闸)
    double zupt_vel_cov_ = 1.0e-3;   // 零速伪观测噪声 (越小钳得越硬)

    // iVox options
    IVoxType::Options ivox_options_;

    // State
    NavState state_point_;
    bool tracking_enabled_ = false;
    bool last_lidar_update_valid_ = false;
    int last_raw_points_ = 0;
    int last_down_points_ = 0;
    int last_effective_surface_points_ = 0;
    double last_lidar_residual_mean_ = 0.0;
    double last_lidar_residual_max_ = 0.0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double first_lidar_time_ = 0.0;
    double last_timestamp_imu_ = -1.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;

    // Buffers
    std::deque<IMUPtr> imu_buffer_;
    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<double> time_buffer_;
    MeasureGroup measures_;
    bool lidar_pushed_ = false;

    // Point clouds
    CloudPtr scan_undistort_{new PointCloudType()};
    CloudPtr scan_down_body_{new PointCloudType()};
    CloudPtr scan_down_world_{new PointCloudType()};
    pcl::VoxelGrid<PointType> voxel_scan_;

    // ObsModel buffers
    std::vector<PointVector> nearest_points_;
    std::vector<char> point_selected_surf_;
    std::vector<Vec4f> plane_coef_;
    std::vector<float> residuals_;

    // Extrinsic
    Mat3d offset_R_lidar_fixed_ = Mat3d::Identity();
    Vec3d offset_t_lidar_fixed_ = Vec3d::Zero();

    // Mutex
    std::mutex mtx_buffer_;
};

}  // namespace hikari::loclite

#endif
