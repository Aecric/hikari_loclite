#pragma once

#ifndef HIKARI_LOCLITE_NODE_HPP
#define HIKARI_LOCLITE_NODE_HPP

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lio/fast_lio_fixed_map.hpp"
#include "lio/gravity_alignment.h"
#include "ndt/ndt_corrector.hpp"
#include "reloc/reloc_manager.hpp"
#include "system/lite_pose_smoother.hpp"
#include "system/loclite_state_machine.hpp"
#include "system/realtime_setup.hpp"
#include "system/stability_gate.hpp"

namespace hikari::loclite {

/// 轻量定位节点.
/// 对外话题 / TF / 服务面与 lightning 定位模式一致 (仅话题前缀 lightning/ → hikari_loc/),
/// 可直接替换 lightning 定位模式做验证, 详见契约文档 hikari_loclite_build_2026-06-10.md 第 21 节.
class LocLiteNode : public rclcpp::Node {
   public:
    LocLiteNode();

    /// @param cli_config   CLI --config 传入的 yaml 路径, 为空时退回 ROS param "config_path" (launch 兼容)
    /// @param cli_map_path CLI --map_path 传入的 lightning 地图目录 (内含 global.pcd / sc_database.bin 等);
    ///                     非空时固定地图从 <map_path>/global.pcd 加载,
    ///                     为空时退回 yaml system.map_path, 再退回 fixed_map.global_pcd
    bool Init(const std::string& cli_config = "", const std::string& cli_map_path = "");
    void Shutdown();

    /// 离线评估节点 (run_loclite_offline) 直接按 bag 顺序喂数据, 走与 online 完全相同的处理路径
    void FeedImu(sensor_msgs::msg::Imu::SharedPtr msg) { OnImu(std::move(msg)); }
    void FeedLivox(livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { OnLivox(std::move(msg)); }
    void FeedCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) { OnCloud(std::move(msg)); }

    LiteLocState CurrentState() const { return state_machine_.State(); }
    const char* CurrentStateStr() const { return state_machine_.StateStr(); }

    /// CPU 亲和 / 实时调度参数 (yaml system.rt_*), 供 main() 在 spin 前对 spin 线程应用
    const RealtimeOptions& realtime_options() const { return rt_options_; }

   private:
    void OnImu(sensor_msgs::msg::Imu::SharedPtr msg);
    void OnLivox(livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    void OnCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void OnInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /// 一帧 lidar 完整处理: LIO → /initialpose sticky retry → 状态机驱动 → 发布 (持锁)
    void ProcessFrame();
    /// /initialpose pending pose 的累积点云 NDT 验证 (持锁, ProcessFrame 内调用)
    void HandlePendingInitPose(const CloudPtr& scan, double ts);
    bool UpdatePendingInitPoseWithGravity();
    void ResetInitAccumulation();
    bool AccumulateInitScan(const CloudPtr& scan, double ts);
    CloudPtr BuildInitAccumulatedCloud() const;
    /// Good 态低频 NDT 漂移校正 (按时间间隔节流, 设计文档第 14 节)
    void MaybeNdtCorrectGood(const CloudPtr& scan, NavState& state, double ts);
    /// SC 重定位尝试 (LOST 态 or cold start), 持锁调用
    void TryScRelocalize(const CloudPtr& scan, double ts, bool manual = false);
    /// 发布 SC 调试话题 (仅在有订阅者时发布)
    void PublishScDebugTopics();
    /// 发布 KISS query(level) 累积云到 sc/accum_cloud (复用 SC 话题, 仅在有订阅者时发布)
    void PublishKissAccumCloud();

    /// odom + TF(map→base_link) + path
    void PublishPose(const NavState& state);
    /// loc_state + ndt_status + loc_status marker
    void PublishStatusTopics(double ts);
    void PublishStatusMarker(double ts);
    /// 输入掉线 watchdog + 富状态话题, 由 health_timer_ 周期调用 (持锁)
    void OnHealthTimer();
    /// /hikari_loc/status: [state, ndt_conf, imu_age_s, lidar_age_s, fps, in_map] (持锁)
    void PublishRichStatus(double now_wall, double imu_age, double lidar_age);
    /// 发布降采样后的全局 PCD 地图到 /pcdmap (transient_local, 供 RViz 定位验证)
    void PublishPcdMap();
    /// lidar_frame → level_frame 重力对齐 TF (每条 IMU 后调用, 持锁)
    void PublishLevelFrameTF(const NavState& state);

    // --- 配置 ---
    std::string config_path_;
    std::string map_frame_id_ = "map";
    std::string lidar_frame_id_ = "lidar_frame";
    std::string level_frame_id_ = "level_frame";
    std::string base_frame_id_ = "base_link";
    SE3 T_lidar_base_;                              // inv(T_base_lidar), 由 system.base_to_lidar_* 构造
    Mat3d cached_extrinsic_R_ = Mat3d::Identity();  // fast_lio.extrinsic_R (R_imu_lidar), 供 level_frame TF 用
    double external_pose_blackout_sec_ = 5.0;
    double lost_timeout_sec_ = 5.0;
    int init_max_retries_ = 5;
    bool init_accum_enabled_ = true;
    int init_accum_min_frames_ = 10;
    int init_accum_min_points_ = 8000;
    double init_accum_window_sec_ = 2.0;
    double init_accum_max_wait_sec_ = 5.0;
    double init_accum_voxel_leaf_ = 0.1;
    double ndt_good_rate_hz_ = 1.0;
    double ndt_gain_good_ = 0.5;
    bool stability_gate_enabled_ = true;  // system.stability_gate_enabled: false 时验证通过即直接 Good
    bool publish_tf_ = true;    // runtime.publish_tf: map→base_link 与 lidar→level TF
    bool publish_odom_ = true;  // runtime.publish_odom: hikari_loc/odom
    bool publish_path_ = true;  // runtime.publish_path: hikari_loc/path (契约默认开启)
    bool publish_pcdmap_ = false;  // runtime.publish_pcdmap: /pcdmap 原始 PCD 降采样地图
    static constexpr size_t kMaxPathPoses = 5000;

    // --- 输入掉线 watchdog + 富状态上报 (runtime.*) ---
    bool watchdog_enabled_ = true;       // runtime.watchdog_enabled: IMU/Lidar 掉线检测
    double imu_timeout_sec_ = 1.0;       // runtime.imu_timeout_sec: IMU 静默超此值视为掉线
    double lidar_timeout_sec_ = 2.0;     // runtime.lidar_timeout_sec: Lidar 静默超此值视为掉线
    bool status_topic_enabled_ = true;   // runtime.status_topic_enabled: 发布 /hikari_loc/status 富状态

    // --- CPU 亲和 / 实时调度 (system.rt_*), 在 Init 中解析, 由 main() spin 前应用 ---
    RealtimeOptions rt_options_;

    // --- 运行时状态 (mutex_ 保护) ---
    std::mutex mutex_;
    bool lio_has_output_ = false;       // LIO 是否产出过至少一帧 (即"LIO 就绪")
    bool has_pending_init_pose_ = false;
    Vec3d pending_init_base_position_ = Vec3d::Zero();  // /initialpose 的 map-frame base position
    double pending_init_yaw_ = 0.0;                     // /initialpose 的 map-frame yaw
    bool pending_init_pose_gravity_ready_ = false;      // 已用当前 lidar roll/pitch 补偿
    SE3 pending_init_pose_;                             // 补偿后的 T_map_lidar, FC/NDT 用
    int init_retry_count_ = 0;
    CloudPtr init_accum_cloud_{new PointCloudType()};
    int init_accum_frames_ = 0;
    int init_accum_points_ = 0;
    double init_accum_first_ts_ = -1.0;
    double init_accum_window_start_ts_ = -1.0;
    double blackout_deadline_sec_ = 0.0;  // SC 自动注入阻断窗口截止时刻 (initialpose blackout, 墙钟域, 勿与 scan ts 比较)
    double lost_enter_ts_ = -1.0;         // Lost 进入时刻 (传感器时间), 供 lost_timeout_sec 计时
    double last_ndt_correct_ts_ = -1.0;   // Good 态 NDT 校正节流时刻
    double last_ndt_confidence_ = 0.0;    // 最近一次 NdtResult.confidence, 供 marker 显示
    double last_path_pub_ts_ = -1.0;
    double last_level_tf_ts_ = -1.0;      // 上次 level TF 的状态时间戳, 防止 IMU 速率下重复广播同一 stamp
    double last_sc_attempt_ts_ = -1.0;    // SC 重定位尝试节流 (cooldown)
    SE3 marker_pose_;                     // loc_status marker 的悬浮位置 (最近一次有意义的位姿)
    // watchdog 时戳全部用 node clock (this->now()): 真机=墙钟, sim/bag=sim 钟, 自洽且不与 scan ts 跨域比较
    double last_imu_wall_ts_ = -1.0;      // 最近一条 IMU 到达时刻
    double last_lidar_wall_ts_ = -1.0;    // 最近一帧 Lidar (Livox/PointCloud2) 到达时刻
    double input_stale_since_wall_ = -1.0;  // 输入掉线起始时刻 (-1 = 未掉线), 用于告警持续时长
    double last_frame_wall_ts_ = -1.0;    // 上一帧 LIO 产出时刻, 算 fps
    double frame_fps_ = 0.0;              // LIO 输出帧率 EWMA

    // --- 组件 ---
    FastLioFixedMap::Ptr lio_;
    NdtCorrector::Ptr ndt_;
    RelocManager::Ptr reloc_;
    LitePoseSmoother smoother_;
    LocLiteStateMachine state_machine_;
    StabilityGate stability_gate_;  // Initializing → Good 放行门控 (持锁访问)
    GravityAlignmentFilter gravity_filter_;

    // --- ROS 接口 ---
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr loc_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ndt_status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loc_status_marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sc_accum_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sc_candidates_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sc_init_guess_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr sc_gravity_check_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;  // /hikari_loc/status 富状态
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sc_reloc_service_;
    rclcpp::TimerBase::SharedPtr wait_state_timer_;  // WAIT_FOR_INITIALPOSE 下 1Hz 持续上报
    rclcpp::TimerBase::SharedPtr health_timer_;      // 输入掉线 watchdog + 富状态上报 (~5Hz)

    nav_msgs::msg::Path path_msg_;
};

}  // namespace hikari::loclite

#endif
