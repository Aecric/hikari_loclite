#include "system/loclite_node.hpp"
#include "math/loclite_math.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace hikari::loclite {

namespace {

geometry_msgs::msg::Pose ToPoseMsg(const SE3& T) {
    geometry_msgs::msg::Pose p;
    p.position.x = T.translation().x();
    p.position.y = T.translation().y();
    p.position.z = T.translation().z();
    const auto q = T.unit_quaternion();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}

rclcpp::Time ToRosTime(double ts) { return rclcpp::Time(static_cast<int64_t>(ts * 1e9)); }

}  // namespace

LocLiteNode::LocLiteNode() : rclcpp::Node("hikari_loclite") {}

bool LocLiteNode::Init(const std::string& cli_config, const std::string& cli_map_path) {
    // config 路径优先级: CLI --config > ROS param config_path (launch 文件兼容)
    this->declare_parameter<std::string>("config_path", "");
    config_path_ = cli_config;
    if (config_path_.empty()) {
        this->get_parameter("config_path", config_path_);
    }
    if (config_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "no config: pass --config <yaml> or set config_path parameter");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing with config: %s", config_path_.c_str());

    // 读取话题名 / frame / system / reloc / ndt 节流参数
    std::string imu_topic = "/livox/imu";
    std::string livox_topic = "/livox/lidar";
    std::string cloud_topic = "/cloud";
    std::string yaml_map_path;
    int lidar_type = 1;
    double voxel_leaf = 0.2;
    double level_cutoff = 0.5;
    StabilityGate::Options gate_options;
    LitePoseSmoother::Options smoother_options;  // smoother.* 缺省即 Options 默认 (0.5m/15° 输出, 0.5m/8° 校正)
    try {
        auto yaml = YAML::LoadFile(config_path_);
        if (yaml["common"]) {
            auto common = yaml["common"];
            imu_topic = common["imu_topic"].as<std::string>(imu_topic);
            livox_topic = common["livox_lidar_topic"].as<std::string>(livox_topic);
            cloud_topic = common["pointcloud_topic"].as<std::string>(cloud_topic);
            map_frame_id_ = common["map_frame_id"].as<std::string>(map_frame_id_);
        }
        if (yaml["system"]) {
            auto sys = yaml["system"];
            yaml_map_path = sys["map_path"].as<std::string>("");
            lidar_frame_id_ = sys["lidar_frame_id"].as<std::string>(lidar_frame_id_);
            level_frame_id_ = sys["level_frame_id"].as<std::string>(level_frame_id_);
            base_frame_id_ = sys["base_frame_id"].as<std::string>(base_frame_id_);
            external_pose_blackout_sec_ = sys["external_pose_blackout_sec"].as<double>(5.0);
            lost_timeout_sec_ = sys["lost_timeout_sec"].as<double>(5.0);
            level_cutoff = sys["level_frame_cutoff_freq"].as<double>(0.5);

            // 稳定门控 knobs (轻量版 lightning StabilityGate, 阈值默认照 default_livox.yaml:131-137;
            // conf_upper_thres 按本包 TP 提前放行语义重标, 见 stability_gate.hpp / yaml 注释)
            gate_options.enabled = sys["stability_gate_enabled"].as<bool>(true);
            gate_options.trans_thres = sys["stability_gate_trans_thres"].as<double>(0.1);
            gate_options.rot_thres_deg = sys["stability_gate_rot_thres_deg"].as<double>(4.0);
            gate_options.window_sec = sys["stability_gate_window_sec"].as<double>(3.0);
            gate_options.conf_upper_thres = sys["stability_gate_conf_upper_thres"].as<double>(3.0);

            // base_link → lidar 外参: R = Rz(yaw)·Ry(pitch)·Rx(roll), T_lidar_base = inv(T_base_lidar)
            const double bx = sys["base_to_lidar_x"].as<double>(0.0);
            const double by = sys["base_to_lidar_y"].as<double>(0.0);
            const double bz = sys["base_to_lidar_z"].as<double>(0.0);
            const double br = sys["base_to_lidar_roll"].as<double>(0.0);
            const double bp = sys["base_to_lidar_pitch"].as<double>(0.0);
            const double byw = sys["base_to_lidar_yaw"].as<double>(0.0);
            const Eigen::AngleAxisd roll_aa(br, Vec3d::UnitX());
            const Eigen::AngleAxisd pitch_aa(bp, Vec3d::UnitY());
            const Eigen::AngleAxisd yaw_aa(byw, Vec3d::UnitZ());
            const Mat3d R_base_lidar = (yaw_aa * pitch_aa * roll_aa).toRotationMatrix();
            const SE3 T_base_lidar(SO3(R_base_lidar), Vec3d(bx, by, bz));
            T_lidar_base_ = T_base_lidar.inverse();
            RCLCPP_INFO(this->get_logger(), "base->lidar extrinsic: t=[%.3f, %.3f, %.3f], rpy=[%.3f, %.3f, %.3f]",
                        bx, by, bz, br, bp, byw);

            // CPU 亲和 / 实时调度 (在 main() spin 前应用; 需 cap_sys_nice, 无权限降级继续)
            rt_options_.enabled = sys["rt_enabled"].as<bool>(false);
            rt_options_.cpu_cores.clear();
            if (sys["rt_cpu_cores"]) {
                rt_options_.cpu_cores = sys["rt_cpu_cores"].as<std::vector<int>>(std::vector<int>{});
            }
            rt_options_.sched_fifo = sys["rt_sched_fifo"].as<bool>(false);
            rt_options_.priority = sys["rt_priority"].as<int>(80);
        }
        if (yaml["fast_lio"]) {
            lidar_type = yaml["fast_lio"]["lidar_type"].as<int>(1);
        }
        if (yaml["reloc"]) {
            auto reloc = yaml["reloc"];
            init_max_retries_ = reloc["init_max_retries"].as<int>(5);
            init_accum_enabled_ = reloc["init_accum_enabled"].as<bool>(true);
            init_accum_min_frames_ = reloc["init_accum_min_frames"].as<int>(10);
            init_accum_min_points_ = reloc["init_accum_min_points"].as<int>(8000);
            init_accum_window_sec_ = reloc["init_accum_window_sec"].as<double>(2.0);
            init_accum_max_wait_sec_ = reloc["init_accum_max_wait_sec"].as<double>(5.0);
            init_accum_voxel_leaf_ = reloc["init_accum_voxel_leaf"].as<double>(0.1);
        }
        if (yaml["runtime"]) {
            auto rt = yaml["runtime"];
            publish_tf_ = rt["publish_tf"].as<bool>(true);
            publish_odom_ = rt["publish_odom"].as<bool>(true);
            publish_path_ = rt["publish_path"].as<bool>(true);
            publish_pcdmap_ = rt["publish_pcdmap"].as<bool>(false);
            watchdog_enabled_ = rt["watchdog_enabled"].as<bool>(true);
            imu_timeout_sec_ = rt["imu_timeout_sec"].as<double>(1.0);
            lidar_timeout_sec_ = rt["lidar_timeout_sec"].as<double>(2.0);
            status_topic_enabled_ = rt["status_topic_enabled"].as<bool>(true);
        }
        if (yaml["ndt"]) {
            ndt_good_rate_hz_ = yaml["ndt"]["good_rate_hz"].as<double>(1.0);
            ndt_gain_good_ = yaml["ndt"]["gain_good"].as<double>(0.5);
        }
        // smoother 跳变门限 (Phase1 可配; 缺省用 Options 默认值, 见 lite_pose_smoother.hpp)
        if (yaml["smoother"]) {
            auto sm = yaml["smoother"];
            smoother_options.max_output_jump_trans_m =
                sm["max_output_jump_trans_m"].as<double>(smoother_options.max_output_jump_trans_m);
            smoother_options.max_output_jump_rot_deg =
                sm["max_output_jump_rot_deg"].as<double>(smoother_options.max_output_jump_rot_deg);
            smoother_options.max_correction_trans_m =
                sm["max_correction_trans_m"].as<double>(smoother_options.max_correction_trans_m);
            smoother_options.max_correction_rot_deg =
                sm["max_correction_rot_deg"].as<double>(smoother_options.max_correction_rot_deg);
        }
        if (yaml["fixed_map"]) {
            voxel_leaf = yaml["fixed_map"]["voxel_leaf"].as<double>(0.2);
        }
        if (yaml["fast_lio"] && yaml["fast_lio"]["extrinsic_R"]) {
            auto er = yaml["fast_lio"]["extrinsic_R"].as<std::vector<double>>();
            if (er.size() == 9) {
                cached_extrinsic_R_ = math::MatFromArray<double>(er);
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse config %s: %s", config_path_.c_str(), e.what());
        return false;
    }

    // Create components
    lio_ = std::make_shared<FastLioFixedMap>();
    ndt_ = std::make_shared<NdtCorrector>();
    reloc_ = std::make_shared<RelocManager>();

    // Init LIO
    if (!lio_->Init(config_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init FastLioFixedMap");
        return false;
    }

    // 固定地图加载优先级: CLI --map_path > yaml system.map_path > yaml fixed_map.global_pcd
    // map_path 指向 lightning 地图目录, 固定地图取 <map_path>/global.pcd
    const std::string map_dir = !cli_map_path.empty() ? cli_map_path : yaml_map_path;
    if (!map_dir.empty()) {
        const std::string pcd_path = map_dir + "/global.pcd";
        RCLCPP_INFO(this->get_logger(), "Loading fixed map from map dir: %s", pcd_path.c_str());
        if (!lio_->LoadFixedMap(pcd_path, voxel_leaf)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load fixed map: %s", pcd_path.c_str());
            return false;
        }
    } else if (!lio_->LoadFixedMapFromConfig(config_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load fixed map");
        return false;
    }

    // Init NDT
    if (!ndt_->Init(config_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init NdtCorrector");
        return false;
    }
    if (!ndt_->SetMap(lio_->FixedMapCloud())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set NDT target map");
        return false;
    }

    // Init RelocManager (SC pipeline)
    if (!reloc_->Init(config_path_, map_dir)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init RelocManager");
        return false;
    }

    // 稳定门控: /initialpose 验证通过后先 Initializing, 滑窗收敛 (或高 TP 提前) 才放行 Good
    stability_gate_.Init(gate_options);
    stability_gate_enabled_ = gate_options.enabled;
    RCLCPP_INFO(this->get_logger(),
                "StabilityGate: enabled=%d, trans_thres=%.2f m, rot_thres=%.1f deg, window=%.1f s, "
                "conf_upper_thres(TP)=%.2f",
                gate_options.enabled ? 1 : 0, gate_options.trans_thres, gate_options.rot_thres_deg,
                gate_options.window_sec, gate_options.conf_upper_thres);
    RCLCPP_INFO(this->get_logger(),
                "Init accumulation: enabled=%d, min_frames=%d, min_points=%d, window=%.2fs, max_wait=%.2fs, "
                "voxel_leaf=%.2fm",
                init_accum_enabled_ ? 1 : 0, init_accum_min_frames_, init_accum_min_points_,
                init_accum_window_sec_, init_accum_max_wait_sec_, init_accum_voxel_leaf_);

    // 位姿平滑器: 输出跳变门限 + GOOD 态 NDT 校正门限 (Phase1 可配, 缺省即 Options 默认值)
    smoother_.Init(smoother_options);
    RCLCPP_INFO(this->get_logger(),
                "LitePoseSmoother: max_output_jump=%.2f m/%.1f deg, max_correction=%.2f m/%.1f deg",
                smoother_options.max_output_jump_trans_m, smoother_options.max_output_jump_rot_deg,
                smoother_options.max_correction_trans_m, smoother_options.max_correction_rot_deg);

    // 重力对齐滤波器 (lidar_frame → level_frame 纯旋转 TF)
    GravityAlignmentFilter::Options gf_options;
    gf_options.cutoff_freq = level_cutoff;
    gf_options.lidar_frame_id = lidar_frame_id_;
    gf_options.level_frame_id = level_frame_id_;
    gravity_filter_.Init(gf_options);
    RCLCPP_INFO(this->get_logger(), "GravityAlignmentFilter: cutoff_freq=%.2f Hz, lidar_frame=%s, level_frame=%s",
                level_cutoff, lidar_frame_id_.c_str(), level_frame_id_.c_str());

    // 初始状态: 等待外部 /initialpose 或 SC 自动重定位
    state_machine_.SetWaitForInitialPose("startup");
    // SC auto-on-init: arm reloc so first ProcessFrame can attempt SC
    if (reloc_->AutoOnInit() && reloc_->ScEnabled()) {
        reloc_->Arm("auto_on_init", this->now().seconds());
        RCLCPP_INFO(this->get_logger(), "SC auto_on_init: armed for first scan");
    }

    // Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::QoS(100),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) { OnImu(std::move(msg)); });

    if (lidar_type == 1) {
        livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic, rclcpp::QoS(5).best_effort(),
            [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { OnLivox(std::move(msg)); });
        RCLCPP_INFO(this->get_logger(), "LiDAR input: Livox CustomMsg only (%s)", livox_topic.c_str());
    } else {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, rclcpp::QoS(5).best_effort(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { OnCloud(std::move(msg)); });
        RCLCPP_INFO(this->get_logger(), "LiDAR input: PointCloud2 only (%s), lidar_type=%d",
                    cloud_topic.c_str(), lidar_type);
    }

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(5),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { OnInitialPose(std::move(msg)); });

    // Publishers (与 lightning 定位模式一致, 前缀 hikari_loc/)
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("hikari_loc/odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("hikari_loc/path", 10);
    path_msg_.header.frame_id = map_frame_id_;
    loc_state_pub_ = create_publisher<std_msgs::msg::Int32>("hikari_loc/loc_state", 10);
    ndt_status_pub_ = create_publisher<std_msgs::msg::Int32>("hikari_loc/ndt_status", 10);
    loc_status_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("hikari_loc/loc_status", 10);
    pcd_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pcdmap", rclcpp::QoS(1).reliable().transient_local());
    PublishPcdMap();

    // SC 重定位调试话题: QoS 1, 仅在有订阅者时发布 (PublishScDebugTopics)
    sc_accum_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("hikari_loc/sc/accum_cloud", 1);
    sc_candidates_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("hikari_loc/sc/candidates", 1);
    sc_init_guess_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("hikari_loc/sc/init_guess", 1);
    sc_gravity_check_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("hikari_loc/sc/gravity_check", 1);
    status_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("hikari_loc/status", 10);

    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // 手动 SC 重定位服务 (bypass blackout, 用户显式触发即视为授权抢注)
    sc_reloc_service_ = create_service<std_srvs::srv::Trigger>(
        "hikari_loc/sc_reloc",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
            RCLCPP_INFO(this->get_logger(), "[SC-Reloc] /hikari_loc/sc_reloc 收到调用");
            std::lock_guard<std::mutex> lk(mutex_);
            if (!reloc_->ScEnabled()) {
                response->success = false;
                response->message = "SC not enabled";
                return;
            }
            if (!lio_has_output_) {
                response->success = false;
                response->message = "LIO not ready (no scan yet)";
                return;
            }

            const CloudPtr scan = lio_->LatestDeskewedCloud();
            const SE3 current_imu_pose = lio_->LatestState().GetPose();
            auto candidate = reloc_->ManualRelocalize(scan, current_imu_pose);

            if (!candidate.valid) {
                response->success = false;
                response->message = "SC query returned no valid candidate";
                PublishScDebugTopics();
                return;
            }

            // NDT validation: SC 候选用 reloc.sc_max_delta_* 专用 delta 门限 (C1, 同自动路径)
            const NdtResult ndt_res = ndt_->Validate(candidate.pose, scan,
                                                     reloc_->ScMaxDeltaTransM(),
                                                     reloc_->ScMaxDeltaRotDeg());
            if (!ndt_res.valid) {
                RCLCPP_WARN(this->get_logger(),
                            "[SC-Reloc] manual SC candidate kf_id=%d rejected by NDT "
                            "(conf=%.3f, dt=%.3f m, dr=%.2f deg)",
                            candidate.kf_id, ndt_res.confidence, ndt_res.delta_trans_m, ndt_res.delta_rot_deg);
                response->success = false;
                response->message = "NDT validation failed";
                PublishScDebugTopics();
                return;
            }

            // Accept: reset to NDT-refined pose
            lio_->ResetToMapPose(ndt_res.pose);
            smoother_.Reset();
            last_ndt_confidence_ = ndt_res.confidence;
            marker_pose_ = ndt_res.pose;
            lost_enter_ts_ = -1.0;
            last_ndt_correct_ts_ = lio_->LatestState().timestamp_;
            has_pending_init_pose_ = false;
            init_retry_count_ = 0;
            ResetInitAccumulation();
            // ResetToMapPose 后 LIO 位姿域断裂, 稳定门控分支下 reloc 仍 armed, 必须清空 SC 累积缓冲
            reloc_->ClearAccumulation();

            if (stability_gate_enabled_) {
                stability_gate_.Reset();
                state_machine_.SetInitializing("manual_sc_validated");
                RCLCPP_INFO(this->get_logger(),
                            "[SC-Reloc] manual SC accepted: kf_id=%d, pos=[%.3f, %.3f, %.3f], TP=%.3f, "
                            "stability gate armed",
                            candidate.kf_id, ndt_res.pose.translation().x(),
                            ndt_res.pose.translation().y(), ndt_res.pose.translation().z(),
                            ndt_res.confidence);
            } else {
                state_machine_.SetGood("manual_sc_validated");
                if (reloc_->DisableAfterGood()) {
                    reloc_->Disarm("manual_sc_validated");
                }
                RCLCPP_INFO(this->get_logger(),
                            "[SC-Reloc] manual SC accepted: kf_id=%d, pos=[%.3f, %.3f, %.3f], TP=%.3f, GOOD",
                            candidate.kf_id, ndt_res.pose.translation().x(),
                            ndt_res.pose.translation().y(), ndt_res.pose.translation().z(),
                            ndt_res.confidence);
            }

            response->success = true;
            response->message = "SC relocalization succeeded (kf_id=" + std::to_string(candidate.kf_id) + ")";
            PublishScDebugTopics();
        });

    // WAIT_FOR_INITIALPOSE 下帧可能不产出, 用 1Hz wall timer 持续上报状态
    wait_state_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
        std::lock_guard<std::mutex> lk(mutex_);
        if (state_machine_.State() != LiteLocState::WaitForInitialPose) {
            return;
        }
        PublishStatusTopics(this->now().seconds());
    });

    // 输入掉线 watchdog + 富状态上报: ~5Hz 独立 wall timer (掉线时帧路径不跑, 仍需周期检查/上报)
    if (watchdog_enabled_ || status_topic_enabled_) {
        health_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] { OnHealthTimer(); });
    }
    RCLCPP_INFO(this->get_logger(),
                "Watchdog: enabled=%d, imu_timeout=%.2fs, lidar_timeout=%.2fs, status_topic=%d; "
                "RT: enabled=%d, sched_fifo=%d, prio=%d, cores=%zu",
                watchdog_enabled_ ? 1 : 0, imu_timeout_sec_, lidar_timeout_sec_,
                status_topic_enabled_ ? 1 : 0, rt_options_.enabled ? 1 : 0,
                rt_options_.sched_fifo ? 1 : 0, rt_options_.priority, rt_options_.cpu_cores.size());

    RCLCPP_INFO(this->get_logger(), "hikari_loclite initialized successfully (imu=%s, lidar_type=%d)",
                imu_topic.c_str(), lidar_type);
    return true;
}

void LocLiteNode::Shutdown() {
    RCLCPP_INFO(this->get_logger(), "hikari_loclite shutting down");
}

void LocLiteNode::OnImu(sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    last_imu_wall_ts_ = this->now().seconds();  // watchdog: IMU 到达时刻 (node clock)
    lio_->AddImu(msg);

    // 每条 IMU 处理后用 LIO 最新状态更新重力对齐滤波器并发布 lidar_frame → level_frame TF
    if (publish_tf_ && lio_has_output_ && tf_pub_) {
        PublishLevelFrameTF(lio_->LatestState());
    }
}

void LocLiteNode::OnLivox(livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        last_lidar_wall_ts_ = this->now().seconds();  // watchdog: Lidar 到达时刻 (node clock)
        lio_->AddLivox(msg);
    }
    ProcessFrame();
}

void LocLiteNode::OnCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        last_lidar_wall_ts_ = this->now().seconds();  // watchdog: Lidar 到达时刻 (node clock)
        lio_->AddCloud(msg);
    }
    ProcessFrame();
}

void LocLiteNode::OnInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);

    // 语义: 收到的是 T_map_base (2D 贴地, 通常 z=roll=pitch=0), 是"指令"而非"建议".
    // 真正要喂给 LIO 的是 T_map_lidar, 必须做外参与姿态补偿 (照 lightning loc_system.cc):
    //   1) 只取 /initialpose 的 x/y/z 与 yaw, roll/pitch 从当前 LIO 重力对齐姿态读取
    //   2) T_map_lidar = T_map_base × T_base_lidar
    const auto& p = msg->pose.pose;
    Quatd q_in(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    q_in.normalize();

    // Step 1: 提取 yaw (忽略其 roll/pitch, RViz 2D Pose Estimate 是平面工具, 即便非 0 也不可信)
    const Mat3d R_msg = q_in.toRotationMatrix();
    const double yaw_init = std::atan2(R_msg(1, 0), R_msg(0, 0));

    // 保存平面 /initialpose. 真正的 T_map_lidar 延迟到 LIO/IMU 已有重力姿态后构造,
    // 避免倾斜安装时过早用 roll/pitch=0 进入 FC/NDT.
    pending_init_base_position_ = Vec3d(p.position.x, p.position.y, p.position.z);
    pending_init_yaw_ = yaw_init;
    pending_init_pose_gravity_ready_ = false;
    has_pending_init_pose_ = true;
    init_retry_count_ = 0;
    ResetInitAccumulation();
    marker_pose_ = SE3(SO3(Eigen::AngleAxisd(yaw_init, Vec3d::UnitZ()).toRotationMatrix()),
                       pending_init_base_position_);
    if (lio_has_output_) {
        UpdatePendingInitPoseWithGravity();
    }

    // 状态机原子复位: 清空 bad/good 计数 (SetInitializing 内) 与 lost 计时器
    state_machine_.SetInitializing("external_pose");
    lost_enter_ts_ = -1.0;

    // blackout 窗口: 记录截止时刻, 阻断 SC 自动注入
    const double now_sec = this->now().seconds();
    blackout_deadline_sec_ = now_sec + external_pose_blackout_sec_;
    // Disarm SC during blackout to prevent automatic SC from overriding user's /initialpose
    if (reloc_->Armed()) {
        reloc_->Disarm("initialpose_blackout");
    }

    RCLCPP_INFO(this->get_logger(),
                "[ExtPose] /initialpose 已接入 (t=%.3f): pos=[%.3f, %.3f, %.3f], yaw=%.1f deg, "
                "gravity_comp=%d, blackout 窗口至 t=%.3f",
                now_sec, p.position.x, p.position.y, p.position.z, yaw_init * 180.0 / M_PI,
                pending_init_pose_gravity_ready_ ? 1 : 0, blackout_deadline_sec_);
}

void LocLiteNode::ProcessFrame() {
    std::lock_guard<std::mutex> lk(mutex_);

    NavState state;
    if (!lio_->RunOnce(&state)) {
        return;
    }
    lio_has_output_ = true;

    // LIO 输出帧率 EWMA (node clock, 供 /hikari_loc/status); 只在真正产出一帧时更新
    const double frame_wall = this->now().seconds();
    if (last_frame_wall_ts_ > 0.0 && frame_wall > last_frame_wall_ts_) {
        const double inst_fps = 1.0 / (frame_wall - last_frame_wall_ts_);
        frame_fps_ = frame_fps_ <= 0.0 ? inst_fps : 0.8 * frame_fps_ + 0.2 * inst_fps;
    }
    last_frame_wall_ts_ = frame_wall;

    const double ts = state.timestamp_;
    const CloudPtr scan = lio_->LatestDeskewedCloud();

    // /initialpose sticky retry: 每个新 scan 到来时验证 pending pose
    if (has_pending_init_pose_) {
        if (!UpdatePendingInitPoseWithGravity()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[FC-Init] waiting for LIO/IMU gravity compensation before accumulation");
            PublishStatusTopics(ts);
            return;
        }
        HandlePendingInitPose(scan, ts);
        PublishStatusTopics(ts);
        return;
    }

    const auto loc_state = state_machine_.State();

    // SC 查询点云滚动累积: armed (init/LOST) 期间逐帧入队 (降采样副本 + LIO 位姿).
    // 必须在 SC cooldown 门之前逐帧执行, 否则缓冲 5s 才进 1 帧永远凑不齐;
    // 合并/重力对齐/查询仍只在 TryScRelocalize 的 cooldown 节奏内发生.
    if (reloc_->Armed() && reloc_->ScEnabled()) {
        reloc_->AccumulateScan(scan, lio_->LatestPose());
    }

    if (loc_state == LiteLocState::Good || loc_state == LiteLocState::Degraded) {
        const SE3 raw_lidar_pose = lio_->ImuPoseToLidarPose(state.GetPose());
        const SE3 smoothed_lidar_pose = smoother_.UpdateFastLioPose(raw_lidar_pose);
        const SE3 smoother_delta = smoothed_lidar_pose.inverse() * raw_lidar_pose;
        const bool output_jump_rejected =
            smoother_delta.translation().norm() > 1e-6 || smoother_delta.so3().log().norm() > 1e-6;
        state.SetPose(lio_->LidarPoseToImuPose(smoothed_lidar_pose));

        const auto prev = state_machine_.State();
        state_machine_.ObserveTrackingQuality(lio_->TrackingQualityGood() && !output_jump_rejected);
        if (state_machine_.State() == LiteLocState::Lost && prev != LiteLocState::Lost) {
            // Lost 进入时刻, 供 lost_timeout_sec 计时 (不立即转 WAIT)
            lost_enter_ts_ = ts;
            RCLCPP_WARN(this->get_logger(), "tracking lost (reason=%s), waiting %.1f s before WAIT_FOR_INITIALPOSE",
                        state_machine_.Reason().c_str(), lost_timeout_sec_);
            // Arm SC for LOST recovery
            if (reloc_->AutoOnLost() && reloc_->ScEnabled()) {
                reloc_->Arm("lost", this->now().seconds());
                RCLCPP_INFO(this->get_logger(), "SC armed for LOST recovery");
            }
        }

        // Good 态低频 NDT 漂移校正
        if (state_machine_.State() == LiteLocState::Good && !output_jump_rejected) {
            MaybeNdtCorrectGood(scan, state, ts);
        } else if (output_jump_rejected) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Fast-LIO output jump rejected by smoother, mark tracking bad");
        }

        PublishPose(state);
    } else if (loc_state == LiteLocState::Initializing) {
        // 稳定门控期 (照 lightning 方案 B): TF/odom 照发, loc_state 保持 Initializing,
        // 滑窗内位姿抖动收敛 (或 NDT TP 提前放行) 后才切 Good.
        // 只有 /initialpose 验证通过后才会走到这里 (验证期间 has_pending_init_pose_ 已提前 return).
        const SE3 raw_lidar_pose = lio_->ImuPoseToLidarPose(state.GetPose());
        const SE3 smoothed_lidar_pose = smoother_.UpdateFastLioPose(raw_lidar_pose);
        const SE3 smoother_delta = smoothed_lidar_pose.inverse() * raw_lidar_pose;
        const bool output_jump_rejected =
            smoother_delta.translation().norm() > 1e-6 || smoother_delta.so3().log().norm() > 1e-6;
        state.SetPose(lio_->LidarPoseToImuPose(smoothed_lidar_pose));
        if (!output_jump_rejected && stability_gate_.Observe(ts, smoothed_lidar_pose, last_ndt_confidence_)) {
            state_machine_.SetGood("stability_gate_released");
            last_ndt_correct_ts_ = ts;  // Good 态 NDT 漂移校正从放行时刻起计时
            if (reloc_->Armed() && reloc_->DisableAfterGood()) {
                reloc_->Disarm("stability_gate_released");
            }
            RCLCPP_INFO(this->get_logger(), "stability gate released, switch to GOOD (TP=%.3f)",
                        last_ndt_confidence_);
        } else if (output_jump_rejected) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Fast-LIO output jump rejected during initialization");
        }
        PublishPose(state);
    } else if (loc_state == LiteLocState::Lost) {
        // Lost 持续超过 lost_timeout_sec 才转 WAIT_FOR_INITIALPOSE
        if (lost_enter_ts_ < 0.0) {
            lost_enter_ts_ = ts;
        }
        if (ts - lost_enter_ts_ > lost_timeout_sec_) {
            RCLCPP_WARN(this->get_logger(), "lost for %.1f s (> %.1f s), switch to WAIT_FOR_INITIALPOSE",
                        ts - lost_enter_ts_, lost_timeout_sec_);
            state_machine_.SetWaitForInitialPose("lost_timeout");
            reloc_->Disarm("lost_timeout");
            lost_enter_ts_ = -1.0;
            PublishStatusTopics(ts);  // 转 WAIT 立即上报一次, 之后交给 1Hz timer
            return;
        }

        // SC recovery in LOST state: attempt if armed and auto_on_lost
        if (reloc_->Armed() && reloc_->AutoOnLost() && reloc_->ScEnabled()) {
            TryScRelocalize(scan, ts);
        }

        // Publish degraded pose (LIO still running, downstream may need odom)
        if (state_machine_.State() == LiteLocState::Lost) {
            PublishPose(state);
        }
    } else if (loc_state == LiteLocState::WaitForInitialPose || loc_state == LiteLocState::Uninitialized) {
        // SC auto-on-init: attempt if armed and LIO has output
        if (reloc_->Armed() && reloc_->AutoOnInit() && reloc_->ScEnabled()) {
            TryScRelocalize(scan, ts);
        }
    }
    // WaitForInitialPose / Uninitialized: 无可信位姿, 只上报状态.
    // WAIT 态按契约 1Hz 节流持续上报 (由 wall timer 负责, 帧不产出时也能上报), 帧路径不再重复发布
    if (state_machine_.State() != LiteLocState::WaitForInitialPose) {
        PublishStatusTopics(ts);
    }
}

void LocLiteNode::HandlePendingInitPose(const CloudPtr& scan, double ts) {
    CloudPtr validate_scan = scan;
    if (init_accum_enabled_) {
        const bool ready = AccumulateInitScan(scan, ts);
        const double total_wait = init_accum_first_ts_ >= 0.0 ? ts - init_accum_first_ts_ : 0.0;
        if (!ready) {
            if (init_accum_first_ts_ >= 0.0 && total_wait >= init_accum_max_wait_sec_) {
                RCLCPP_WARN(this->get_logger(),
                            "[FC-Init] accumulation timeout: wait=%.2fs, frames=%d, points=%d; rearm /initialpose",
                            total_wait, init_accum_frames_, init_accum_points_);
                has_pending_init_pose_ = false;
                init_retry_count_ = 0;
                ResetInitAccumulation();
                state_machine_.SetWaitForInitialPose("init_accum_timeout");
            }
            return;
        }
        validate_scan = BuildInitAccumulatedCloud();
        RCLCPP_INFO(this->get_logger(),
                    "[FC-Init] run accumulated validation: attempt=%d, wait=%.2fs, frames=%d, raw_points=%d, "
                    "filtered_points=%zu",
                    init_retry_count_ + 1, total_wait, init_accum_frames_, init_accum_points_,
                    validate_scan ? validate_scan->size() : 0);
    }

    const NdtResult res = ndt_->Validate(pending_init_pose_, validate_scan);
    if (res.valid) {
        const int retries = init_retry_count_;
        lio_->ResetToMapPose(res.pose);
        smoother_.Reset();
        last_ndt_confidence_ = res.confidence;
        has_pending_init_pose_ = false;
        init_retry_count_ = 0;
        ResetInitAccumulation();
        // ResetToMapPose 后 LIO 位姿域断裂, 防御性清空 SC 累积缓冲
        // (当前 /initialpose 回调已 disarm → 缓冲为空, 此处保证不变量不依赖 disarm 策略)
        reloc_->ClearAccumulation();
        lost_enter_ts_ = -1.0;
        last_ndt_correct_ts_ = ts;  // 刚做过 NDT, Good 态漂移校正从现在起计时
        marker_pose_ = res.pose;
        if (stability_gate_enabled_) {
            // 稳定门控: 验证通过先停在 Initializing (TF/odom 照发), 滑窗收敛或高 TP 提前才放行 Good
            stability_gate_.Reset();
            state_machine_.SetInitializing("external_pose_validated");
            RCLCPP_INFO(this->get_logger(),
                        "init with external pose: [%.3f, %.3f, %.3f] (after %d retries), TP=%.3f, "
                        "stability gate armed (Initializing)",
                        res.pose.translation().x(), res.pose.translation().y(), res.pose.translation().z(),
                        retries, res.confidence);
        } else {
            // 门控关闭: 维持旧行为, 验证通过即 Good
            state_machine_.SetGood("external_pose_validated");
            RCLCPP_INFO(this->get_logger(), "init with external pose: [%.3f, %.3f, %.3f] (after %d retries)",
                        res.pose.translation().x(), res.pose.translation().y(), res.pose.translation().z(),
                        retries);
        }
        return;
    }

    ++init_retry_count_;
    if (!init_accum_enabled_ && init_retry_count_ >= init_max_retries_) {
        RCLCPP_WARN(this->get_logger(), "[FC-Init] /initialpose 重试 %d 次仍失败, 放弃, 通知上层 rearm",
                    init_max_retries_);
        has_pending_init_pose_ = false;
        init_retry_count_ = 0;
        ResetInitAccumulation();
        state_machine_.SetWaitForInitialPose("init_retry_exhausted");
    } else if (init_accum_enabled_) {
        const double total_wait = init_accum_first_ts_ >= 0.0 ? ts - init_accum_first_ts_ : 0.0;
        if (total_wait >= init_accum_max_wait_sec_) {
            RCLCPP_WARN(this->get_logger(),
                        "[FC-Init] accumulated FC rejected after %.2fs/%d attempts, rearm /initialpose",
                        total_wait, init_retry_count_);
            has_pending_init_pose_ = false;
            init_retry_count_ = 0;
            ResetInitAccumulation();
            state_machine_.SetWaitForInitialPose("init_accum_rejected");
            return;
        }
        RCLCPP_WARN(this->get_logger(),
                    "[FC-Init] accumulated FC rejected, clear window and keep accumulating (attempt=%d, wait=%.2fs)",
                    init_retry_count_, total_wait);
        init_accum_cloud_->clear();
        init_accum_frames_ = 0;
        init_accum_points_ = 0;
        init_accum_window_start_ts_ = -1.0;
    } else {
        RCLCPP_WARN(this->get_logger(), "[FC-Init] FC 拒绝本帧, 保留 init pose 用下一帧重试 (%d/%d)",
                    init_retry_count_, init_max_retries_);
    }
}

bool LocLiteNode::UpdatePendingInitPoseWithGravity() {
    if (!has_pending_init_pose_ || !lio_has_output_ || !lio_) {
        return false;
    }
    if (pending_init_pose_gravity_ready_) {
        return true;
    }

    const Mat3d R_lidar = lio_->LatestPose().so3().matrix();
    const double roll_lio = std::atan2(R_lidar(2, 1), R_lidar(2, 2));
    const double pitch_lio = std::asin(-std::clamp(R_lidar(2, 0), -1.0, 1.0));

    const Eigen::AngleAxisd yaw_aa(pending_init_yaw_, Vec3d::UnitZ());
    const Eigen::AngleAxisd pitch_aa(pitch_lio, Vec3d::UnitY());
    const Eigen::AngleAxisd roll_aa(roll_lio, Vec3d::UnitX());
    const Mat3d R_map_base = (yaw_aa * pitch_aa * roll_aa).toRotationMatrix();
    const SE3 T_map_base(SO3(R_map_base), pending_init_base_position_);
    pending_init_pose_ = T_map_base * T_lidar_base_.inverse();
    marker_pose_ = pending_init_pose_;
    pending_init_pose_gravity_ready_ = true;

    RCLCPP_INFO(this->get_logger(),
                "[FC-Init] gravity compensation ready: roll_lidar=%.1f deg, pitch_lidar=%.1f deg, "
                "candidate=[%.3f, %.3f, %.3f], yaw=%.1f deg",
                roll_lio * 180.0 / M_PI, pitch_lio * 180.0 / M_PI,
                pending_init_pose_.translation().x(), pending_init_pose_.translation().y(),
                pending_init_pose_.translation().z(), pending_init_yaw_ * 180.0 / M_PI);
    return true;
}

void LocLiteNode::ResetInitAccumulation() {
    if (!init_accum_cloud_) {
        init_accum_cloud_.reset(new PointCloudType());
    }
    init_accum_cloud_->clear();
    init_accum_frames_ = 0;
    init_accum_points_ = 0;
    init_accum_first_ts_ = -1.0;
    init_accum_window_start_ts_ = -1.0;
}

bool LocLiteNode::AccumulateInitScan(const CloudPtr& scan, double ts) {
    if (!scan || scan->empty()) {
        return false;
    }
    if (!init_accum_cloud_) {
        init_accum_cloud_.reset(new PointCloudType());
    }
    if (init_accum_first_ts_ < 0.0) {
        init_accum_first_ts_ = ts;
    }
    if (init_accum_window_start_ts_ < 0.0) {
        init_accum_window_start_ts_ = ts;
    }

    init_accum_cloud_->points.insert(init_accum_cloud_->points.end(), scan->points.begin(), scan->points.end());
    init_accum_cloud_->width = init_accum_cloud_->size();
    init_accum_cloud_->height = 1;
    init_accum_cloud_->is_dense = false;
    ++init_accum_frames_;
    init_accum_points_ += static_cast<int>(scan->size());

    const double window_elapsed = ts - init_accum_window_start_ts_;
    const bool enough_frames = init_accum_frames_ >= init_accum_min_frames_;
    const bool enough_points = init_accum_points_ >= init_accum_min_points_;
    const bool enough_time = window_elapsed >= init_accum_window_sec_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "[FC-Init] accumulating: frames=%d/%d, points=%d/%d, window=%.2f/%.2fs",
                         init_accum_frames_, init_accum_min_frames_, init_accum_points_,
                         init_accum_min_points_, window_elapsed, init_accum_window_sec_);
    return enough_frames && enough_points && enough_time;
}

CloudPtr LocLiteNode::BuildInitAccumulatedCloud() const {
    CloudPtr filtered(new PointCloudType());
    if (!init_accum_cloud_ || init_accum_cloud_->empty()) {
        return filtered;
    }
    if (init_accum_voxel_leaf_ <= 0.0) {
        *filtered = *init_accum_cloud_;
        return filtered;
    }

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(init_accum_voxel_leaf_, init_accum_voxel_leaf_, init_accum_voxel_leaf_);
    voxel.setInputCloud(init_accum_cloud_);
    voxel.filter(*filtered);
    return filtered;
}

void LocLiteNode::MaybeNdtCorrectGood(const CloudPtr& scan, NavState& state, double ts) {
    if (ndt_good_rate_hz_ <= 0.0) {
        return;
    }
    // 按时间间隔判断 (而非帧数), bag 回退时重置节流
    const double period = 1.0 / ndt_good_rate_hz_;
    if (last_ndt_correct_ts_ >= 0.0 && ts >= last_ndt_correct_ts_ && ts - last_ndt_correct_ts_ < period) {
        return;
    }
    last_ndt_correct_ts_ = ts;

    const SE3 fast_pose = lio_->ImuPoseToLidarPose(state.GetPose());
    const NdtResult res = ndt_->Align(scan, fast_pose);
    if (!res.valid) {
        return;  // 未收敛 / scan 过稀: 本周期跳过, 不打扰 LIO
    }
    last_ndt_confidence_ = res.confidence;

    // Phase2: 退化检测 — 退化场景下 NDT 可能收敛到错误解 (长走廊/平面/开阔地), 跳过本次校正
    if (res.degenerate) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[NDT-Good] correction skipped (degenerate): trans_cr=%.1f, rot_cr=%.1f",
                             res.trans_condition_ratio, res.rot_condition_ratio);
        return;
    }

    // TP 下限守门 (与 Validate 同口径): 低置信度校正宁可跳过等下个周期,
    // 坏校正比漂移更伤; delta 上限由 smoother 的 max_correction_* 兜底
    if (res.confidence < ndt_->MinConfidence()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[NDT-Good] correction skipped: TP=%.3f < min_confidence=%.3f (inlier=%.3f)",
                             res.confidence, ndt_->MinConfidence(), res.inlier_ratio);
        return;
    }

    // inlier 覆盖率正交门 (与 Validate 同口径; MinInlierRatio()<=0 时关闭): TP 量纲塌陷时兜底防伪匹配
    if (ndt_->MinInlierRatio() > 0.0 && res.inlier_ratio < ndt_->MinInlierRatio()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[NDT-Good] correction skipped: inlier=%.3f < min_inlier_ratio=%.3f (TP=%.3f)",
                             res.inlier_ratio, ndt_->MinInlierRatio(), res.confidence);
        return;
    }

    // 增益混合 + delta 门限 (门限在 smoother 内部, 被拒时原样返回 fast_pose)
    const SE3 corrected = smoother_.ApplyNdtCorrection(fast_pose, res.pose, ndt_gain_good_);
    const SE3 delta = fast_pose.inverse() * corrected;
    if (delta.translation().norm() < 1e-6 && delta.so3().log().norm() < 1e-6) {
        return;  // 校正被门限拒绝或可忽略, 不打扰 LIO
    }

    lio_->ResetToMapPose(corrected, false);
    smoother_.Reset();
    state.SetPose(lio_->LidarPoseToImuPose(corrected));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[NDT-Good] drift corrected: dt=%.3f m, dr=%.2f deg, confidence=%.3f, inlier=%.3f",
                         delta.translation().norm(), delta.so3().log().norm() * 180.0 / M_PI,
                         res.confidence, res.inlier_ratio);
}

void LocLiteNode::PublishPose(const NavState& state) {
    const double ts = state.timestamp_;
    const rclcpp::Time stamp = ToRosTime(ts);
    const SE3 T_map_lidar = lio_->ImuPoseToLidarPose(state.GetPose());
    // 补偿 base_link → lidar 外参: T_map_base = T_map_lidar × T_lidar_base
    const SE3 T_map_base = T_map_lidar * T_lidar_base_;
    marker_pose_ = T_map_lidar;

    // odom: map 下的雷达位姿
    if (publish_odom_) {
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = stamp;
        odom_msg->header.frame_id = map_frame_id_;
        odom_msg->child_frame_id = lidar_frame_id_;
        odom_msg->pose.pose = ToPoseMsg(T_map_lidar);
        odom_pub_->publish(std::move(odom_msg));
    }

    // TF map → base_link
    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = map_frame_id_;
        tf_msg.child_frame_id = base_frame_id_;
        tf_msg.transform.translation.x = T_map_base.translation().x();
        tf_msg.transform.translation.y = T_map_base.translation().y();
        tf_msg.transform.translation.z = T_map_base.translation().z();
        const auto q = T_map_base.unit_quaternion();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();
        tf_pub_->sendTransform(tf_msg);
    }

    // path: 0.1s 节流追加, 最多保存 kMaxPathPoses 个位姿 (超出从头部裁剪)
    if (publish_path_ &&
        (last_path_pub_ts_ <= 0.0 || ts < last_path_pub_ts_ || ts - last_path_pub_ts_ >= 0.1)) {
        last_path_pub_ts_ = ts;
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = map_frame_id_;
        ps.pose = ToPoseMsg(T_map_base);
        path_msg_.header.stamp = stamp;
        path_msg_.header.frame_id = map_frame_id_;
        path_msg_.poses.push_back(ps);
        while (path_msg_.poses.size() > kMaxPathPoses) {
            path_msg_.poses.erase(path_msg_.poses.begin());
        }
        path_pub_->publish(path_msg_);
    }
}

void LocLiteNode::PublishStatusTopics(double ts) {
    // loc_state: 状态机枚举整数 0-5
    std_msgs::msg::Int32 state_msg;
    state_msg.data = static_cast<int>(state_machine_.State());
    loc_state_pub_->publish(state_msg);

    // ndt_status: 当前固定 0 (默认 1.0m 单层 NDT); 1 保留给级联模式
    std_msgs::msg::Int32 ndt_msg;
    ndt_msg.data = 0;
    ndt_status_pub_->publish(ndt_msg);

    PublishStatusMarker(ts);
}

void LocLiteNode::PublishStatusMarker(double ts) {
    // 文字与颜色照抄 lightning loc_system.cc 的 loc_status marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_id_;
    marker.header.stamp = ToRosTime(ts);
    marker.ns = "loc_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = marker_pose_.translation().x();
    marker.pose.position.y = marker_pose_.translation().y();
    marker.pose.position.z = marker_pose_.translation().z() + 2.0;  // 悬浮在上方
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 1.5;  // text size

    const auto state = state_machine_.State();
    if (state == LiteLocState::Good) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.text = "GOOD (" + std::to_string(last_ndt_confidence_).substr(0, 4) + ")";
    } else if (state == LiteLocState::Degraded) {
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.text = "DEGRADED (" + std::to_string(last_ndt_confidence_).substr(0, 4) + ")";
    } else if (state == LiteLocState::Initializing) {
        // 初始化中: 蓝色提示, 区别于红色 LOST
        marker.color.r = 0.2f;
        marker.color.g = 0.6f;
        marker.color.b = 1.0f;
        marker.text = "INIT (" + std::to_string(last_ndt_confidence_).substr(0, 4) + ")";
    } else if (state == LiteLocState::WaitForInitialPose) {
        // 等待外部 /initialpose: 紫色, 与 LOST 红 / INIT 蓝 区分
        marker.color.r = 1.0f;
        marker.color.g = 0.2f;
        marker.color.b = 1.0f;
        marker.text = "WAIT FOR /initialpose";
    } else {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.text = "LOST";
    }
    marker.color.a = 1.0;
    loc_status_marker_pub_->publish(marker);
}

void LocLiteNode::OnHealthTimer() {
    std::lock_guard<std::mutex> lk(mutex_);

    const double now_wall = this->now().seconds();
    const double imu_age =
        last_imu_wall_ts_ < 0.0 ? std::numeric_limits<double>::infinity() : now_wall - last_imu_wall_ts_;
    const double lidar_age =
        last_lidar_wall_ts_ < 0.0 ? std::numeric_limits<double>::infinity() : now_wall - last_lidar_wall_ts_;

    // 输入掉线检测: 仅在 LIO 已产出且处于"应有数据流"的活跃态时判定
    // (WAIT/Uninitialized 本就没数据流, 不该误报). 掉线 → 告警 + 置 LOST, 不 reset Fast-LIO 状态.
    if (watchdog_enabled_ && lio_has_output_) {
        const auto st = state_machine_.State();
        const bool active = st == LiteLocState::Good || st == LiteLocState::Degraded ||
                            st == LiteLocState::Initializing || st == LiteLocState::Lost;
        if (active) {
            const bool stale = imu_age > imu_timeout_sec_ || lidar_age > lidar_timeout_sec_;
            if (stale) {
                if (input_stale_since_wall_ < 0.0) {
                    input_stale_since_wall_ = now_wall;
                }
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "input dropout: imu_age=%.2fs (>%.2f) lidar_age=%.2fs (>%.2f), stale for %.1fs",
                    imu_age, imu_timeout_sec_, lidar_age, lidar_timeout_sec_,
                    now_wall - input_stale_since_wall_);
                if (st != LiteLocState::Lost) {
                    state_machine_.SetLost("input_timeout");
                    lost_enter_ts_ = -1.0;  // 重置, 输入恢复后由 ProcessFrame 的 Lost 分支按 scan ts 重新计时
                    if (reloc_->AutoOnLost() && reloc_->ScEnabled()) {
                        reloc_->Arm("input_timeout", now_wall);
                    }
                    RCLCPP_ERROR(this->get_logger(),
                                 "input dropout (imu_age=%.2fs, lidar_age=%.2fs) -> LOST", imu_age,
                                 lidar_age);
                }
            } else if (input_stale_since_wall_ >= 0.0) {
                RCLCPP_INFO(this->get_logger(), "input recovered (imu_age=%.2fs, lidar_age=%.2fs)",
                            imu_age, lidar_age);
                input_stale_since_wall_ = -1.0;
            }
        }
    }

    if (status_topic_enabled_) {
        PublishRichStatus(now_wall, imu_age, lidar_age);
    }
}

void LocLiteNode::PublishRichStatus(double /*now_wall*/, double imu_age, double lidar_age) {
    const auto st = state_machine_.State();
    // in_map: 当前是否在固定图内被可信定位 (Good/Degraded). 非 Good/Degraded 时无可信位姿.
    const bool in_map = st == LiteLocState::Good || st == LiteLocState::Degraded;

    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "state;ndt_conf;imu_age_s;lidar_age_s;fps;in_map";
    msg.layout.dim[0].size = 6;
    msg.layout.dim[0].stride = 6;
    // age 为 inf 时上报 -1 (下游易判: 负值=尚无该输入或已长时间掉线)
    const float imu_age_f = std::isfinite(imu_age) ? static_cast<float>(imu_age) : -1.0f;
    const float lidar_age_f = std::isfinite(lidar_age) ? static_cast<float>(lidar_age) : -1.0f;
    msg.data = {static_cast<float>(static_cast<int>(st)),
                static_cast<float>(last_ndt_confidence_),
                imu_age_f,
                lidar_age_f,
                static_cast<float>(frame_fps_),
                in_map ? 1.0f : 0.0f};
    status_pub_->publish(msg);
}

void LocLiteNode::PublishPcdMap() {
    if (!publish_pcdmap_) {
        return;
    }
    if (!pcd_map_pub_) {
        RCLCPP_WARN(this->get_logger(), "/pcdmap publisher is not ready");
        return;
    }
    const CloudPtr map_cloud = lio_ ? lio_->FixedMapCloud() : nullptr;
    if (!map_cloud || map_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "skip /pcdmap publish: fixed map cloud is empty");
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_cloud, msg);
    msg.header.stamp = this->now();
    msg.header.frame_id = map_frame_id_;
    pcd_map_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published /pcdmap: frame=%s, points=%zu", map_frame_id_.c_str(),
                map_cloud->size());
}

void LocLiteNode::TryScRelocalize(const CloudPtr& scan, double ts, bool manual) {
    // Cooldown check: avoid running SC every frame (skip for manual requests)
    const double cooldown = reloc_->ScCooldownSec();
    if (!manual && last_sc_attempt_ts_ >= 0.0 && ts - last_sc_attempt_ts_ < cooldown) {
        return;
    }

    // Blackout check: block automatic SC during /initialpose blackout window
    // 时钟域: deadline 由 /initialpose 回调用墙钟设定, 比较必须同为墙钟.
    // bag 回放时 scan ts 远小于墙钟, 若用 ts 比较 blackout 永真, SC 自动注入被永久阻断.
    const double wall_now = this->now().seconds();
    if (!manual && wall_now < blackout_deadline_sec_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[SC-Reloc] in blackout window (wall_now=%.3f < deadline=%.3f), skipping SC attempt",
                             wall_now, blackout_deadline_sec_);
        return;
    }

    // 帧数不足的 skip 不消耗 cooldown (C2): last_sc_attempt_ts_ 只在真正执行 QueryTopK 时写入.
    // 否则 Arm 后首次尝试 (frames=1/N) 即烧掉一个 cooldown 周期, 缓冲攒满 (~2s) 后还要再等
    // 5s, LOST 窗口 (lost_timeout_sec=5s) 内抢不到一次真查询. 此处提前判帧数, 不足直接 return,
    // 缓冲由逐帧 AccumulateScan 继续填充, 攒满即查. (manual 走 RunScanContextOnce 单帧回退)
    if (!manual && reloc_->ScAccumFrames() > 1 &&
        reloc_->AccumulatedFrames() < reloc_->ScAccumFrames()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "[SC-Reloc] accumulating frames=%d/%d, skip without consuming cooldown",
                             reloc_->AccumulatedFrames(), reloc_->ScAccumFrames());
        return;
    }
    last_sc_attempt_ts_ = ts;

    const SE3 current_imu_pose = lio_->LatestState().GetPose();
    RelocCandidate candidate;
    if (manual) {
        candidate = reloc_->ManualRelocalize(scan, current_imu_pose);
    } else {
        candidate = reloc_->TryRelocalize(scan, current_imu_pose, wall_now);
    }

    PublishScDebugTopics();

    if (!candidate.valid) {
        return;
    }

    // NDT validation: SC 候选用 reloc.sc_max_delta_* 专用 delta 门限 (C1) —
    // SC sector 分辨率 6° 量化 + 关键帧间距使正确候选 delta 常超 ndt.* 门限 (/initialpose 量纲)
    const NdtResult ndt_res =
        ndt_->Validate(candidate.pose, scan, reloc_->ScMaxDeltaTransM(), reloc_->ScMaxDeltaRotDeg());
    if (!ndt_res.valid) {
        RCLCPP_WARN(this->get_logger(),
                    "[SC-Reloc] SC candidate kf_id=%d rejected by NDT "
                    "(conf=%.3f, dt=%.3f m, dr=%.2f deg)",
                    candidate.kf_id, ndt_res.confidence, ndt_res.delta_trans_m, ndt_res.delta_rot_deg);
        return;
    }

    // Accept: reset to NDT-refined pose
    lio_->ResetToMapPose(ndt_res.pose);
    smoother_.Reset();
    last_ndt_confidence_ = ndt_res.confidence;
    marker_pose_ = ndt_res.pose;
    lost_enter_ts_ = -1.0;
    last_ndt_correct_ts_ = ts;
    has_pending_init_pose_ = false;
    init_retry_count_ = 0;
    ResetInitAccumulation();
    // ResetToMapPose 后 LIO 位姿域断裂, 稳定门控分支下 reloc 仍 armed, 必须清空 SC 累积缓冲
    reloc_->ClearAccumulation();

    if (stability_gate_enabled_) {
        stability_gate_.Reset();
        state_machine_.SetInitializing("sc_validated");
        RCLCPP_INFO(this->get_logger(),
                    "[SC-Reloc] SC accepted: kf_id=%d, pos=[%.3f, %.3f, %.3f], TP=%.3f, stability gate armed",
                    candidate.kf_id, ndt_res.pose.translation().x(),
                    ndt_res.pose.translation().y(), ndt_res.pose.translation().z(),
                    ndt_res.confidence);
    } else {
        state_machine_.SetGood("sc_validated");
        if (reloc_->DisableAfterGood()) {
            reloc_->Disarm("sc_validated");
        }
        RCLCPP_INFO(this->get_logger(),
                    "[SC-Reloc] SC accepted: kf_id=%d, pos=[%.3f, %.3f, %.3f], TP=%.3f, GOOD",
                    candidate.kf_id, ndt_res.pose.translation().x(),
                    ndt_res.pose.translation().y(), ndt_res.pose.translation().z(),
                    ndt_res.confidence);
    }
}

void LocLiteNode::PublishScDebugTopics() {
    const auto& debug = reloc_->LastDebugInfo();

    // SC accumulated query cloud (level 系: 合并到最新帧 body 系后重力对齐, 即 QueryTopK 实际输入)
    if (sc_accum_cloud_pub_->get_subscription_count() > 0) {
        const CloudPtr query_cloud = reloc_->LastQueryCloud();
        if (query_cloud && !query_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*query_cloud, msg);
            msg.header.stamp = this->now();
            msg.header.frame_id = level_frame_id_;
            sc_accum_cloud_pub_->publish(msg);
        }
    }

    // SC init_guess (winning candidate pose)
    if (sc_init_guess_pub_->get_subscription_count() > 0 && debug.best.valid) {
        auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        msg->header.stamp = this->now();
        msg->header.frame_id = map_frame_id_;
        msg->pose = ToPoseMsg(debug.best.pose);
        sc_init_guess_pub_->publish(std::move(msg));
    }

    // SC candidates (MarkerArray: green=winner, white=others)
    if (sc_candidates_pub_->get_subscription_count() > 0 && !debug.candidates.empty()) {
        auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
        int id = 0;
        for (const auto& c : debug.candidates) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = map_frame_id_;
            m.header.stamp = this->now();
            m.ns = "sc_candidates";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 1.0;
            m.scale.y = 0.2;
            m.scale.z = 0.2;
            m.color.a = 0.8f;
            // Green for winner, white for others
            if (debug.best.valid && c.kf_id == debug.best.kf_id) {
                m.color.r = 0.0f;
                m.color.g = 1.0f;
                m.color.b = 0.0f;
            } else {
                m.color.r = 1.0f;
                m.color.g = 1.0f;
                m.color.b = 1.0f;
            }
            // Place at the keyframe's map-frame position with yaw_diff orientation
            const SE3 kf_pose = reloc_->KfPose(c.kf_id);
            m.pose.position.x = kf_pose.translation().x();
            m.pose.position.y = kf_pose.translation().y();
            m.pose.position.z = kf_pose.translation().z();
            // Apply SC yaw_diff to keyframe orientation
            const Mat3d R_kf = kf_pose.so3().matrix();
            const double kf_yaw = std::atan2(R_kf(1, 0), R_kf(0, 0));
            const double corrected_yaw = kf_yaw + static_cast<double>(c.yaw_diff_rad);
            const Eigen::AngleAxisd yaw_aa(corrected_yaw, Vec3d::UnitZ());
            const auto q = Quatd(yaw_aa);
            m.pose.orientation.x = q.x();
            m.pose.orientation.y = q.y();
            m.pose.orientation.z = q.z();
            m.pose.orientation.w = q.w();
            markers->markers.push_back(m);
        }
        sc_candidates_pub_->publish(std::move(markers));
    }

    // SC gravity check
    if (sc_gravity_check_pub_->get_subscription_count() > 0) {
        auto msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
        msg->data = {debug.roll_err_deg, debug.pitch_err_deg, debug.gravity_passed ? 1.0f : 0.0f};
        sc_gravity_check_pub_->publish(std::move(msg));
    }
}

void LocLiteNode::PublishLevelFrameTF(const NavState& state) {
    // 本包 LIO 状态只在每帧 lidar 后更新 (没有 lightning GetIMUState 那样的逐 IMU 传播状态);
    // IMU 速率远高于 lidar, 时间戳未推进时直接跳过, 避免以相同 stamp 重复广播 TF
    // (下游 tf2 会刷 TF_REPEATED_DATA 警告), 也避免 dt=0 的无效滤波器更新
    if (state.timestamp_ == last_level_tf_ts_) {
        return;
    }
    last_level_tf_ts_ = state.timestamp_;

    // R_world_lidar = R_world_imu * R_imu_lidar
    const Mat3d R_world_lidar = state.rot_.matrix() * cached_extrinsic_R_;

    // 通过低通滤波计算水平对齐旋转 R_level_lidar
    const SO3 R_level_lidar = gravity_filter_.Update(state.grav_, R_world_lidar, state.timestamp_);

    // 为保持 TF 树连通, level_frame 作为 lidar_frame 的子节点发布 (平移为 0 的纯旋转).
    // 发布的是 level_frame 在 lidar_frame 中的位姿, 即 R_lidar_level = R_level_lidar^(-1).
    // 下游 lookupTransform(level_frame, lidar_frame) 即可得到 R_level_lidar.
    geometry_msgs::msg::TransformStamped tf_msg;
    const auto seconds = static_cast<int32_t>(state.timestamp_);
    tf_msg.header.stamp.sec = seconds;
    tf_msg.header.stamp.nanosec = static_cast<uint32_t>((state.timestamp_ - seconds) * 1e9);
    tf_msg.header.frame_id = lidar_frame_id_;
    tf_msg.child_frame_id = level_frame_id_;

    const auto q = R_level_lidar.inverse().unit_quaternion();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;

    tf_pub_->sendTransform(tf_msg);
}

}  // namespace hikari::loclite
