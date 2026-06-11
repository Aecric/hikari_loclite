#include "system/loclite_node.hpp"
#include "math/loclite_math.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
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
    double voxel_leaf = 0.2;
    double level_cutoff = 0.5;
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
        }
        if (yaml["reloc"]) {
            init_max_retries_ = yaml["reloc"]["init_max_retries"].as<int>(5);
        }
        if (yaml["runtime"]) {
            auto rt = yaml["runtime"];
            publish_tf_ = rt["publish_tf"].as<bool>(true);
            publish_odom_ = rt["publish_odom"].as<bool>(true);
            publish_path_ = rt["publish_path"].as<bool>(true);
            publish_pcdmap_ = rt["publish_pcdmap"].as<bool>(false);
        }
        if (yaml["ndt"]) {
            ndt_good_rate_hz_ = yaml["ndt"]["good_rate_hz"].as<double>(1.0);
            ndt_gain_good_ = yaml["ndt"]["gain_good"].as<double>(0.5);
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
    ndt_->SetMap(lio_->FixedMapCloud());

    // 重力对齐滤波器 (lidar_frame → level_frame 纯旋转 TF)
    GravityAlignmentFilter::Options gf_options;
    gf_options.cutoff_freq = level_cutoff;
    gf_options.lidar_frame_id = lidar_frame_id_;
    gf_options.level_frame_id = level_frame_id_;
    gravity_filter_.Init(gf_options);
    RCLCPP_INFO(this->get_logger(), "GravityAlignmentFilter: cutoff_freq=%.2f Hz, lidar_frame=%s, level_frame=%s",
                level_cutoff, lidar_frame_id_.c_str(), level_frame_id_.c_str());

    // 初始状态: 等待外部 /initialpose (SC 自动重定位接入前的唯一初始化入口)
    state_machine_.SetWaitForInitialPose("startup");

    // Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::QoS(100),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) { OnImu(std::move(msg)); });

    livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic, rclcpp::QoS(5).best_effort(),
        [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { OnLivox(std::move(msg)); });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::QoS(5).best_effort(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { OnCloud(std::move(msg)); });

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

    // SC 重定位调试话题: 先建话题面 (QoS 1, 仅在有订阅者时发布)
    // TODO(P5): SC 重定位管线 (RelocManager) 接入后, 按 lightning SetScDebugCallback 的格式
    //           发布累积点云 / 候选 MarkerArray / 初始猜测 / 重力检查, 当前不发布
    sc_accum_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("hikari_loc/sc/accum_cloud", 1);
    sc_candidates_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("hikari_loc/sc/candidates", 1);
    sc_init_guess_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("hikari_loc/sc/init_guess", 1);
    sc_gravity_check_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("hikari_loc/sc/gravity_check", 1);

    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // 手动 SC 重定位服务
    sc_reloc_service_ = create_service<std_srvs::srv::Trigger>(
        "hikari_loc/sc_reloc",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
            // 入口先打日志: 确认 service callback 真的被 executor 派发 (与 lightning 同一排障习惯)
            RCLCPP_INFO(this->get_logger(), "[SC-Reloc] /hikari_loc/sc_reloc 收到调用");
            // TODO(P5): RelocManager / SC 管线接入后改为重定位请求入队, 并 bypass /initialpose
            //           blackout 窗口 (用户显式触发即视为授权抢注, 见契约文档第 21 节)
            response->success = false;
            response->message = "SC relocalization not available (sc pipeline not wired yet)";
        });

    // WAIT_FOR_INITIALPOSE 下帧可能不产出, 用 1Hz wall timer 持续上报状态
    wait_state_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
        std::lock_guard<std::mutex> lk(mutex_);
        if (state_machine_.State() != LiteLocState::WaitForInitialPose) {
            return;
        }
        PublishStatusTopics(this->now().seconds());
    });

    RCLCPP_INFO(this->get_logger(), "hikari_loclite initialized successfully (imu=%s, livox=%s, cloud=%s)",
                imu_topic.c_str(), livox_topic.c_str(), cloud_topic.c_str());
    return true;
}

void LocLiteNode::Shutdown() {
    RCLCPP_INFO(this->get_logger(), "hikari_loclite shutting down");
}

void LocLiteNode::OnImu(sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    lio_->AddImu(msg);

    // 每条 IMU 处理后用 LIO 最新状态更新重力对齐滤波器并发布 lidar_frame → level_frame TF
    if (publish_tf_ && lio_has_output_ && tf_pub_) {
        PublishLevelFrameTF(lio_->LatestState());
    }
}

void LocLiteNode::OnLivox(livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        lio_->AddLivox(msg);
    }
    ProcessFrame();
}

void LocLiteNode::OnCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
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

    // Step 2: roll/pitch 从当前 LIO 重力对齐姿态读取, LIO 未就绪则退回 0 (冷启动, 还没有 IMU 观测)
    double roll_lio = 0.0, pitch_lio = 0.0;
    if (lio_has_output_) {
        const Mat3d R_lio = lio_->LatestState().rot_.matrix();
        roll_lio = std::atan2(R_lio(2, 1), R_lio(2, 2));
        pitch_lio = std::asin(-std::clamp(R_lio(2, 0), -1.0, 1.0));
    }

    // Step 3: R_map_base = Rz(yaw)·Ry(pitch_lio)·Rx(roll_lio)
    const Eigen::AngleAxisd yaw_aa(yaw_init, Vec3d::UnitZ());
    const Eigen::AngleAxisd pitch_aa(pitch_lio, Vec3d::UnitY());
    const Eigen::AngleAxisd roll_aa(roll_lio, Vec3d::UnitX());
    const Mat3d R_map_base = (yaw_aa * pitch_aa * roll_aa).toRotationMatrix();
    const SE3 T_map_base(SO3(R_map_base), Vec3d(p.position.x, p.position.y, p.position.z));

    // Step 4: T_map_lidar = T_map_base × T_base_lidar (T_base_lidar = inv(T_lidar_base_))
    const SE3 T_map_lidar = T_map_base * T_lidar_base_.inverse();

    // 限次重试 (sticky retry): 保存 pending pose, 由后续每帧的去畸变 scan 做 NDT 验证
    pending_init_pose_ = T_map_lidar;
    has_pending_init_pose_ = true;
    init_retry_count_ = 0;
    marker_pose_ = T_map_lidar;

    // 状态机原子复位: 清空 bad/good 计数 (SetInitializing 内) 与 lost 计时器
    state_machine_.SetInitializing("external_pose");
    lost_enter_ts_ = -1.0;

    // blackout 窗口: 记录截止时刻.
    // TODO(P5): SC 管线接入后, SC worker 入口与 inflight job 在 now < blackout_deadline_sec_ 时
    //           全部丢弃, 让用户的 pose 拥有独占试错窗口
    const double now_sec = this->now().seconds();
    blackout_deadline_sec_ = now_sec + external_pose_blackout_sec_;

    RCLCPP_INFO(this->get_logger(),
                "[ExtPose] /initialpose 已接入 (t=%.3f): pos=[%.3f, %.3f, %.3f], yaw=%.1f deg, "
                "roll_lio=%.1f deg, pitch_lio=%.1f deg, blackout 窗口至 t=%.3f",
                now_sec, p.position.x, p.position.y, p.position.z, yaw_init * 180.0 / M_PI,
                roll_lio * 180.0 / M_PI, pitch_lio * 180.0 / M_PI, blackout_deadline_sec_);
}

void LocLiteNode::ProcessFrame() {
    std::lock_guard<std::mutex> lk(mutex_);

    NavState state;
    if (!lio_->RunOnce(&state)) {
        return;
    }
    lio_has_output_ = true;

    const double ts = state.timestamp_;
    const CloudPtr scan = lio_->LatestDeskewedCloud();

    // /initialpose sticky retry: 每个新 scan 到来时验证 pending pose
    if (has_pending_init_pose_) {
        HandlePendingInitPose(scan, ts);
        PublishStatusTopics(ts);
        return;
    }

    const auto loc_state = state_machine_.State();

    if (loc_state == LiteLocState::Good || loc_state == LiteLocState::Degraded) {
        state.SetPose(smoother_.UpdateFastLioPose(state.GetPose()));

        const auto prev = state_machine_.State();
        state_machine_.ObserveTrackingQuality(lio_->TrackingQualityGood());
        if (state_machine_.State() == LiteLocState::Lost && prev != LiteLocState::Lost) {
            // Lost 进入时刻, 供 lost_timeout_sec 计时 (不立即转 WAIT)
            lost_enter_ts_ = ts;
            RCLCPP_WARN(this->get_logger(), "tracking lost (reason=%s), waiting %.1f s before WAIT_FOR_INITIALPOSE",
                        state_machine_.Reason().c_str(), lost_timeout_sec_);
        }

        // Good 态低频 NDT 漂移校正
        if (state_machine_.State() == LiteLocState::Good) {
            MaybeNdtCorrectGood(scan, state, ts);
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
            lost_enter_ts_ = -1.0;
            PublishStatusTopics(ts);  // 转 WAIT 立即上报一次, 之后交给 1Hz timer
            return;
        }
    }
    // WaitForInitialPose / Uninitialized / Initializing: 无可信位姿, 只上报状态.
    // WAIT 态按契约 1Hz 节流持续上报 (由 wall timer 负责, 帧不产出时也能上报), 帧路径不再重复发布
    if (state_machine_.State() != LiteLocState::WaitForInitialPose) {
        PublishStatusTopics(ts);
    }
}

void LocLiteNode::HandlePendingInitPose(const CloudPtr& scan, double ts) {
    const NdtResult res = ndt_->Validate(pending_init_pose_, scan);
    if (res.valid) {
        const int retries = init_retry_count_;
        lio_->ResetToMapPose(res.pose);
        smoother_.Reset();
        state_machine_.SetGood("external_pose_validated");
        last_ndt_confidence_ = res.confidence;
        has_pending_init_pose_ = false;
        init_retry_count_ = 0;
        lost_enter_ts_ = -1.0;
        last_ndt_correct_ts_ = ts;  // 刚做过 NDT, Good 态漂移校正从现在起计时
        marker_pose_ = res.pose;
        RCLCPP_INFO(this->get_logger(), "init with external pose: [%.3f, %.3f, %.3f] (after %d retries)",
                    res.pose.translation().x(), res.pose.translation().y(), res.pose.translation().z(), retries);
        return;
    }

    ++init_retry_count_;
    if (init_retry_count_ >= init_max_retries_) {
        RCLCPP_WARN(this->get_logger(), "[FC-Init] /initialpose 重试 %d 次仍失败, 放弃, 通知上层 rearm",
                    init_max_retries_);
        has_pending_init_pose_ = false;
        init_retry_count_ = 0;
        state_machine_.SetWaitForInitialPose("init_retry_exhausted");
    } else {
        RCLCPP_WARN(this->get_logger(), "[FC-Init] FC 拒绝本帧, 保留 init pose 用下一帧重试 (%d/%d)",
                    init_retry_count_, init_max_retries_);
    }
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

    const SE3 fast_pose = state.GetPose();
    const NdtResult res = ndt_->Align(scan, fast_pose);
    if (!res.valid) {
        return;
    }
    last_ndt_confidence_ = res.confidence;

    // 增益混合 + delta 门限 (门限在 smoother 内部, 被拒时原样返回 fast_pose)
    const SE3 corrected = smoother_.ApplyNdtCorrection(fast_pose, res.pose, ndt_gain_good_);
    const SE3 delta = fast_pose.inverse() * corrected;
    if (delta.translation().norm() < 1e-6 && delta.so3().log().norm() < 1e-6) {
        return;  // 校正被门限拒绝或可忽略, 不打扰 LIO
    }

    lio_->ResetToMapPose(corrected);
    smoother_.Reset();
    state.SetPose(corrected);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[NDT-Good] drift corrected: dt=%.3f m, dr=%.2f deg, confidence=%.3f",
                         delta.translation().norm(), delta.so3().log().norm() * 180.0 / M_PI, res.confidence);
}

void LocLiteNode::PublishPose(const NavState& state) {
    const double ts = state.timestamp_;
    const rclcpp::Time stamp = ToRosTime(ts);
    const SE3 T_map_lidar = state.GetPose();
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
