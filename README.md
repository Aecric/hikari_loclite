# hikari_loclite

## 中文

`hikari_loclite` 是一个轻量级 ROS 2 固定地图 LiDAR 定位包，面向嵌入式部署。它从 Livox Mid360 或通用 `PointCloud2` 点云和 IMU 输入中进行固定地图跟踪，输出 `map` 坐标系下的里程计、路径、TF 和定位状态。

项目目标不是完整 SLAM 框架，而是在已有地图上做稳定、可部署、资源可控的定位。运行链路以 Fast-LIO 固定地图跟踪为主，NDT 用于 `/initialpose`、Scan Context 候选验证和低频漂移校正，Scan Context 用于冷启动或 LOST 状态下的重定位。

### 主要能力

- 固定地图 Fast-LIO 定位，不在运行时扩展地图。
- 支持 Livox `CustomMsg` 和通用 `sensor_msgs/msg/PointCloud2` 输入。
- 支持 `/initialpose` 外部初值注入，并通过 NDT 验证后再重置定位状态。
- 支持 Scan Context 自动或手动重定位。
- 支持 GOOD / DEGRADED / LOST / WAIT_FOR_INITIALPOSE 等轻量状态机。
- 支持 `/pcdmap` 固定地图可视化、定位状态 Marker 和富状态数组。
- 支持 Release 在线部署，以及非 Release rosbag 离线评估。
- 提供 Docker buildx `.deb` 打包流程和 systemd unit 模板。

### 目录结构

```text
.
├── config/                  # 默认 YAML 配置
├── launch/                  # ROS 2 launch 文件
├── include/hikari_loclite/  # 公共类型、LIO、NDT、重定位和系统头文件
├── src/
│   ├── app/                 # run_loclite_online / run_loclite_offline 入口
│   ├── common/              # 导航状态等公共实现
│   ├── lio/                 # Fast-LIO、ESKF、预处理、Scan Context
│   ├── ndt/                 # NDT OMP 与校正/验证
│   ├── reloc/               # Scan Context 重定位管理
│   └── system/              # ROS 2 节点、状态机、实时调度
├── scripts/zupt_calib/      # ZUPT 标定辅助脚本
├── thirdparty/              # Sophus、nanoflann 等本地第三方代码
├── docker2/                 # .deb 打包 Dockerfile、postinst、systemd 模板
└── build.sh                 # Docker buildx 打包脚本
```

### 依赖

核心依赖：

- ROS 2 `ament_cmake`
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `std_srvs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `visualization_msgs`
- `pcl_conversions`
- `pcl_ros`
- `livox_ros_driver2`
- Eigen3
- PCL
- yaml-cpp
- OpenMP

`rosbag2_cpp` 只用于非 Release 构建中的 `run_loclite_offline` 离线评估节点。Release 构建不会编译或安装离线节点。

### 开源库和参考项目

本项目使用或 vendored 的 GitHub 开源库包括 [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher)、Sophus、nanoflann 等；实现上参考了 `lightning-lm` 的固定地图定位、NDT、Scan Context 和稳定门控链路，第三方代码仍遵循其各自许可证。

### 构建

在 ROS 2 工作空间中构建：

```bash
source /opt/ros/<ros_distro>/setup.bash
colcon build --base-paths src/hikari_loclite \
  --packages-select hikari_loclite \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

如果需要离线 rosbag 评估节点，使用非 Release 构建类型：

```bash
source /opt/ros/<ros_distro>/setup.bash
colcon build --base-paths src/hikari_loclite \
  --packages-select hikari_loclite \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

构建后加载环境：

```bash
source install/setup.bash
```

### 地图和配置

默认配置文件：

```text
config/loclite_livox.yaml
```

地图路径解析顺序：

1. CLI 参数 `--map_path`
2. YAML 中的 `system.map_path`
3. YAML 中的 `fixed_map.global_pcd`

当使用 `--map_path <map_dir>` 时，节点会从地图目录加载固定地图和重定位数据，例如：

```text
<map_dir>/global.pcd
<map_dir>/sc_database.bin
<map_dir>/poses.txt
```

不同地图、安装姿态和点云密度会影响 NDT 置信度阈值、Scan Context 参数和 ZUPT 阈值。上线前应根据现场数据重新标定 `config/loclite_livox.yaml` 中的相关参数。

### 在线运行

推荐使用 CLI 参数运行：

```bash
ros2 run hikari_loclite run_loclite_online \
  --config /path/to/loclite_livox.yaml \
  --map_path /path/to/map_dir
```

也可以通过 launch 文件运行：

```bash
ros2 launch hikari_loclite loclite.launch.py \
  config:=/path/to/loclite_livox.yaml
```

`run_loclite_online` 支持：

```bash
--config <yaml>
--config=<yaml>
--map_path <map_dir>
--map_path=<map_dir>
```

### 离线评估

离线节点只在非 Release 构建中可用：

```bash
ros2 run hikari_loclite run_loclite_offline \
  --config /path/to/loclite_livox.yaml \
  --input_bag /path/to/bag \
  --map_path /path/to/map_dir
```

离线节点按 rosbag 顺序读取 IMU、Livox `CustomMsg` 和 `PointCloud2` 数据，并复用在线 `LocLiteNode` 的同一套处理路径，便于回放评估和 RViz 对比。

### ROS 接口

默认订阅：

| Topic | Type | 说明 |
| --- | --- | --- |
| `/livox/imu` | `sensor_msgs/msg/Imu` | IMU 输入 |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Livox 点云输入，`fast_lio.lidar_type: 1` 时使用 |
| `/cloud` | `sensor_msgs/msg/PointCloud2` | 通用点云输入，非 Livox 类型时使用 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 外部初始位姿候选 |

默认发布：

| Topic | Type | 说明 |
| --- | --- | --- |
| `hikari_loc/odom` | `nav_msgs/msg/Odometry` | 定位里程计，pose 为 `map -> lidar_frame_id`，linear twist 为 ESKF 速度旋转到 `child_frame_id`；angular twist 和 covariance 当前不提供 |
| `hikari_loc/path` | `nav_msgs/msg/Path` | 轨迹路径 |
| `hikari_loc/loc_state` | `std_msgs/msg/Int32` | 定位状态枚举 |
| `hikari_loc/ndt_status` | `std_msgs/msg/Int32` | NDT 状态 |
| `hikari_loc/loc_status` | `visualization_msgs/msg/Marker` | RViz 文本状态 |
| `/pcdmap` | `sensor_msgs/msg/PointCloud2` | 固定地图可视化，transient local |
| `hikari_loc/status` | `std_msgs/msg/Float32MultiArray` | `[state, ndt_conf, imu_age_s, lidar_age_s, fps, in_map]` |
| `hikari_loc/sc/accum_cloud` | `sensor_msgs/msg/PointCloud2` | SC 累积点云调试 |
| `hikari_loc/sc/candidates` | `visualization_msgs/msg/MarkerArray` | SC 候选调试 |
| `hikari_loc/sc/init_guess` | `geometry_msgs/msg/PoseStamped` | SC 初值调试 |
| `hikari_loc/sc/gravity_check` | `std_msgs/msg/Float32MultiArray` | SC 重力一致性调试 |

服务：

| Service | Type | 说明 |
| --- | --- | --- |
| `hikari_loc/sc_reloc` | `std_srvs/srv/Trigger` | 手动触发 Scan Context 重定位 |

根据配置，节点还会发布 `map -> base_frame_id` 和 `lidar_frame_id -> level_frame_id` TF。

### 初始化和重定位流程

冷启动后节点进入 `WAIT_FOR_INITIALPOSE`，等待 `/initialpose` 或自动 Scan Context 重定位。外部 `/initialpose` 不会被直接当作真值使用，而是作为候选位姿，经最新去畸变点云的 NDT 验证后才调用 `ResetToMapPose()`。

当启用稳定门控时，验证通过后节点先进入 `Initializing`，持续发布 TF/odom，等滑动窗口内位姿抖动满足阈值或 NDT 置信度达到提前放行阈值后再进入 `Good`。

在 `Good` 状态下，Scan Context 会被 disarm，避免持续消耗 CPU；在初始化或 LOST 恢复时才进行有界重定位尝试。

### .deb 打包和部署

默认使用 Docker buildx 打包 ROS 2 Humble / Ubuntu Jammy：

```bash
./build.sh
```

常用覆盖项：

```bash
ROS_DISTRO=humble UBUNTU_CODENAME=jammy TARGETARCH=amd64 BUILD_JOBS=2 ./build.sh
ROS_DISTRO=humble UBUNTU_CODENAME=jammy TARGETARCH=arm64 BUILD_JOBS=2 ./build.sh
OUTPUT_DIR=/tmp/hikari-loclite-debs ./build.sh
```

生成的 `.deb` 默认输出到：

```text
debs/<ros_distro>-<arch>/
```

安装示例：

```bash
sudo dpkg -i ros-<ros_distro>-livox-ros-driver2_*.deb \
  ros-<ros_distro>-hikari-loclite_*.deb
sudo apt -f install
```

安装包包含 systemd unit 模板。默认只安装并 reload systemd，不自动 enable 或 start：

```bash
sudo systemctl enable --now hikari-loclite
```

真机部署通常需要将 unit 中的 `HIKARI_LOCLITE_CONFIG` 指向现场标定后的 YAML，或使用 systemd drop-in override。

### 注意事项

- Release 包只面向在线定位，不携带离线 rosbag 评估节点。
- 固定地图定位过程中不应把当前扫描插入地图。
- `ndt.min_confidence`、`system.stability_gate_conf_upper_thres`、SC 参数和 ZUPT 参数都应按现场地图和传感器重新标定。
- CPU 亲和和实时调度通过 YAML 中 `system.rt_*` 控制；无权限时节点会降级为普通调度并继续运行。

### 许可证

本仓库采用 MIT 许可证，见 [LICENSE](LICENSE)。第三方代码和 vendored 依赖仍遵循其各自许可证文件。

## English

`hikari_loclite` is a lightweight ROS 2 fixed-map LiDAR localization package for embedded deployment. It consumes Livox Mid360 or generic `PointCloud2` point clouds plus IMU data, then publishes odometry, path, TF, and localization status in the `map` frame.

This project is not a full SLAM framework. It focuses on stable, deployable, resource-bounded localization against an existing map. The main runtime path is fixed-map Fast-LIO tracking. NDT is used for `/initialpose` validation, Scan Context candidate validation, and low-rate drift correction. Scan Context is used for cold-start or LOST-state relocalization.

### Features

- Fixed-map Fast-LIO localization without runtime map growth.
- Livox `CustomMsg` and generic `sensor_msgs/msg/PointCloud2` inputs.
- `/initialpose` candidate injection with NDT validation before state reset.
- Automatic and manual Scan Context relocalization.
- Lightweight GOOD / DEGRADED / LOST / WAIT_FOR_INITIALPOSE state machine.
- `/pcdmap` visualization, status marker, and rich status array publishing.
- Release online deployment and non-Release rosbag offline evaluation.
- Docker buildx `.deb` packaging flow and systemd unit template.

### Repository Layout

```text
.
├── config/                  # Default YAML configuration
├── launch/                  # ROS 2 launch files
├── include/hikari_loclite/  # Public types, LIO, NDT, relocalization, system headers
├── src/
│   ├── app/                 # run_loclite_online / run_loclite_offline entrypoints
│   ├── common/              # Shared navigation-state implementation
│   ├── lio/                 # Fast-LIO, ESKF, preprocessing, Scan Context
│   ├── ndt/                 # NDT OMP and correction/validation
│   ├── reloc/               # Scan Context relocalization manager
│   └── system/              # ROS 2 node, state machine, realtime setup
├── scripts/zupt_calib/      # ZUPT calibration helper scripts
├── thirdparty/              # Vendored Sophus, nanoflann, and other local code
├── docker2/                 # .deb packaging Dockerfile, postinst, systemd template
└── build.sh                 # Docker buildx packaging script
```

### Dependencies

Core dependencies:

- ROS 2 `ament_cmake`
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `std_srvs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `visualization_msgs`
- `pcl_conversions`
- `pcl_ros`
- `livox_ros_driver2`
- Eigen3
- PCL
- yaml-cpp
- OpenMP

`rosbag2_cpp` is used only by the non-Release `run_loclite_offline` evaluation binary. Release builds do not compile or install the offline node.

### Open-Source Libraries and References

This project uses or vendors GitHub open-source libraries including [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher), Sophus, and nanoflann. Its implementation also references `lightning-lm` fixed-map localization, NDT, Scan Context, and stability-gate flows. Third-party code remains governed by its own license files.

### Build

Build in a ROS 2 workspace:

```bash
source /opt/ros/<ros_distro>/setup.bash
colcon build --base-paths src/hikari_loclite \
  --packages-select hikari_loclite \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Use a non-Release build type when the offline rosbag evaluator is needed:

```bash
source /opt/ros/<ros_distro>/setup.bash
colcon build --base-paths src/hikari_loclite \
  --packages-select hikari_loclite \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

After building:

```bash
source install/setup.bash
```

### Map and Configuration

Default config:

```text
config/loclite_livox.yaml
```

Map path resolution order:

1. CLI `--map_path`
2. YAML `system.map_path`
3. YAML `fixed_map.global_pcd`

When `--map_path <map_dir>` is provided, the node loads the fixed map and relocalization data from the map directory, for example:

```text
<map_dir>/global.pcd
<map_dir>/sc_database.bin
<map_dir>/poses.txt
```

Different maps, sensor mounting angles, and point-cloud densities affect NDT confidence thresholds, Scan Context parameters, and ZUPT thresholds. Recalibrate the relevant keys in `config/loclite_livox.yaml` before field deployment.

### Online Runtime

Preferred CLI form:

```bash
ros2 run hikari_loclite run_loclite_online \
  --config /path/to/loclite_livox.yaml \
  --map_path /path/to/map_dir
```

Launch-file form:

```bash
ros2 launch hikari_loclite loclite.launch.py \
  config:=/path/to/loclite_livox.yaml
```

`run_loclite_online` accepts:

```bash
--config <yaml>
--config=<yaml>
--map_path <map_dir>
--map_path=<map_dir>
```

### Offline Evaluation

The offline node is available only in non-Release builds:

```bash
ros2 run hikari_loclite run_loclite_offline \
  --config /path/to/loclite_livox.yaml \
  --input_bag /path/to/bag \
  --map_path /path/to/map_dir
```

The offline runner reads IMU, Livox `CustomMsg`, and `PointCloud2` messages in rosbag order and feeds the same `LocLiteNode` processing path used online. This keeps replay evaluation and RViz comparison close to real runtime behavior.

### ROS Interfaces

Default subscriptions:

| Topic | Type | Description |
| --- | --- | --- |
| `/livox/imu` | `sensor_msgs/msg/Imu` | IMU input |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Livox cloud input when `fast_lio.lidar_type: 1` |
| `/cloud` | `sensor_msgs/msg/PointCloud2` | Generic point-cloud input for non-Livox modes |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | External initial-pose candidate |

Default publications:

| Topic | Type | Description |
| --- | --- | --- |
| `hikari_loc/odom` | `nav_msgs/msg/Odometry` | Localization odometry. Pose is `map -> lidar_frame_id`; linear twist is ESKF velocity rotated into `child_frame_id`; angular twist and covariance are currently unavailable |
| `hikari_loc/path` | `nav_msgs/msg/Path` | Pose path |
| `hikari_loc/loc_state` | `std_msgs/msg/Int32` | Localization state enum |
| `hikari_loc/ndt_status` | `std_msgs/msg/Int32` | NDT status |
| `hikari_loc/loc_status` | `visualization_msgs/msg/Marker` | RViz text status |
| `/pcdmap` | `sensor_msgs/msg/PointCloud2` | Fixed-map visualization, transient local |
| `hikari_loc/status` | `std_msgs/msg/Float32MultiArray` | `[state, ndt_conf, imu_age_s, lidar_age_s, fps, in_map]` |
| `hikari_loc/sc/accum_cloud` | `sensor_msgs/msg/PointCloud2` | SC accumulated-cloud debug output |
| `hikari_loc/sc/candidates` | `visualization_msgs/msg/MarkerArray` | SC candidate debug output |
| `hikari_loc/sc/init_guess` | `geometry_msgs/msg/PoseStamped` | SC initial-guess debug output |
| `hikari_loc/sc/gravity_check` | `std_msgs/msg/Float32MultiArray` | SC gravity-consistency debug output |

Service:

| Service | Type | Description |
| --- | --- | --- |
| `hikari_loc/sc_reloc` | `std_srvs/srv/Trigger` | Manually trigger Scan Context relocalization |

Depending on configuration, the node also publishes `map -> base_frame_id` and `lidar_frame_id -> level_frame_id` TF.

### Initialization and Relocalization

On cold start, the node enters `WAIT_FOR_INITIALPOSE` and waits for `/initialpose` or automatic Scan Context relocalization. An external `/initialpose` is treated as a candidate, not as ground truth. It must pass NDT validation against the latest deskewed scan before `ResetToMapPose()` is called.

When the stability gate is enabled, a validated pose first enters `Initializing`. TF and odometry continue to publish. The state switches to `Good` only after the sliding-window pose jitter satisfies the configured thresholds or the NDT confidence reaches the early-release threshold.

In `Good` state, Scan Context is disarmed to avoid continuous CPU usage. It is used only during initialization or LOST recovery.

### .deb Packaging and Deployment

The default Docker buildx packaging target is ROS 2 Humble / Ubuntu Jammy:

```bash
./build.sh
```

Common overrides:

```bash
ROS_DISTRO=humble UBUNTU_CODENAME=jammy TARGETARCH=amd64 BUILD_JOBS=2 ./build.sh
ROS_DISTRO=humble UBUNTU_CODENAME=jammy TARGETARCH=arm64 BUILD_JOBS=2 ./build.sh
OUTPUT_DIR=/tmp/hikari-loclite-debs ./build.sh
```

Generated packages are written to:

```text
debs/<ros_distro>-<arch>/
```

Install example:

```bash
sudo dpkg -i ros-<ros_distro>-livox-ros-driver2_*.deb \
  ros-<ros_distro>-hikari-loclite_*.deb
sudo apt -f install
```

The package includes a systemd unit template. Installation only installs the unit and reloads systemd; it does not enable or start the service automatically:

```bash
sudo systemctl enable --now hikari-loclite
```

For field deployment, point `HIKARI_LOCLITE_CONFIG` in the unit to the calibrated YAML file, or use a systemd drop-in override.

### Notes

- Release packages are for online localization only and do not include the offline rosbag evaluator.
- Fixed-map localization must not insert live scan points into the map.
- `ndt.min_confidence`, `system.stability_gate_conf_upper_thres`, Scan Context parameters, and ZUPT parameters should be recalibrated for each field map and sensor setup.
- CPU affinity and realtime scheduling are configured through YAML `system.rt_*` keys. If permissions are missing, the node falls back to normal scheduling and continues running.

### License

This repository is released under the MIT License. See [LICENSE](LICENSE). Third-party code and vendored dependencies remain under their respective license files.
