# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What is hikari_loclite

Lightweight fixed-map LiDAR localization for embedded deployment. Independent ROS2 package — NOT a slimmed-down fork of `lightning`. Reuses engineering experience and minimal algorithm code from `lightning-lm` but has its own build, deploy, and runtime boundary.

Core localization chain:

```text
GOOD state:
  Livox/PointCloud2 + IMU → Fast-LIO fixed-map tracking → Lite pose gate/smoother → TF/odom/loc_state

INIT/LOST state:
  /initialpose or SC/NDT candidate → NDT validation → Fast-LIO ResetToMapPose() → fixed-map tracking
```

Explicitly does NOT include: PGO, dynamic maps, Pangolin UI, KISS as default capability, full LidarLoc state machine, persistent SC/KISS background threads.

## Build / run

```bash
# from /home/aecriclin/3d_slam_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hikari_loclite --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# run
ros2 run hikari_loclite run_loclite_online --ros-args -p config_path:=/path/to/loclite_livox.yaml
# or
ros2 launch hikari_loclite loclite.launch.py config:=/path/to/loclite_livox.yaml
```

For real-time scheduling on embedded:
```bash
sudo setcap cap_sys_nice+ep install/hikari_loclite/lib/hikari_loclite/run_loclite_online
```

## Dependencies

First version only: `rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `pcl_conversions`, `pcl_ros`, `livox_ros_driver2`, `yaml-cpp`, PCL, Eigen3, OpenMP.

Do NOT add: Pangolin, OpenCV highgui, KISS-Matcher, miao optimizer, g2o, rosbag2_cpp, visualization_msgs.

## Code reuse from lightning-lm

Extract minimal code, do NOT link `lightning.libs`. Acceptable to extract:
- `common/eigen_types.h`, `common/nav_state.h/.cc`, `common/point_def.h`
- `core/lio/aa_faster_lio/eskf.*`, `core/lio/aa_faster_lio/imu_processing.hpp`, `core/lio/pointcloud_preprocess.*`
- `core/lio/scan_context.*` (only if SC is needed)
- `core/localization/lidar_loc/pclomp/*` (NDT OMP only, not full LidarLoc)
- `thirdparty/nanoflann`, `core/ivox3d`

Do NOT include: `core/system/loc_system.*`, `core/localization/localization.*`, `core/localization/pose_graph/*`, `core/maps/tiled_map.*`, `ui/*`, `kiss_matcher_wrapper.*`

## Architecture

Namespace: `hikari::loclite`

Single executable: `run_loclite_online` → `LocLiteNode` (rclcpp::Node)

Components:
- **FastLioFixedMap** — LIO front-end with fixed map. Loads `global.pcd`, builds local iVox/kdtree via crop radius. Never inserts current scan into map (`MapIncremental()` disabled). Provides `ResetToMapPose()` for relocalization.
- **NdtCorrector** — Low-frequency NDT alignment for drift correction (GOOD state) and candidate validation (LOST state). Runs at configurable rate (1 Hz GOOD, 3 Hz LOST). Has confidence/delta gates to reject bad corrections.
- **RelocManager** — Arms on init/LOST, disarms on GOOD. Runs bounded SC query (top-k=1), not a persistent worker. Candidate must pass NDT validation before `ResetToMapPose()`.
- **LitePoseSmoother** — Jump filter on output pose. Rejects large jumps in both Fast-LIO output and NDT corrections. Applies gain-blended NDT corrections.
- **LocLiteStateMachine** — States: Uninitialized → Initializing → Good ↔ Degraded → Lost. Transitions driven by tracking quality counters.

## State machine

```text
LiteLocState: Uninitialized(0) | Initializing(1) | Good(2) | Degraded(3) | Lost(4) | WaitForInitialPose(5)
```

- `ObserveTrackingQuality(good)` increments bad/good counters
- `degraded_bad_frames=3`, `lost_bad_frames=10`, `recover_good_frames=5`

## Configuration

Single yaml file (`config/loclite_livox.yaml`). Key sections:
- `common.*` — topic names, frame IDs
- `runtime.*` — TF/odom/path publishing toggles
- `fixed_map.*` — global PCD path, voxel leaf, crop radius, max points
- `fast_lio.*` — scan filter, iVox grid, iterations, lidar type, extrinsics
- `ndt.*` — threads, resolution, correction/lost rates, confidence/delta gates, gain per state
- `reloc.*` — auto arm/disarm, SC database path, cooldown, KISS/FP toggles
- `smoother.*` — jump thresholds for output and correction

## Design document

The full design with code snippets and phased implementation plan is in `hikari_loclite_build_2026-06-10.md`. Read it before implementing any component.

## Conventions

- Comments and design docs are in Chinese; identifiers and log strings are in English. Match the surrounding style.
- Use `RCLCPP_INFO/WARN/ERROR` for logging (not glog — this package doesn't depend on glog/gflags).
- `livox_ros_driver2/msg/CustomMsg` and `sensor_msgs/PointCloud2` both supported as input.
- Config paths should be absolute. Update launch-file defaults when changing them.
- Build as Release by default. The target is embedded ARM64 — keep CPU budget tight.
