# Localization Architecture

The product is lightweight fixed-map LiDAR localization for embedded devices.
It is not a full SLAM framework. The normal runtime chain is:

```text
Livox/PointCloud2 + IMU
  -> Fast-LIO fixed-map tracking
  -> Lite pose gate / smoother
  -> TF / odom / loc_state
```

Initialization or LOST recovery uses:

```text
/initialpose or Scan Context/NDT candidate
  -> NDT validation
  -> Fast-LIO ResetToMapPose()
  -> fixed-map tracking
```

## Fixed-Map Fast-LIO Rules

- Fixed-map mode must not call `MapIncremental()`.
- Current scan points must not be inserted into the fixed map.
- `RunOnce()` should sync measurements, deskew the scan, match against the
  fixed map, and return a state. It should leave map mutation to explicit map
  load/rebuild paths only.
- `LoadFixedMap()` should load the global PCD, voxel-filter it, and build the
  fixed iVox or KD-tree structure.
- `RebuildLocalMapAround()` may crop around the current map-frame pose and
  rebuild a local fixed structure. It must derive from the loaded fixed map, not
  from live scan accumulation.

Current references:

- `include/hikari_loclite/ivox3d/ivox3d.h`
- `include/hikari_loclite/lio/imu_processing.hpp`
- `include/hikari_loclite/lio/eskf.hpp`
- `src/lio/pointcloud_preprocess.cc`
- `hikari_loclite_build_2026-06-10.md`

## Point Cloud And Sensor Types

- Keep PCL as the runtime point-cloud representation unless a profiled local
  change proves otherwise.
- Preserve custom point fields for intensity and time. `PointXYZIT`,
  `PointRobotSense`, Velodyne, and Ouster point structs are registered in
  `include/hikari_loclite/common/point_def.h`.
- Do not discard timestamp fields during conversions; deskewing depends on
  point timing.
- Livox custom messages and generic `sensor_msgs::msg::PointCloud2` are both
  expected inputs.

## NDT Role

NDT is a validator and low-frequency corrector, not the primary tracking loop.

- Use NDT to validate `/initialpose` and relocalization candidates.
- In Good state, NDT correction should be low-rate and gain-limited.
- Reject corrections beyond configured translation and rotation bounds.
- NDT result structs should carry `valid`, confidence, inlier ratio, translation
  delta, and rotation delta.

## Scan Context Role

Scan Context is for initialization and LOST recovery. It must not become a
permanent background CPU consumer.

- Load SC database offline.
- During init/LOST, run one bounded query with small Top-K defaults.
- Good state must disarm relocalization.
- Candidate poses must pass NDT validation before Fast-LIO reset.

Current SC implementation references:

- `include/hikari_loclite/lio/scan_context.h`
- `src/lio/scan_context.cc`

## Explicit Non-Goals

Do not add these to normal runtime:

- Pose graph optimization
- Dynamic maps
- Pangolin UI
- KISS as a default capability
- Full LidarLoc state machine
- Always-on SC/KISS background threads
