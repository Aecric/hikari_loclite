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
- `ResetToMapPose()` must not reset the IMU processor or force the next frame
  through the cold-start first-scan branch. External poses and NDT corrections
  are applied after a deskewed scan has already proven the candidate pose; the
  next frame must continue IMU propagation and fixed-map matching from the new
  ESKF state. Resetting IMU initialization here freezes pose output for the init
  window and can let stability gates release a static pose.

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
- Reject corrections beyond configured translation/rotation bounds via the
  smoother gate (see runtime-and-relocalization "Pose Output Gating").
- NDT result structs carry `valid`, confidence (TP), `inlier_ratio`, translation
  delta, and rotation delta.
- Confidence is pclomp `getTransformationProbability()` (TP): the mean Gaussian
  density of matched points, not a 0..1 ratio. **The TP scale is resolution- and
  map-density-dependent — measure it per map, do not copy a threshold from
  lightning.** lightning's "real matches fall in [1.5, 5]" describes its
  *cascaded* multi-scale NDT; a single-stage NDT on a voxel-downsampled fixed map
  sits lower. Measured on zt_5201_map (`fixed_map.voxel_leaf=0.2`) at
  `ndt.resolution=1.0`, true-pose TP ≈ 1.3–1.4, hence `ndt.min_confidence=1.0`
  and `system.stability_gate_conf_upper_thres=1.3`. Re-measure and re-tune for a
  new map/sensor; a too-high threshold silently breaks init validation, GOOD-state
  correction, and SC validation all at once.
- The validation gate is TP + delta, plus an optional orthogonal `inlier_ratio`
  coverage gate: `valid = converged && TP >= ndt.min_confidence && delta_trans <=
  ndt.max_delta_trans_m && delta_rot <= ndt.max_delta_rot_deg && (ndt.min_inlier_ratio
  <= 0 || inlier_ratio >= ndt.min_inlier_ratio)`. `inlier_ratio` is the fraction of
  downsampled source points that, transformed by the **converged** pose (not the
  guess), find a target neighbor within `ndt.inlier_dist_m` (target kd-tree built
  once in `SetMap`). It complements TP (TP = Gaussian density of matched points;
  inlier = geometric coverage). Default `ndt.min_inlier_ratio=0.0` keeps the gate
  off (record-only) for field calibration; raise it once a map's true/false inlier
  separation is known.

### Design Decision: single-stage NDT at resolution 1.0 (not 0.5, not cascaded)

**Context**: With `ndt.resolution=0.5` on a `voxel_leaf=0.2` fixed map, measured
TP collapsed to 0.15–1.2 even at the true pose — below the (mis-copied)
`min_confidence=1.5` — so `/initialpose` validation, GOOD-state correction, and
SC validation all silently failed. lightning warns (lidar_loc.cc:501) that fine
NDT confidence collapses on sparse maps.

**Decision (user 2026-06-11)**: single-stage `ndt.resolution=1.0` (raises and
stabilizes TP into a usable ~1.3–1.4 band), recalibrate thresholds to the
measured scale, and add `inlier_ratio` as a cheap orthogonal gate. Do NOT adopt
lightning's cascaded 5→2→0.5 multi-scale NDT — embedded budget is `threads=1`
and cascade triples NDT cost. Escalate to cascade only if a future map proves
single-stage TP cannot separate true from false matches.

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
