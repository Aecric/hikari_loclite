# Runtime And Relocalization

Runtime orchestration belongs in `LocLiteNode` and related system classes. The
node should own ROS2 subscriptions, publishers, TF, config loading, state
machine transitions, and lifecycle-level coordination.

## ROS2 Node Contract

The intended node subscribes to:

- Livox custom cloud topic, default `/livox/lidar`
- Generic point cloud topic, default `/cloud`
- IMU topic, default `/livox/imu`
- `/initialpose`

It publishes:

- odometry
- localization state
- TF when configured
- optional path/debug output only when configured

## Scenario: Optional PCD Map Debug Publisher

### 1. Scope / Trigger

- Trigger: runtime needs a fixed-map visualization topic so online and offline
  localization can be compared against the loaded global PCD in RViz.
- Owner: `LocLiteNode`, because online and offline runners both call
  `LocLiteNode::Init()`.

### 2. Signatures

- YAML key: `runtime.publish_pcdmap: bool`
- ROS topic: `/pcdmap`
- Message type: `sensor_msgs::msg::PointCloud2`

### 3. Contracts

- When `runtime.publish_pcdmap` is `true`, publish the loaded fixed-map cloud
  once after map loading and publisher creation.
- The published cloud must use `header.frame_id = common.map_frame_id`
  (default `map`).
- The cloud must be the global PCD after `fixed_map.voxel_leaf` downsampling,
  not a local cropped iVox view and not live scan accumulation.
- The publisher QoS must be `reliable + transient_local` with depth 1 so RViz
  can subscribe after startup and still receive the map.
- The topic name is fixed as `/pcdmap`; do not remap it through another config
  key unless the external validation contract changes.

### 4. Validation & Error Matrix

- Config key missing -> default disabled, no warning.
- Config key false -> do not publish `/pcdmap`.
- Fixed map cloud missing or empty while enabled -> warn once and continue
  localization startup.
- PCD load failure -> existing fixed-map load path returns `false`; node init
  fails before `/pcdmap` publish.

### 5. Good/Base/Bad Cases

- Good: `publish_pcdmap: true`, valid `global.pcd` -> `/pcdmap` latched in
  `map` frame with downsampled points.
- Base: `publish_pcdmap: false` -> no extra map topic, normal localization
  behavior unchanged.
- Bad: publishing a cropped local map or current scan on `/pcdmap` -> RViz
  comparison becomes misleading and violates the fixed-map invariant.

### 6. Tests Required

- Build: run the containerized Release build from the Build And Dependencies
  spec.
- Static check: `git diff --check`.
- Runtime smoke when a map is available: enable `runtime.publish_pcdmap`,
  start online or offline node, and assert `ros2 topic echo --once /pcdmap`
  reports `header.frame_id` equal to `common.map_frame_id`.

### 7. Wrong vs Correct

#### Wrong

Publishing `/pcdmap` from a live scan, an accumulated scan context cloud, or a
locally cropped map.

#### Correct

Publishing `FastLioFixedMap::FixedMapCloud()` after the fixed map has been
loaded and downsampled by the configured voxel leaf.

Config should be provided via a `config_path` ROS parameter and parsed from a
YAML file like `config/loclite_livox.yaml`.

## State Machine

Use the lightweight states from the build document:

- `Uninitialized`
- `Initializing`
- `Good`
- `Degraded`
- `Lost`
- `WaitForInitialPose`

Tracking quality should use frame counters, not single-frame panic switches.
The documented defaults are 3 bad frames for Degraded, 10 bad frames for Lost,
and 5 good frames for recovery to Good.

### Initializing -> Good stability gate

A validated `/initialpose` (or SC candidate) enters `Initializing`, not `Good`
directly. The gate (`system/stability_gate.hpp`) releases to `Good` only when
the smoothed output pose stays within `system.stability_gate_trans_thres` /
`stability_gate_rot_thres_deg` jitter across a `stability_gate_window_sec`
sliding window, or earlier when NDT TP confidence exceeds
`system.stability_gate_conf_upper_thres` (TP scale, see NDT Role). During the
gate window TF/odom keep publishing but `loc_state` stays `Initializing`. Set
`system.stability_gate_enabled: false` to restore the old "validated ->
immediately Good" behavior. Reset the gate on every `ResetToMapPose()` path
(`/initialpose`, manual SC) so a stale window cannot release a fresh pose.

## Main Processing Rules

- IMU callbacks should enqueue IMU data under mutex.
- LiDAR callbacks should enqueue cloud data, then trigger bounded frame
  processing.
- `ProcessFrame()` should run Fast-LIO once, evaluate tracking quality, handle
  NDT correction or relocalization based on state, then publish pose/state.
- Do not hold a mutex while running expensive relocalization work unless the
  shared data actually requires it.

## `/initialpose`

- Treat `/initialpose` as an external candidate, not truth.
- Convert it to map-frame pose explicitly.
- Validate it with NDT against the latest deskewed scan before calling
  `ResetToMapPose()`.
- Rejected initial poses must leave Fast-LIO state unchanged.

## LOST Recovery

- LOST should arm `RelocManager`.
- `RelocManager::TryRelocalize()` should return immediately if disarmed or if
  the scan is empty.
- A successful SC candidate must pass NDT validation before state reset.
- After successful relocalization, set Good and disarm relocalization.
- In Good state, disarm relocalization so SC threads or workers do not consume
  CPU.

## Pose Output Gating

Use a smoother/gate to prevent unrealistic published pose jumps:

- Reject output jumps beyond configured translation/rotation limits.
- Apply NDT correction with a configurable gain.
- Use stricter correction limits than LOST recovery validation.

Defaults from the build document include `max_output_jump_trans_m: 0.5`,
`max_output_jump_rot_deg: 15.0`, `max_correction_trans_m: 0.3`, and
`max_correction_rot_deg: 5.0`.
