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
