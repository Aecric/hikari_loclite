# IMU-only localization motion consistency LOST gate

## Goal

Add an IMU-only consistency gate that can mark localization as LOST when short-window IMU motion evidence conflicts with the current localization output. This is a test-oriented first version for the field issue where existing ESKF speed/acceleration sanity gates miss two failure modes: localization drifts at a physically smooth speed, or localization reports almost no motion while the robot is actually moving.

## What I already know

* Existing `sanity_gate` in `src/system/loclite_node.cpp` checks ESKF `state.vel_.norm()` and `|v_k - v_{k-1}| / dt` only in `Good` / `Degraded`, then immediately calls `SetLost("sanity_speed"/"sanity_accel")`.
* Existing LOST handling already freezes TF/odom at the last trusted pose and can use NDT local recovery / reloc recovery. The new gate should reuse this path instead of adding another LOST mechanism.
* `OnImu()` already receives every IMU sample, updates `last_imu_wall_ts_`, feeds Fast-LIO, publishes optional IMU-extrapolated TF, and updates gravity-alignment TF.
* `FastLioFixedMap` already has a `StaticDetector` used for ZUPT. It computes gyro-norm and accel-norm window standard deviation with hysteresis, but it lives inside the LIO module and is coupled to ZUPT calibration semantics.
* `runtime.imu_extrapolate_tf` deliberately uses gyro for attitude and constant-velocity translation; comments state raw acceleration is unsafe for publication-only position extrapolation on humanoid vibration. The LOST gate must therefore treat raw acceleration conservatively.
* There is no odom source available from the base. The only external intent signal discussed is `cmd_vel`, but this task intentionally excludes it per user request.
* Pure IMU cannot distinguish "truly static" from "constant-velocity straight motion" once acceleration and angular velocity settle near zero. This task can catch acceleration/rotation evidence and static-vs-localization contradictions, but not all constant-velocity cases.

## Assumptions

* This first version is a test feature and defaults to disabled; the user will enable it after deployment to compare behavior.
* "Only based on IMU" means no wheel odom and no `cmd_vel`, but comparison against localization output is allowed because LOST is defined as IMU evidence contradicting localization state.
* The gate should be conservative: require a sliding window and consecutive abnormal windows before triggering LOST, avoiding one-sample vibration or timestamp glitches.
* It should run only when localization is already in `Good` or `Degraded`; `Lost`, `WaitForInitialPose`, `Initializing`, and cold-start periods should not be judged by this gate.

## Requirements

* Add a configurable IMU consistency gate, for example `sanity_gate.imu_consistency_*` or a nested `imu_consistency_gate` section, with at least:
  * `enabled`, default `false`
  * `window_sec`
  * `min_check_interval_sec` or equivalent per-lidar-frame evaluation cadence
  * `loc_static_speed_mps`
  * `loc_static_dist_m`
  * `loc_moving_dist_m`
  * `gyro_dynamic_radps`
  * `gyro_yaw_delta_rad`
  * `acc_dynamic_mps2`
  * `acc_window_energy_mps2`
  * `fail_windows`
  * `log_rate_hz`
* Maintain an IMU sliding window from `OnImu()` using sensor timestamps:
  * gyro norm / yaw-rate evidence
  * accel norm variation or high-pass / gravity-removed acceleration energy if a reliable orientation and gravity estimate is available
  * timestamp rollback / large gap should reset the window rather than trigger LOST
* Maintain a localization motion window using accepted lidar-rate states:
  * start/end timestamp
  * position delta
  * yaw delta
  * ESKF velocity norm
* Detect "IMU static, localization moving":
  * IMU window appears static or low dynamic
  * localization position delta or velocity exceeds configured threshold
  * after `fail_windows` consecutive failures, call existing LOST path with reason such as `imu_consistency_static_loc_moving`
* Detect "IMU dynamic/rotating, localization stationary":
  * gyro yaw integration or acceleration energy exceeds configured threshold
  * localization position/yaw delta and velocity remain below configured threshold
  * after `fail_windows` consecutive failures, call existing LOST path with reason such as `imu_consistency_imu_moving_loc_static`
* Rotation should be a first-class check:
  * if IMU gyro integrates a meaningful yaw change but localization yaw barely changes, this is stronger evidence than linear acceleration alone.
* Trigger behavior must mirror existing sanity gate behavior:
  * save/freeze last good pose using the same already-implemented freeze path
  * publish frozen pose immediately on the trigger frame
  * arm lost recovery when configured by existing reloc settings
  * publish status topics after the transition
* Add throttled diagnostic logs:
  * current IMU static/dynamic metrics
  * localization delta metrics
  * fail counter and trigger reason
  * logs must be rate-limited to avoid IMU-rate spam
* YAML comments must clearly state the limitation: IMU-only cannot detect perfectly constant straight-line motion without acceleration/rotation.

## Acceptance Criteria

* [ ] With the gate disabled, behavior and logs remain equivalent to the current system.
* [ ] With the gate enabled, a static IMU window plus nonzero localization motion for consecutive windows triggers LOST with an IMU consistency reason.
* [ ] With the gate enabled, meaningful IMU gyro yaw integration but near-zero localization yaw/motion for consecutive windows triggers LOST with an IMU consistency reason.
* [ ] Timestamp rollback, IMU gaps, or insufficient window duration reset/defer the gate rather than causing LOST.
* [ ] The trigger reuses existing frozen-pose publication and recovery flow; no divergent LIO pose is published after the gate trips.
* [ ] Configuration is present in `config/loclite_livox.yaml` with conservative defaults and clear comments.
* [ ] Build passes.

## Definition of Done

* Code changes are localized to `LocLiteNode` / a small helper under `include/hikari_loclite/system/` if that keeps the node readable.
* No new external dependency is introduced.
* Logging follows existing throttling style.
* Existing sanity gate behavior remains intact.
* Relevant Trellis specs are referenced in `implement.jsonl` / `check.jsonl` before implementation starts.

## Out of Scope

* No wheel odom.
* No `cmd_vel` subscription in this task.
* No full IMU preintegration factor or new ESKF formulation.
* No automatic threshold calibration tool in this task.
* No attempt to solve pure constant-velocity straight-line movement when IMU acceleration and gyro are both indistinguishable from static.

## Technical Notes

* Likely files:
  * `include/hikari_loclite/system/loclite_node.hpp`
  * `src/system/loclite_node.cpp`
  * `config/loclite_livox.yaml`
  * possibly a helper header such as `include/hikari_loclite/system/imu_consistency_gate.hpp`
* Relevant existing code:
  * Existing physical sanity gate: `src/system/loclite_node.cpp` Good/Degraded branch before smoother.
  * IMU callback: `LocLiteNode::OnImu`.
  * IMU-TF extrapolation: `LocLiteNode::MaybePublishImuExtrapolatedTF`.
  * Static detection reference: `include/hikari_loclite/lio/static_detector.h`.
  * Existing PRD: `.trellis/tasks/06-17-physical-velocity-accel-lost-gate-tf-freeze-reloc-recovery/prd.md`.
* Relevant specs:
  * `.trellis/spec/backend/index.md`
  * `.trellis/spec/backend/localization-architecture.md`
  * `.trellis/spec/backend/runtime-and-relocalization.md`
  * `.trellis/spec/backend/logging-guidelines.md`
  * `.trellis/spec/backend/quality-guidelines.md`

## Open Question

* None.

## Decisions

* 2026-06-25: Default `enabled: false` for safe rollout. The user will enable the gate after deployment to compare behavior against the current baseline.
