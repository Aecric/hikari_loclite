# IMU Extrapolated map to lidar TF Publishing

## Goal

Reduce the effective latency of localization pose publication by publishing an IMU-extrapolated `map -> lidar/base` pose between lidar frames. The current output is tied to `FastLioFixedMap::RunOnce()` and therefore updates only after a lidar frame is synchronized, deskewed, matched, and published, which can leave downstream TF users seeing roughly one lidar period of delay.

## What I Already Know

* User wants an IMU extrapolated localization output similar in spirit to lightning's fast pose output, not a full PGO/backend copy.
* Current `LocLiteNode::PublishPose(const NavState&)` publishes odom and TF only from lidar-frame LIO output.
* `LocLiteNode::OnImu()` currently only records watchdog time, enqueues IMU into `FastLioFixedMap`, and publishes `lidar_frame -> level_frame` when the LIO state timestamp advances.
* `FastLioFixedMap` does not expose a per-IMU propagated state; its ESKF is propagated inside `ImuProcess::Process()` during lidar processing.
* Existing frame contract: odom pose is `map -> lidar_frame`; TF currently publishes `map -> base_frame_id`, and the default config sets `base_frame_id` to `livox_frame`, matching the lidar frame on this rig.
* Existing LOST/freeze behavior deliberately republishes the last trusted pose and must not be bypassed by extrapolated IMU output.

## Requirements

* Add an optional IMU extrapolated pose publisher for runtime pose output.
* Seed the extrapolator from accepted lidar-frame LIO output after the normal smoother/gates/state machine path.
* On each IMU callback, extrapolate from the last published/extrapolated state to the IMU timestamp using bias-corrected gyro for attitude and last trusted LIO velocity for constant-velocity translation.
* Do not integrate raw accelerometer into published position in the MVP; humanoid footstep vibration can inject visible TF jitter if acceleration is double-integrated without Fast-LIO's full filtering and lidar-update context.
* Publish only when localization has a trusted output (`Initializing`, `Good`, or `Degraded` with existing policy), and suppress extrapolated output in `Lost`, `WaitForInitialPose`, and frozen-pose periods.
* Bound extrapolation with config limits so stale IMU, timestamp rollback, large dt, or excessive time since last lidar correction cannot publish runaway poses.
* Keep the extrapolated state publication-only; do not mutate the Fast-LIO ESKF or fixed-map tracking state from the IMU callback.
* Preserve existing lidar-frame path publication behavior; path should remain lidar-rate unless explicitly changed later.
* Keep the feature disabled or safely configurable if field testing shows a rig-specific issue.

## Acceptance Criteria

* [ ] With `runtime.imu_extrapolate_tf: true`, `map -> base_frame_id` TF updates at IMU cadence between lidar frames after the first trusted LIO output.
* [ ] Published extrapolated timestamps use the IMU message timestamp and never repeat or go backward.
* [ ] The node does not publish extrapolated poses while LOST/frozen or before localization has a trusted pose.
* [ ] Extrapolation stops when `imu_ts - last_lidar_state_ts` exceeds a configured maximum, then resumes on the next trusted lidar output.
* [ ] Existing lidar-frame publishing still runs and periodically re-seeds the extrapolator with corrected LIO/NDT pose.
* [ ] `colcon build --packages-select hikari_loclite --cmake-args -DCMAKE_BUILD_TYPE=Release` passes, or any inability to run it is documented.

## Technical Approach

Recommended implementation is a lightweight publication-side extrapolator in `LocLiteNode`:

* Store a cached extrapolation state seeded by `PublishPose(const NavState&)`: timestamp, `T_map_imu`, `T_map_lidar`, velocity in map frame, gyro bias, gravity, and last lidar correction timestamp.
* In `OnImu()`, after `lio_->AddImu(msg)`, integrate:
  * `gyro_avg = 0.5 * (last_gyro + current_gyro)` when a previous IMU sample is available, matching Fast-LIO's adjacent-sample averaging style.
  * `omega = gyro_avg - bg`
  * `pos += vel * dt`
  * `rot = rot * Exp(omega * dt)`
* Convert extrapolated IMU pose to lidar pose via `FastLioFixedMap::ImuPoseToLidarPose()` and publish through a shared pose-output helper.
* Add runtime config keys:
  * `runtime.imu_extrapolate_tf`
  * `runtime.imu_extrapolate_max_dt_sec`
  * `runtime.imu_extrapolate_max_ahead_sec`
* Keep path publication on lidar outputs only.
* Keep `hikari_loc/odom` on lidar outputs only; the MVP accelerates TF only.

## Alternatives Considered

**A. Publication-side gyro/constant-velocity extrapolator in LocLiteNode (recommended)**

* Pros: low risk, avoids double-integrating humanoid acceleration vibration, no mutation of ESKF internals from IMU callback, bounded CPU cost, easy to disable, respects existing lidar correction path.
* Cons: translation is only constant-velocity between lidar corrections, so aggressive acceleration is not fully represented during the short extrapolation window.

**B. Expose Fast-LIO per-IMU propagated state**

* Pros: closer to lightning-style `GetIMUState()` semantics.
* Cons: touches `ImuProcess`/ESKF ownership and buffering, risks double-consuming IMU samples, and increases coupling in the hottest tracking path.

**C. Timer-based constant-velocity extrapolation**

* Pros: simpler and independent of raw IMU quality.
* Cons: less accurate during turns/acceleration and does not address the user's request for IMU-based pose extrapolation.

## Decision (ADR-lite)

**Context**: The current package keeps Fast-LIO fixed-map tracking and node publishing separated. IMU data is already buffered for the lidar update; changing that ownership risks corrupting synchronization.

**Decision**: Use a publication-side IMU extrapolator in `LocLiteNode`, seeded by trusted lidar output and bounded by short time windows.

**Consequences**: Downstream TF latency drops toward IMU callback latency, while lidar-frame matching remains the authoritative correction source. Extrapolated output is intentionally short-horizon, does not double-integrate raw acceleration, and is disabled in unsafe states.

## Out of Scope

* Copying lightning PGO or adding a full backend optimizer.
* Mutating the Fast-LIO ESKF state on each IMU callback.
* Publishing extrapolated path points at IMU rate.
* Changing the fixed-map matching, NDT validation, KISS/SC relocalization, or LOST recovery logic beyond pose output gating.

## Technical Notes

* Relevant files inspected:
  * `src/system/loclite_node.cpp`
  * `include/hikari_loclite/system/loclite_node.hpp`
  * `src/lio/fast_lio_fixed_map.cpp`
  * `include/hikari_loclite/lio/fast_lio_fixed_map.hpp`
  * `include/hikari_loclite/lio/imu_processing.hpp`
  * `include/hikari_loclite/common/nav_state.h`
  * `config/loclite_livox.yaml`
* Relevant specs:
  * `.trellis/spec/backend/index.md`
  * `.trellis/spec/backend/localization-architecture.md`
  * `.trellis/spec/backend/runtime-and-relocalization.md`
  * `.trellis/spec/backend/quality-guidelines.md`
  * `.trellis/spec/guides/cross-layer-thinking-guide.md`

## Open Questions

* Answered 2026-06-17: publish only extrapolated TF. Keep `hikari_loc/odom` lidar-rate for now.
