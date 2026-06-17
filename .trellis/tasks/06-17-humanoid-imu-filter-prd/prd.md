# brainstorm: improve humanoid IMU filter

## Goal

Improve the existing `fast_lio.imu_filter` path so it can be safely used on a humanoid robot where walking introduces large step-frequency IMU spikes. The replacement should reduce harmful acceleration shocks without adding gyro phase lag that breaks Fast-LIO propagation and point cloud deskew. It must also coexist with ZUPT, because disabling ZUPT causes too much static drift, while the current static-to-moving transition can produce fly-away behavior.

## What I Already Know

* Target platform is a humanoid robot.
* During walking, IMU signals have large periodic spikes tied to step cadence.
* The observed failure is fly-away during motion, especially after static -> moving transitions.
* ZUPT is still needed because disabling it allows excessive static drift.
* The current `imu_filter: true` was tried before and caused fly-away plus lag.
* Current config has `fast_lio.imu_filter: false`, `gyr_cov: 0.01`, `acc_cov: 0.01`, and ZUPT enabled.
* Current `IMUFilter` only modifies `angular_velocity`; it does not filter or robustly downweight `linear_acceleration`.
* Current `IMUFilter` applies median, moving average, and rate limit to gyro, which can delay angular velocity and damage deskew.
* Current default gyro rate limit is `3.0 rad/s^2`; at 200 Hz this permits only about `0.015 rad/s` change per sample, likely too restrictive for humanoid motion.
* `movingAverage(value, history)` ignores the passed filtered value and averages raw history, so spike/median output is effectively overwritten.
* Filtering is performed inside `ImuProcess::UndistortPcl()` over each LiDAR frame's `v_imu`, not at IMU ingestion time.
* User wants a separate Python ROS2 calibration node, not participating in C++ build/compile, to estimate humanoid IMU spike thresholds from walking data.
* User explicitly wants the old gyro-smoothing filter removed, not preserved behind a compatibility mode.

## Assumptions

* LiDAR and IMU are mounted rigidly enough that a single fixed extrinsic remains valid during walking.
* The main harmful step artifact is linear acceleration impulse rather than true angular velocity.
* Gyro should either pass through unchanged or be subject only to extremely conservative outlier handling.
* A robust acceleration strategy should prefer covariance inflation/downweighting or bounded residual handling over delayed low-pass smoothing.
* ZUPT should remain available, but its moving-state exit behavior must be compatible with humanoid stepping.
* Humanoid IMU filtering and ZUPT static detection should remain independent in MVP.

## Requirements

* Replace or redesign `IMUFilter` so it is safe for Fast-LIO deskew and propagation.
* Avoid median/moving-average/rate-limit smoothing on gyro in the main propagation path by default.
* Add robust handling for acceleration spikes from humanoid walking.
* Preserve IMU timestamps exactly; filtering must not shift timing.
* Preserve raw IMU availability for diagnostics or static detection if needed.
* Make behavior configurable from YAML under `fast_lio`.
* Keep config simple: `imu_filter` is the only user-facing switch for this feature.
* Remove the legacy gyro smoother semantics. After this task, `imu_filter: true` means the new humanoid acceleration-robust filter; `imu_filter: false` means no IMU filter.
* Delete the old median/moving-average/rate-limit gyro smoothing implementation; do not keep a `legacy` mode or hidden compatibility branch.
* Prevent ZUPT from remaining active into motion after static -> moving transition.
* Keep humanoid spike filtering independent from ZUPT state. The filter must not directly force ZUPT enter or exit in MVP.
* Add diagnostics that make it visible when IMU samples are clamped, downweighted, or classified as spike/moving/static.
* Runtime diagnostics should be aggregate and rate-limited only; do not publish per-IMU-sample debug topics in the MVP.
* Keep implementation lightweight and deterministic for embedded runtime.
* Add a standalone Python ROS2 calibration node under `scripts/` that subscribes to IMU data and computes recommended humanoid filter thresholds from walking recordings.
* Calibration node must not be added to CMake/package install targets unless explicitly requested later.

## Acceptance Criteria

* [ ] With filter disabled, behavior remains unchanged.
* [ ] With new humanoid filter enabled, gyro is not delayed by moving-average/rate-limit smoothing.
* [ ] The old gyro median/moving-average/rate-limit path is removed from `IMUFilter`.
* [ ] Acceleration spikes are either bounded or assigned larger process noise without corrupting nominal samples.
* [ ] Static ZUPT still suppresses drift during real static periods.
* [ ] Static -> moving transition exits ZUPT quickly enough that no `ZUPT applied` persists into walking startup.
* [ ] Walking logs no longer show immediate fly-away after static -> moving transition in the target bag/test.
* [ ] Diagnostics expose at least spike count/rate, clamp count/rate, covariance-inflation count/rate, and current filter mode via rate-limited aggregate logs.
* [ ] MVP does not publish per-sample IMU spike classification topics.
* [ ] Python calibration node can run independently with `ros2 run` equivalent via direct `python3 scripts/...py` invocation while a bag is replayed.
* [ ] Calibration output includes recommended YAML values for acceleration norm bounds, acceleration delta threshold, clamp max, and covariance scale.
* [ ] A representative bag/log replay covering static -> startup -> walking -> stopping is used for validation.
* [ ] Replay validation records filter aggregate logs, ZUPT application timing, Fast-LIO residual/quality diagnostics, and final localization state.
* [ ] Existing build and tests pass.

## Recommended MVP Direction

Implement a new acceleration-focused robust IMU mode instead of repairing the existing gyro smoothing behavior. MVP chooses the hybrid path: gyro passthrough, acceleration spike detection, mild acceleration clamp for impossible impulses, and per-sample acceleration process-noise inflation for suspicious intervals.

Suggested behavior:

* `gyro`: pass through unchanged by default.
* `acc`: detect outliers using deviation from recent gravity/magnitude baseline or expected acceleration magnitude.
* For detected acc spikes:
  * soft spike: keep the raw acceleration unchanged and inflate acc process noise for that integration interval so the ESKF trusts the impulse less.
  * hard spike: only when acceleration exceeds the configured hard clamp upper bound, clamp acceleration and also inflate acc process noise.
* Add optional very small EMA only for estimating baseline/statistics, not for replacing propagation gyro.
* Keep ZUPT as a separate state machine, but tune it for slow enter / fast exit and optionally inhibit ZUPT for a short grace window after moving evidence.
* Do not couple filter spike classification into `StaticDetector` in MVP. Shared aggregate logging is allowed, but state transitions remain separate.

## Options To Decide

### Option A: Acceleration Clamp Filter

Clamp only acceleration impulses before propagation. Gyro passes through.

Pros: simple, deterministic, minimal ESKF changes.

Cons: clamp threshold must be tuned; can hide real high acceleration.

### Option B: Dynamic Acc Covariance Inflation

Do not alter the IMU measurement, but increase `acc_cov` for spike intervals.

Pros: cleaner probabilistic model; avoids inventing fake acceleration.

Cons: requires changing `ImuProcess::UndistortPcl()` to support per-sample process noise.

### Option C: Hybrid Clamp + Cov Inflation

Mild clamp for impossible acceleration plus covariance inflation for suspicious but plausible spikes.

Pros: most robust for humanoid impact spikes.

Cons: more parameters and more implementation complexity.

Decision: choose Option C for MVP because the user prefers an "一步到位" implementation. Clamp is only allowed for values above the hard upper bound (`imu_humanoid_acc_clamp_norm_max`); softer spikes should preserve raw acceleration and only increase process noise. Implementation must still keep the feature behind `fast_lio.imu_filter` so rollback to unfiltered behavior is one YAML change.

## Proposed Config Surface

Names are provisional:

```yaml
fast_lio:
  imu_filter: false

  imu_humanoid_acc_spike_enabled: true
  imu_humanoid_acc_norm_min: 4.0          # conservative fallback; calibration tool should provide final value
  imu_humanoid_acc_norm_max: 18.0         # conservative fallback; calibration tool should provide final value
  imu_humanoid_acc_delta_max: 8.0         # conservative fallback; calibration tool should provide final value
  imu_humanoid_acc_clamp_norm_max: 18.0   # conservative fallback; calibration tool should provide final value
  imu_humanoid_acc_cov_scale_on_spike: 10.0
  imu_humanoid_spike_log_rate_hz: 1.0
```

Default numeric ranges are placeholders only. The intended workflow is to run the calibration node on representative walking data and then copy the resulting p99.5-based values into YAML. The code should handle missing keys with safe defaults, but production tuning should be data-driven.

## Calibration Tool Requirement

Add a Python ROS2 node, proposed path:

```text
scripts/imu_calib/humanoid_imu_spike_calib.py
```

It should be a direct-run script, not part of the compiled package:

```bash
python3 scripts/imu_calib/humanoid_imu_spike_calib.py --ros-args -p imu_topic:=/livox/imu
```

Expected behavior:

* Subscribe to `sensor_msgs/msg/Imu`.
* Track acceleration norm `|acc|`, sample-to-sample acceleration delta `|acc[k]-acc[k-1]|`, gyro norm, and optional gyro delta.
* Print periodic statistics while running.
* On shutdown, print copy-pasteable YAML recommendations for:
  * `imu_humanoid_acc_norm_min`
  * `imu_humanoid_acc_norm_max`
  * `imu_humanoid_acc_delta_max`
  * `imu_humanoid_acc_clamp_norm_max`
  * `imu_humanoid_acc_cov_scale_on_spike`
* Use percentile-based suggestions, for example p95/p99/p99.5, so the user can tune from real walking data.
* Support a warmup duration to ignore startup transients.
* Optionally write CSV/JSON summary to `/tmp` or a user-provided path.
* Segment the stream into coarse phases using IMU statistics, without requiring manual labels:
  * static
  * static -> moving startup
  * walking/moving
  * stopping/moving -> static
* Print both global statistics and per-phase statistics. The first implementation can use simple heuristics based on rolling acceleration/gyro norm standard deviation and transition edges; it does not need supervised labels.
* Default YAML recommendation policy uses p99.5 for acceleration norm and acceleration delta thresholds.
* The tool should still print p95, p99, p99.5, and p99.9 so the user can inspect sensitivity before copying values.

Calibration workflow:

1. Replay or run a representative humanoid walking dataset.
2. Run the calibration node against the IMU topic.
3. Include static, static -> moving startup, normal walking, turning, and stopping phases in the recording.
4. Review global and per-phase percentile statistics.
5. Copy the recommended YAML values into `config/loclite_livox.yaml`.
6. Validate with Fast-LIO logs: spike count is nonzero during steps, ZUPT exits immediately on motion, no gyro lag is introduced.

## Replay Validation Requirement

Use at least one representative humanoid dataset containing:

* static before walking
* static -> moving startup
* normal walking
* turning if available
* stopping / moving -> static

Run validation with:

1. `fast_lio.imu_filter: false` baseline.
2. `fast_lio.imu_filter: true` with calibrated humanoid filter values.

Record and compare:

* whether fly-away occurs after static -> moving transition
* whether `ZUPT applied` persists after movement starts
* filter aggregate logs: spike count/rate, clamp count/rate, covariance-inflation count/rate
* Fast-LIO diagnostics: effective points, residual mean/max, `quality_good`, abnormal dt, rejected updates
* localization state transitions: Initializing / Good / Degraded / Lost
* rough pose continuity: no large single-frame output jump during startup unless rejected by smoother

Pass criteria:

* The filtered run does not introduce gyro-lag symptoms or deskew degradation compared with baseline.
* Static drift remains controlled with ZUPT enabled.
* Startup walking no longer causes immediate fly-away in the target dataset.
* Hard clamp count should be low; frequent hard clamps indicate thresholds are too low or the sensor mounting is mechanically problematic.
* Covariance inflation may occur during steps; this is expected and should correlate with walking impacts.

## Out Of Scope

* Online LiDAR-IMU extrinsic estimation.
* Foot contact detection from leg kinematics.
* Replacing Fast-LIO with a humanoid-specific state estimator.
* Removing ZUPT entirely.
* Coupling humanoid filter spike detection directly into ZUPT enter/exit logic.
* Tuning KISS relocalization or NDT relocalization gates in this task.

## Technical Notes

* Current filter: `include/hikari_loclite/lio/imu_filter.h`.
* Replace the legacy gyro smoother implementation instead of preserving a runtime `legacy` mode. The existing `medianFilter`, `movingAverage`, and `rateLimit` gyro pipeline should be deleted or made unreachable by removal, not left dormant.
* Filter call site: `include/hikari_loclite/lio/imu_processing.hpp`, inside `ImuProcess::UndistortPcl()`.
* Config load: `src/lio/fast_lio_fixed_map.cpp`.
* Current YAML keys: `config/loclite_livox.yaml` under `fast_lio`.
* ZUPT static detector: `include/hikari_loclite/lio/static_detector.h`.
* Existing calibration-style scripts live under `scripts/zupt_calib/`; use that style as a reference but create a separate humanoid IMU spike calibration tool.
* Relevant risk: gyro lag breaks deskew because `angvel_avr` drives `math::Exp(angvel_avr, dt)` for point compensation.

## Open Questions

* Is the PRD ready to start implementation?
