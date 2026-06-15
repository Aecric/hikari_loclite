# Logging Guidelines

The extracted algorithm code uses a lightweight stream-style logging shim in
`include/hikari_loclite/log.h`. ROS2 node orchestration code may use `RCLCPP_*`
macros when a node logger is available.

## What To Log

- Initialization failure: missing config path, missing map path, PCD load
  failure, missing SC database, invalid calibration values.
- State transitions: Uninitialized, Initializing, Good, Degraded, Lost, and
  WaitForInitialPose, including the reason string.
- Validation rejection: NDT confidence below threshold, correction too large,
  empty cloud, or invalid relocalization candidate.
- One-time or rate-limited performance warnings: scan processing over budget,
  SC query timeout, repeated IMU timestamp anomalies.

Existing examples:

- `src/lio/eskf.cc` logs non-finite covariance and rejected ESKF updates.
- `include/hikari_loclite/lio/imu_processing.hpp` logs abnormal IMU timing and
  IMU initialization progress.
- `src/lio/scan_context.cc` logs successful SC database load.

## Hot-Path Rules

- Do not log every point, every nearest-neighbor query, or every ESKF
  iteration in normal operation.
- Use `LOG_EVERY_N(...)` or ROS2 throttled logging for repeated callbacks.
- Keep logs deterministic and short enough for embedded devices.

### Throttle macros (`include/hikari_loclite/log.h`)

The local shim provides two throttles, each keyed per call site by `__LINE__`:

- `LOG_EVERY_N(severity, n)` — count-based: emit once every `n` hits. Throttle
  rate is coupled to call frequency (`n=20` at 10 Hz ≈ one line / 2 s, but
  unbounded if the frame rate spikes).
- `LOG_EVERY_T(severity, period_sec)` — time-based: emit at most once per
  `period_sec` (steady monotonic clock, independent of sensor/bag timestamps;
  concurrency-tolerant for logging). First call always passes.

Prefer `LOG_EVERY_T` for **steady-state diagnostics** whose budget is expressed
as "a few lines per second" regardless of frame rate (e.g. the per-frame
`FastLioFixedMap diag` line uses `LOG_EVERY_T(INFO, 5.0)`). Keep anomaly/warning
logs on `LOG_EVERY_N` (or unthrottled if rare) so they surface promptly.

## Severity

- `INFO`: startup, map load summary, SC database load, successful major state
  transition.
- `WARNING`: recoverable quality degradation, rejected candidate, abnormal but
  non-fatal sensor timing.
- `ERROR`: initialization failure, invalid required configuration, failed map
  load, rejected ESKF update that prevents the frame from being used.

Do not add glog as a dependency just to satisfy extracted `LOG(...)` calls; use
the local shim unless there is a deliberate dependency decision.
