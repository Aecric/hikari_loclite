# Error Handling

This package favors explicit return values and validation gates over exceptions.
Use `bool` for simple initialization/load/run outcomes and result structs for
localization decisions that need diagnostics.

## Local Patterns

- `FastLioFixedMap::Init`, `LoadFixedMapFromConfig`, `LoadFixedMap`,
  `RebuildLocalMapAround`, `ResetToMapPose`, and `RunOnce` should return `false`
  when required data is unavailable or validation fails.
- `NdtCorrector::Validate` should return a result object containing `valid`,
  confidence, inlier ratio, and pose-delta fields. A rejected candidate is a
  normal control-flow outcome, not a process error.
- `RelocManager::TryRelocalize` should return an invalid candidate when it is
  disarmed, has no scan, times out, or cannot find a bounded SC match.
- `src/app/run_loclite_online.cpp` should return non-zero only when node
  initialization fails.

Reference patterns:

- `include/hikari_loclite/lio/scan_context.h` uses `bool SaveDatabase` and
  `bool LoadDatabase` for persistence failures.
- `include/hikari_loclite/log.h` provides stream-style `LOG(...)` macros for
  warnings and errors.
- `hikari_loclite_build_2026-06-10.md` defines `NdtResult` and
  `RelocCandidate` as validity-carrying structs.

## Validation Rules

- Do not reset Fast-LIO state from `/initialpose`, Scan Context, or another
  external candidate until NDT validation succeeds.
- Reject NDT corrections that exceed configured translation or rotation deltas.
  The build document defaults are `max_delta_trans_m: 1.0` and
  `max_delta_rot_deg: 10.0`.
- Reject pose smoother output jumps beyond configured limits instead of
  publishing discontinuous pose.
- Treat empty clouds, missing fixed maps, and failed PCD loads as recoverable
  failures that leave the current state unchanged.

## Avoid

- Do not throw exceptions through ROS2 callbacks or hot-path scan processing.
- Do not silently accept invalid candidate poses.
- Do not mutate the fixed map or ESKF state before validation has passed.
- Do not crash the node for normal localization states such as LOST,
  uninitialized, bad initial pose, or temporarily missing scans.
