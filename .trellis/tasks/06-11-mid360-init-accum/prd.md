# MID360 Initialpose Accumulation Before FC

## Problem

MID360 single-frame point clouds are too sparse for reliable FC/NDT validation.
The current `/initialpose` path validates against one deskewed scan per retry,
then gives up after a small retry count. Lightning waits in place, accumulates
several seconds of deskewed scans, and runs FC only after the accumulated cloud
has enough density.

## Goal

Implement initial pose validation with a bounded rolling accumulation window:

- `/initialpose` enters `Initializing`.
- While pending, collect deskewed scans at the fixed initial pose.
- Accumulate for 1-3 seconds or until configured density conditions are met.
- Downsample the accumulated cloud before FC/NDT validation.
- For tilted MID360 installs, delay candidate construction until LIO/IMU has a
  gravity-aligned roll/pitch estimate, then combine `/initialpose` yaw with the
  measured roll/pitch before FC/NDT validation.
- Call `ResetToMapPose()` only after accumulated-cloud validation succeeds.
- On validation failure, continue with a fresh accumulation window instead of
  giving up after a few single frames.

## Requirements

- Add YAML knobs under `reloc`:
  - `init_accum_enabled`
  - `init_accum_min_frames`
  - `init_accum_min_points`
  - `init_accum_window_sec`
  - `init_accum_max_wait_sec`
  - `init_accum_voxel_leaf`
- Keep fixed-map invariant: accumulated init scans must not be inserted into the
  fixed map or iVox target.
- Use accumulated scan only for `/initialpose` validation.
- Do not validate a pending `/initialpose` with roll/pitch forced to zero; wait
  for LIO/IMU gravity compensation first.
- Keep scan accumulation bounded by time and points.
- If validation fails after a ready accumulation window, clear the accumulation
  buffer and begin a new window while keeping the pending initial pose.
- If `init_accum_max_wait_sec` is exceeded without a successful validation,
  return to `WaitForInitialPose`.

## Validation

- Build in the Docker/ROS environment.
- `git diff --check` must pass.
- Runtime logs should show accumulation readiness and FC/NDT validation attempts
  using accumulated frame/point counts.
