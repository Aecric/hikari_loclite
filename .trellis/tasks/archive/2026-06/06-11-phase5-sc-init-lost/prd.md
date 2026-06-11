# Phase5 SC init and LOST relocalization

## Goal

Complete Phase 5 from `hikari_loclite_build_2026-06-10.md`: wire Scan Context into `hikari_loclite` as a bounded initialization and LOST-recovery path. The node should be able to load a Lightning-format map directory with `sc_database.bin` and `poses.txt`, run one-shot SC candidate lookup when configured, validate the candidate through NDT, reset Fast-LIO only after validation succeeds, and disarm SC after localization becomes Good.

## What I Already Know

- The current runtime has fixed-map Fast-LIO, NDT validation/correction, `/initialpose` sticky retry, stability gate, loc_state/marker outputs, and placeholder SC debug publishers.
- Current startup intentionally waits for `/initialpose`; comments say SC is not wired yet.
- The current `hikari_loc/sc_reloc` service logs the request but returns `success=false` with `SC relocalization not available`.
- `ScanContextManager` already exists in `include/hikari_loclite/lio/scan_context.h` and supports `LoadDatabase`, `Query`, and `QueryTopK`.
- `ScanContextManager` returns `kf_id`, `sc_dist`, and `yaw_diff_rad`; Phase 5 still needs `kf_id -> map pose` lookup through map metadata such as `poses.txt`.
- The build document requires SC to run only in init/LOST, not as a permanent background worker.
- Existing specs require all SC candidates to pass NDT before `ResetToMapPose()`.
- Current dirty working tree contains previous in-progress changes; this task must work with them and avoid reverting unrelated edits.

## Requirements

- Add a lightweight `RelocManager` or equivalent module under the project-owned relocalization boundary (`include/hikari_loclite/reloc/`, `src/reloc/` preferred by spec).
- Load SC config from YAML and/or `--map_path` with the expected Lightning map layout:
  - `<map_path>/sc_database.bin`
  - `<map_path>/poses.txt`
  - optional explicit YAML fallback paths matching the build document.
- Support bounded SC behavior:
  - `auto_on_init`
  - `auto_on_lost`
  - `disable_after_good`
  - `max_runtime_sec`
  - `sc_top_k`
  - `sc_cooldown_sec`
  - `sc_enabled`
- On cold start, if `reloc.auto_on_init` and SC data are available, attempt SC initialization using the latest deskewed scan rather than requiring `/initialpose`.
- In LOST state, arm SC recovery when `reloc.auto_on_lost` is enabled and attempt bounded relocalization before falling through to `WAIT_FOR_INITIALPOSE` timeout behavior.
- Manual `ros2 service call /hikari_loc/sc_reloc std_srvs/srv/Trigger "{}"` should request one SC attempt and bypass `/initialpose` blackout because it is an explicit user command.
- `/initialpose` blackout must prevent automatic SC injection during `system.external_pose_blackout_sec`; stale automatic SC candidates computed during blackout must be dropped.
- Every SC candidate must be converted into a map-frame candidate pose and validated with `NdtCorrector::Validate()` before any Fast-LIO state mutation.
- On accepted SC+NDT relocalization:
  - call `ResetToMapPose()` with the NDT-refined pose
  - reset the smoother
  - reset LOST/init retry timers as needed
  - enter Initializing with stability gate armed when enabled, or Good when disabled
  - disarm RelocManager after Good or successful relocalization
- On rejected or unavailable SC candidate:
  - leave Fast-LIO state unchanged
  - keep state machine behavior deterministic
  - log a concise reason with enough data for field tuning.
- Publish SC debug topics only when there are subscribers:
  - `hikari_loc/sc/init_guess`
  - `hikari_loc/sc/candidates`
  - `hikari_loc/sc/gravity_check`
  - `hikari_loc/sc/accum_cloud` if an accumulation window is implemented.

## Acceptance Criteria

- [ ] With a map directory containing `global.pcd`, `sc_database.bin`, and `poses.txt`, startup logs show SC database and pose metadata loaded.
- [ ] With `reloc.auto_on_init: true`, a valid live scan can produce a SC candidate, pass NDT, call `ResetToMapPose()`, and progress through stability gate to Good.
- [ ] With `reloc.auto_on_lost: true`, Lost state arms SC; a valid candidate can recover localization and then disarm SC again.
- [ ] Good/Degraded normal tracking does not run continuous SC queries or keep a background SC worker consuming CPU.
- [ ] Bad SC candidates or failed NDT validation leave Fast-LIO state unchanged.
- [ ] `/initialpose` remains authoritative during its blackout window and automatic SC does not override it.
- [ ] Manual `hikari_loc/sc_reloc` produces a real attempt and reports success/failure in the service response.
- [ ] Existing `/initialpose` NDT validation behavior remains intact.
- [ ] `colcon build --packages-select hikari_loclite --cmake-args -DCMAKE_BUILD_TYPE=Release` passes.
- [ ] `git diff --check` passes.

## Out Of Scope

- No KISS-Matcher integration.
- No FP brute-force fallback unless it already exists in the code path being reused.
- No PGO, dynamic map, full Lightning runtime, or permanent SC/KISS background workers.
- No map building or SC database generation in this runtime task.
- No broad namespace migration or unrelated formatting cleanup.

## Technical Notes

- Primary contract: `hikari_loclite_build_2026-06-10.md`, especially Phase 5 and interface section 21.
- Relevant specs:
  - `.trellis/spec/backend/localization-architecture.md`
  - `.trellis/spec/backend/runtime-and-relocalization.md`
  - `.trellis/spec/backend/error-handling.md`
  - `.trellis/spec/backend/logging-guidelines.md`
  - `.trellis/spec/backend/quality-guidelines.md`
  - `.trellis/spec/backend/directory-structure.md`
  - `.trellis/spec/guides/cross-layer-thinking-guide.md`
  - `.trellis/spec/guides/code-reuse-thinking-guide.md`
- Existing SC code:
  - `include/hikari_loclite/lio/scan_context.h`
  - `src/lio/scan_context.cc`
- Existing orchestration placeholders:
  - `include/hikari_loclite/system/loclite_node.hpp`
  - `src/system/loclite_node.cpp`
- Existing state machine:
  - `include/hikari_loclite/system/loclite_state_machine.hpp`

## Open Question

- MVP should use only `poses.txt` for `kf_id -> pose`, or should it also validate against keyframe cloud files when available?

## Recommended Answer

Use only `poses.txt` for the MVP. It is enough to turn `kf_id + yaw_diff` into an initial map-frame pose, keeps Phase 5 bounded, avoids keyframe point-cloud I/O on the hot path, and still relies on NDT as the hard validation gate.
