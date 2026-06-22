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

### Scenario: Odometry Pose And Twist Semantics

### 1. Scope / Trigger

- Trigger: any change to `hikari_loc/odom`, `PublishPose`, frame IDs, frozen
  LOST publication, or downstream velocity semantics.
- Owner: `LocLiteNode`, because it owns ROS odometry publication and TF output.

### 2. Signatures

- ROS topic: `hikari_loc/odom`
- Message type: `nav_msgs/msg/Odometry`
- Normal publisher path: `LocLiteNode::PublishPose(const NavState& state)`
- Frozen/recovery immediate publisher path: `LocLiteNode::PublishPose(const SE3& T_map_lidar, double ts)`

### 3. Contracts

- `header.frame_id` must be `common.map_frame_id`.
- `child_frame_id` must be `system.lidar_frame_id`.
- `pose.pose` must represent `map -> lidar_frame_id`, not `map -> base_frame_id`.
- During normal tracking, `twist.twist.linear` must be ESKF `NavState::GetVel()`
  rotated from map/world frame into `child_frame_id`:
  `v_lidar = R_map_lidar.inverse() * v_map`.
- `twist.twist.angular` stays zero until an angular velocity posterior is
  explicitly exposed.
- `twist.covariance` stays at the ROS message default until velocity covariance
  is explicitly modeled.
- LOST/frozen publication must publish frozen `map -> lidar_frame_id` pose and
  zero twist. TF freeze may still publish `map -> base_frame_id` separately.
- Do not add IMU/lidar lever-arm velocity compensation unless angular velocity
  is available and the frame contract is updated here.

### 4. Validation & Error Matrix

- Missing ESKF velocity -> publish zero only if the caller is explicitly a
  frozen/recovery immediate path; normal tracking should use `NavState::GetVel()`.
- LOST/frozen state -> publish zero twist to avoid reporting stale physical
  motion as live odometry velocity.
- Need base-frame velocity -> downstream should transform through TF; this node
  does not publish base-link odometry.

### 5. Good/Base/Bad Cases

- Good: normal `hikari_loc/odom` has `child_frame_id=livox_frame` and linear
  twist in that same Livox/lidar frame.
- Base: `runtime.publish_odom=false` disables odometry publication entirely.
- Bad: pose is `map -> lidar_frame_id` but twist remains in map/world frame.
- Bad: frozen LOST odometry switches `child_frame_id` to `base_frame_id`.

### 6. Tests Required

- Build: run the containerized Release build from the Build And Dependencies spec.
- Runtime smoke when data is available: echo one normal `hikari_loc/odom` message
  and assert `child_frame_id == system.lidar_frame_id` and nonzero linear twist
  while moving.
- Runtime LOST/freeze smoke when available: force LOST and assert odometry keeps
  `child_frame_id == system.lidar_frame_id` with zero twist.

### 7. Wrong vs Correct

#### Wrong

Publishing ESKF velocity directly into `odom.twist.twist.linear` while
`child_frame_id` is `livox_frame`; that reports map-frame velocity under a
child-frame twist contract.

#### Correct

Rotate ESKF map-frame linear velocity into the odometry child frame and leave
angular/covariance fields unset until those states are modeled.

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

### WaitForInitialPose <-> reloc-armed invariant

`WaitForInitialPose` is the persistent auto-relocalization state, **not** a
"sit and wait for a human" state. Every transition into it MUST go through the
single helper `LocLiteNode::EnterWaitForInitialPose(reason)`, which enforces the
invariant:

> in `WaitForInitialPose`, `reloc.auto_on_init && RelocReady()` ⟹ reloc armed;
> otherwise disarmed.

This holds regardless of how the state was entered — cold start, `lost_timeout`
(from `Lost`), or an `/initialpose` rejected by NDT (`init_accum_timeout`,
`init_retry_exhausted`, `init_accum_rejected`). The earlier code disarmed reloc
on the `lost_timeout` path and left the rejection paths disarmed, so after a
loss or a bad `/initialpose` the node sat idle waiting for a human — the helper
removes that asymmetry. Do not call `state_machine_.SetWaitForInitialPose()`
directly; route through the helper so the invariant cannot drift.

`/initialpose` still interrupts: `OnInitialPose` disarms reloc, bumps
`reloc_gen_` (invalidates in-flight KISS workers), opens the blackout window,
and enters `Initializing`. If validation succeeds → `Good`; if it is rejected,
the rejection path re-enters `WaitForInitialPose` and the helper re-arms auto
relocalization. `Lost` is independent of this invariant — it runs its own short
local-recovery window and (optionally, `auto_on_lost`) global reloc, then hands
off to `WaitForInitialPose` on timeout.

#### `max_runtime_sec <= 0` = unlimited

`reloc.max_runtime_sec <= 0` (convention: `-1`) means **no timeout disarm**:
armed reloc retries until success / a valid `/initialpose` / manual stop.
`RelocManager::RuntimeUnlimited()` is the single predicate; both max-runtime
guards (`RelocManager::TryRelocalize` and the async KISS dispatch in
`MaybeDispatchKissReloc`) gate on `!RuntimeUnlimited()` rather than a bare
`> 0.0`, and `Arm()` logs the unlimited mode for auditability. A finite
`max_runtime_sec > 0` caps a single armed episode: it disarms after the window
and stays disarmed until the state re-enters `WaitForInitialPose` (or a new
`/initialpose`). Re-arming is tied to **state entry**, never to the timeout, so
a finite cap never silently becomes an infinite retry loop. WAIT has no other
timeout that disarms auto reloc, so `<= 0` yields genuinely unlimited cold-start
and post-loss relocalization; `max_runtime_sec` does not bound the `Lost`
window (that is `system.lost_timeout_sec`).

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

## Relocalization Backend Selector

`RelocManager` dispatches on `reloc.reloc_backend` (`kiss` | `sc`, **default
`kiss`**). The selector is the cross-layer contract every call site must respect.

### Contract

- `RelocBackend Backend()` / `BackendIsKiss()` / `BackendIsSc()`.
- `bool RelocReady()` — backend-agnostic readiness predicate. **Use this, not
  `ScEnabled()`**, to guard arm / accumulate / trigger in the node
  (`kiss` → KISS target injected & under point guard; `sc` → SC enabled).
- `TryRelocalize(scan, current_imu_pose, current_time, NdtCorrector* ndt)` and
  `ManualRelocalize(scan, current_imu_pose, NdtCorrector* ndt)` both take the NDT
  corrector now — the KISS path runs NDT internally (yaw sweep). SC backend may
  pass it through but does not require it.
- `SetKissTarget(FixedMapCloud())` is called once after the fixed map loads
  (only meaningful for `kiss`). Target is the whole `global.pcd` pre-voxelized
  and cached as `vector<Vec3f>`; small map needs no crop. `max_target_pts` guard
  warns-and-skips (target stays not-ready) rather than crashing on a huge map.
- `RelocCooldownSec()` (config `reloc.reloc_cooldown_sec`, old
  `sc_cooldown_sec` still read as fallback) is shared by KISS and SC automatic
  attempts. Do not put shared scheduling keys under SC-only comments.
- `QueryAccumFrames()` and the query accumulation config
  (`reloc.query_accum_frames`, `reloc.query_accum_voxel_leaf`,
  `reloc.query_accum_max_rel_trans_m`; old `sc_accum_*` keys still read as
  fallback) are shared by KISS and SC.
- `RelocMaxDeltaTransM()` / `RelocMaxDeltaRotDeg()` (config
  `reloc.reloc_max_delta_*`, renamed from `sc_max_delta_*`, old keys still read
  as fallback) — shared NDT-validation delta gate for both backends; wider than
  `ndt.max_delta_*` because both SC sector quantization and KISS yaw-sweep start
  offsets push the correct candidate's delta past the `/initialpose` gate.

### KISS path (`RunKissOnce`)

```
accumulate 20 frames (shared with SC) → gravity-align to level
  → KISS.MatchGlobal(query_level)            # global 6DOF, no initial guess
  → gate: valid && rot_inliers>=min_rotation_inliers && final_inliers>=min_final_inliers
  → compose T_map_lidar = T_map_level * SE3(R_body_level, 0)   # level→lidar frame
  → yaw sweep [-range,+range]@step (deg): NdtCorrector::Validate each start, keep best TP
  → any NDT-valid? pose = best NDT solution, source="kiss"/"manual_kiss"
                 : reject (clean return + log), wait cooldown / → WAIT
```

Config: `reloc.kiss_voxel_size` (0.3), `target_pre_voxel` (0.2),
`max_target_pts` (800000), `min_rotation_inliers` (50), `min_final_inliers`
(30), `yaw_refine_range_deg` (9), `yaw_refine_step_deg` (3). KISS rotation `R`
**must** be orthonormalized via `Eigen::Quaterniond` before constructing `SO3`,
or Sophus asserts.

### SC retained but disabled by default

All SC code (`ScanContextManager`, `LoadPoses`, `KfIdToMapPose`, `kf_poses_`,
`RunScanContextOnce`, the `sc/*` debug topics) is kept compilable for A/B and
fallback. When `reloc_backend=kiss` (default) the node does **not** load the SC
database / poses and does **not** publish `sc/candidates|init_guess|gravity_check`
(it reuses `sc/accum_cloud` via `PublishKissAccumCloud()` for the KISS query
cloud). Manual service name `hikari_loc/sc_reloc` is preserved for downstream
compatibility; it dispatches by backend internally.

## LOST Recovery

- LOST should arm `RelocManager`.
- `RelocManager::TryRelocalize()` should return immediately if disarmed or if
  the scan is empty.
- A successful candidate (KISS or SC) must pass NDT validation before state
  reset. For the KISS backend the candidate pose is already the yaw-sweep NDT
  solution, so node-side re-validation confirms it (delta ≈ 0) rather than
  re-aligning.
- After successful relocalization, set Good and disarm relocalization.
- In Good state, disarm relocalization so relocalization threads or workers do
  not consume CPU.

### Convention: one clock domain per comparison (gotcha, bag-verified)

`LocLiteNode` carries two time domains: **scan/measurement time** `ts` (bag
time during replay — can be weeks behind) and **wall clock**
`this->now().seconds()`. Any comparison must keep both sides in one domain:

- `/initialpose` blackout: deadline set from wall clock, compared against wall
  clock (`loclite_node.cpp` `TryScRelocalize`). The original code compared the
  wall-clock deadline against `ts`; during bag replay `ts` is always smaller,
  so after one `/initialpose` automatic SC was **permanently** blacked out —
  invisible on the real robot (scan ts ≈ wall), fatal for every bag-based SC
  test. Fixed in commit 3a6e9b8.
- Reloc cooldown: `ts` vs `last_sc_attempt_ts_` (both scan time) — consistent.
  The member name is historical; KISS also uses it.
- Reloc runtime limit / Arm timestamps: wall vs wall — consistent.

When adding any new timer/deadline, state the domain in the member's comment
(see `blackout_deadline_sec_`) and log both sides on the skip path.

### Reloc attempt cadence (LOST window budget)

The LOST window is only `system.lost_timeout_sec` (5 s) long before the node
disarms relocalization and drops to `WAIT_FOR_INITIALPOSE`. Budget inside it:
Arm clears the accumulation buffer → buffer refills at frame rate
(~2 s for 20 frames) → first real query must fire immediately after.
Therefore the **insufficient-frames skip must not consume the reloc cooldown**:
`last_sc_attempt_ts_` is written only when a real backend query is dispatched
or executed (KISS async worker dispatch / SC `QueryTopK`). The original SC
ordering burned the only in-window attempt on a `frames=1/20` skip and SC never
queried before WAIT; the same budget rule applies to KISS query dispatch.

## Pose Output Gating

Use a smoother/gate to prevent unrealistic published pose jumps:

- Reject output jumps beyond configured translation/rotation limits.
- Apply NDT correction with a configurable gain.

The `LitePoseSmoother` thresholds are yaml-configurable under the `smoother:`
section (each key falls back to the struct default if absent):
`max_output_jump_trans_m: 0.5`, `max_output_jump_rot_deg: 15.0`,
`max_correction_trans_m: 0.5`, `max_correction_rot_deg: 8.0`.

> **Responsibility boundary (do not re-tighten without re-architecting):**
> GOOD-state NDT correction only nudges *moderate* drift — `max_correction_*` is
> the per-cycle clamp on how far a single NDT correction may pull. It was 0.3m/5°,
> which silently discarded any correction once drift exceeded 0.3m (NDT could
> never recover). Widened to 0.5m/8° because the now-calibrated TP gate plus the
> `inlier_ratio` gate already filter bad corrections. **Large drift is not the
> GOOD-state job** — it must trip degradation detection (residual + NDT TP/divergence)
> into Degraded/Lost and hand off to SC relocalization, not be chased by a bigger
> GOOD-state nudge.

## Scenario: IMU-Extrapolated TF Publishing

### 1. Scope / Trigger

- Trigger: reduce downstream `map -> base_frame_id` TF latency between lidar
  frames without changing Fast-LIO's authoritative ESKF state.
- Owner: `LocLiteNode`, because this is a ROS publication policy, not a fixed-map
  tracking update.

### 2. Signatures

- YAML keys under `runtime`:
  - `imu_extrapolate_tf: bool`
  - `imu_extrapolate_max_dt_sec: double`
  - `imu_extrapolate_max_ahead_sec: double`
- Output: existing TF `common.map_frame_id -> system.base_frame_id`.
- Non-output: `hikari_loc/odom` remains lidar-rate unless a later task changes
  that contract explicitly.

### 3. Contracts

- Seed the extrapolator only from accepted lidar-rate `NavState` publication.
- IMU callback may publish extrapolated TF, but must not mutate `FastLioFixedMap`,
  `ESKF`, fixed maps, NDT, or relocalization state.
- Use gyro-only attitude extrapolation with bias from the latest trusted LIO
  state. Use adjacent IMU gyro averaging when a previous IMU sample exists,
  matching Fast-LIO's propagation style.
- Translation is constant-velocity from the latest trusted LIO state. Do **not**
  double-integrate raw acceleration for publication-only position output.
- Lidar/NDT output remains authoritative and re-seeds the extrapolator every
  trusted lidar frame.

### 4. Validation & Error Matrix

- No trusted lidar seed -> do not publish extrapolated TF.
- State is `Lost`, `WaitForInitialPose`, `Uninitialized`, or pose is frozen ->
  invalidate extrapolator and do not publish extrapolated TF.
- IMU timestamp repeats or goes backward -> skip; if it rolls behind the cached
  state timestamp, invalidate until the next lidar seed.
- `imu_dt > imu_extrapolate_max_dt_sec` -> invalidate until next lidar seed.
- `imu_ts - last_lidar_seed_ts > imu_extrapolate_max_ahead_sec` -> invalidate
  until next lidar seed.

### 5. Good/Base/Bad Cases

- Good: `Good`/`Initializing`/`Degraded` after a trusted lidar output, fresh IMU
  arrives at 100-400 Hz -> publish TF at IMU cadence with bounded short-horizon
  extrapolation.
- Base: feature disabled -> existing lidar-rate TF behavior is unchanged.
- Bad: LOST/frozen pose or lidar dropout -> do not keep publishing inertial-only
  drift as localization truth.

### 6. Tests Required

- Static check: `git diff --check`.
- Build: run the Docker/Jazzy build command from `quality-guidelines.md`.
- Runtime smoke when a bag or robot is available: enable
  `runtime.imu_extrapolate_tf`, echo `/tf`, and verify `map -> base_frame_id`
  stamps advance at IMU cadence while `hikari_loc/odom` stays lidar-rate.

### 7. Wrong vs Correct

#### Wrong

Using raw humanoid IMU acceleration in the ROS IMU callback as
`pos += vel*dt + 0.5*a*dt^2`; footstep vibration is not filtered by the full
Fast-LIO measurement window there and will show up as TF jitter.

#### Correct

Use gyro+bias for attitude and constant-velocity translation for the short
publication window; let the next lidar/NDT update correct position and re-seed
the extrapolator.
