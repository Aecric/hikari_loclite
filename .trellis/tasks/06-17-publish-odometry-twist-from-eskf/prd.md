# Publish Odometry Twist From ESKF

## Goal

Publish a rigorous, clearly documented `twist` field on `hikari_loc/odom` so downstream consumers can use the LIO/ESKF velocity as a reference without inferring velocity from pose differencing.

## What I Already Know

* `hikari_loc/odom` is published from `LocLiteNode::PublishPose`.
* The current odometry message fills `header`, `child_frame_id`, and `pose.pose`, but does not fill `twist`.
* The normal odometry branch publishes pose as `map -> lidar_frame` (`map -> livox_frame` in the current config).
* The frozen LOST/TF-freeze branch currently publishes odometry pose as `map -> base_link`, even though the normal odometry branch uses `map -> lidar_frame`.
* `NavState` carries ESKF velocity as `vel_` and exposes it through `GetVel()`.
* `FastLioFixedMap::MatchAgainstFixedMap` returns `kf_.GetX()` as `state_point_`, so the node already receives the ESKF velocity in `NavState`.
* The ESKF velocity is in the inertial/map/world frame because state propagation uses `pos_dot = vel`.
* `nav_msgs/Odometry.twist` conventionally represents velocity in the `child_frame_id` frame, so publishing must define and respect frame semantics.
* The reliable odometry transform available to this node for this feature is `map -> lidar_frame` / `map -> livox_frame`.
* The node already stores both `frozen_T_map_base_` and `frozen_T_map_lidar_`; `frozen_T_map_lidar_` can be used to keep frozen odometry in the Livox/lidar frame while preserving any separate TF-freeze behavior.

## Assumptions (Temporary)

* The feature should preserve the existing `hikari_loc/odom` topic instead of adding a second velocity topic unless a better reason emerges.
* `base_link` is not guaranteed to be a rigid body suitable for this odometry topic; conversion to `base_link` should be left to downstream TF consumers/modules.
* Frozen pose publication should not report stale physical motion as live odometry velocity.
* Velocity is intended as a reference signal, not as a replacement for wheel odometry or controller-grade velocity feedback.

## Open Questions

* Final confirmation from user.

## Requirements (Evolving)

* Publish linear velocity from ESKF in odometry `twist`.
* Express `twist.twist.linear` in `livox_frame` / the configured `lidar_frame_id`, matching `Odometry.child_frame_id`.
* Do not publish angular velocity in this task; keep `twist.twist.angular` zero because ESKF/NavState does not currently expose an angular velocity posterior state.
* Do not fill `twist.covariance` in this task; leave covariance at the ROS message default and document that covariance is currently unavailable.
* During frozen LOST / TF-freeze odometry publication, set all twist components to zero.
* During frozen LOST / TF-freeze odometry publication, publish `/hikari_loc/odom` as `map -> lidar_frame` using `frozen_T_map_lidar_`; do not publish odometry as `map -> base_link`.
* Do not add a new runtime/config switch; when `publish_odom` is true, fill odometry twist according to this PRD.
* Convert ESKF linear velocity to Livox frame by rotation only: `v_livox = R_map_livox.inverse() * state.GetVel()`.
* Do not add IMU-to-Livox lever-arm velocity compensation in this task because angular velocity is not available as an ESKF posterior state.
* Do not require new automated tests for this task; build verification is required and final runtime validation will be performed by the user.
* Define frame semantics explicitly in code comments/config/docs so downstream users know how to consume it.
* Preserve existing pose/TF behavior unless intentionally changed by the PRD.
* Do not change normal `hikari_loc/odom` to `base_link`; keep the odometry topic tied to the sensor rigid body frame and let downstream consumers transform when needed.
* Do not invent a `base_link` odometry frame in this node; use only `map -> lidar_frame` / `map -> livox_frame`.
* Preserve TF-freeze behavior separately from odometry semantics: if TF still needs frozen `map -> base_link`, keep using `frozen_T_map_base_` for TF only, not for `/hikari_loc/odom`.

## Acceptance Criteria (Evolving)

* [ ] `hikari_loc/odom.twist.twist.linear` is non-empty and derived from ESKF velocity during normal tracking.
* [ ] Twist frame semantics are documented and match `child_frame_id`.
* [ ] `hikari_loc/odom.twist.twist.angular` remains zero and is documented as intentionally unavailable.
* [ ] `hikari_loc/odom.twist.covariance` behavior is documented as intentionally not provided in this task.
* [ ] Frozen LOST publication explicitly publishes zero twist and does not expose misleading live velocity.
* [ ] Frozen LOST odometry uses `child_frame_id == lidar_frame_id_` and `pose == frozen_T_map_lidar_` instead of `base_frame_id_` / `frozen_T_map_base_`.
* [ ] No new runtime parameter is required to enable odometry twist.
* [ ] Implementation documents that Livox-frame linear velocity ignores lever-arm contribution.
* [ ] Implementation exposes the expected odometry fields clearly enough for user-side runtime validation.
* [ ] Build passes for the package.

## Definition of Done

* Tests added only if implementation naturally exposes a low-friction helper; otherwise manual verification is acceptable for this narrow change.
* Build/lint/type checks run where available.
* Runtime behavior documented in config comments or project docs if the public topic semantics change.
* Rollback is straightforward by disabling or reverting the twist-fill logic.

## Out of Scope (Explicit)

* Fusing wheel odometry or adding a new wheel-speed observation model.
* Designing a controller-grade velocity estimator.
* Changing localization state machine behavior unless needed for correct twist semantics.
* Publishing `hikari_loc/odom` as `base_link` odometry.
* Solving non-rigid `base_link` modeling inside the localization node.
* Publishing level-frame velocity in `hikari_loc/odom.twist`.
* Estimating or publishing angular velocity from IMU samples or pose differencing.
* Publishing ESKF velocity covariance.
* Compensating IMU-to-Livox lever-arm velocity with gyro or pose-differenced angular velocity.

## Technical Notes

* Normal odom publication: `src/system/loclite_node.cpp`, `LocLiteNode::PublishPose(const NavState&)`.
* Frozen odom publication: `src/system/loclite_node.cpp`, `LocLiteNode::PublishPose(const SE3&, double)`.
* Frozen pose storage: `include/hikari_loclite/system/loclite_node.hpp` has `frozen_T_map_base_` and `frozen_T_map_lidar_`.
* ESKF state/API: `include/hikari_loclite/common/nav_state.h`, `include/hikari_loclite/lio/eskf.hpp`.
* LIO state return path: `src/lio/fast_lio_fixed_map.cpp`, `state_point_ = kf_.GetX()`.

## Technical Approach

* Normal odometry:
  * Continue publishing `/hikari_loc/odom` with `header.frame_id = map_frame_id_` and `child_frame_id = lidar_frame_id_`.
  * Continue publishing pose as `T_map_lidar`.
  * Fill `twist.twist.linear` with `T_map_lidar.so3().inverse() * state.GetVel()`.
  * Leave `twist.twist.angular` as zero.
  * Leave `twist.covariance` at message default.
* Frozen LOST / TF-freeze odometry:
  * Publish `/hikari_loc/odom` with `header.frame_id = map_frame_id_`, `child_frame_id = lidar_frame_id_`, and `pose.pose = frozen_T_map_lidar_`.
  * Publish zero twist.
  * Keep any separate frozen TF behavior explicit; odometry must not switch to `base_frame_id_`.
* Implementation should prefer a small local helper for filling linear twist if it keeps the code readable, but automated tests are not required by this PRD.

## Decision (ADR-lite)

**Context**: Downstream consumers need reference velocity from ESKF, but this node only owns reliable `map -> livox_frame` odometry semantics. `base_link` is not a suitable rigid odometry frame here, and angular velocity/covariance are not exposed as ESKF posterior outputs.

**Decision**: Publish only ESKF-derived linear velocity in `/hikari_loc/odom.twist`, expressed in `livox_frame` / `lidar_frame_id_`. Do not publish angular velocity, covariance, level-frame velocity, or base-link odometry. Make frozen odometry use the same lidar child frame and zero twist.

**Consequences**: The odometry topic has consistent frame semantics and gives downstream users a useful reference velocity. The linear velocity ignores IMU-to-Livox lever-arm velocity and has no covariance; those are explicitly out of scope until angular velocity/covariance are modeled rigorously.

## Implementation Plan For Coding Agent

1. Update normal `LocLiteNode::PublishPose(const NavState&)` to fill `odom_msg->twist.twist.linear` from ESKF velocity rotated into `lidar_frame_id_`.
2. Update frozen odometry publishing so `/hikari_loc/odom` uses `frozen_T_map_lidar_` and `child_frame_id = lidar_frame_id_`, with all twist components zero.
3. Keep TF-freeze behavior separate from odometry semantics; do not convert `/hikari_loc/odom` to `base_link`.
4. Add concise comments near twist filling explaining frame, zero angular velocity, missing covariance, and ignored lever-arm term.
5. Build the package; runtime validation is user-owned.
