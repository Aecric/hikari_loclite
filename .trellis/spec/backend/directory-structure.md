# Directory Structure

`hikari_loclite` is an independent ROS2 package under
`/home/aecriclin/3d_slam_ws/src/hikari_loclite`. Do not place new code inside
`lightning-lm` or turn this package into a wrapper around full `lightning`.

## Current Layout

- `CMakeLists.txt` and `package.xml` define the standalone ROS2 package.
- `include/hikari_loclite/common/` holds shared math, point, IMU, odom, and
  navigation-state types such as `point_def.h`, `nav_state.h`, and
  `eigen_types.h`.
- `include/hikari_loclite/lio/` and `src/lio/` hold Fast-LIO-related ESKF,
  preprocessing, IMU processing, and Scan Context code.
- `include/hikari_loclite/ivox3d/` holds the custom iVox map implementation.
- `include/hikari_loclite/ndt/` and `src/ndt/` hold the extracted NDT OMP and
  voxel covariance implementation.
- `include/hikari_loclite/log.h` is the local logging shim that replaces glog.
- `thirdparty/nanoflann/` is the local nearest-neighbor dependency used by Scan
  Context.

The build document defines the intended product layout for upcoming system code:
`src/app/` for executables, `src/system/` for ROS2 orchestration, `src/lio/` for
fixed-map Fast-LIO, `src/ndt/` for NDT validation, and `src/reloc/` for bounded
relocalization.

## Module Boundaries

- Keep algorithm code in `lio`, `ndt`, `ivox3d`, `reloc`, or `common`; keep ROS2
  subscriptions, publishers, TF, and parameters in `system` or `app`.
- Keep fixed-map localization separate from mapping. Runtime tracking must not
  insert the current scan into the fixed map.
- Keep extracted code minimal. It is acceptable to copy small proven algorithms
  from `lightning-lm`, but do not link `lightning.libs`.
- Preserve the existing PCL/Eigen point-type model. Custom point types live in
  `include/hikari_loclite/common/point_def.h`.

## Namespaces And Compatibility

Current extracted code still uses the `lightning` namespace in many headers and
sources. New product orchestration code may use `hikari::loclite`, as shown in
`hikari_loclite_build_2026-06-10.md`, but do not perform broad namespace churn
while making feature changes. Namespace migration should be explicit and tested.

## Forbidden Includes

Do not directly include full-framework headers such as:

- `core/system/loc_system.h`
- `core/localization/localization.h`
- `core/localization/lidar_loc/lidar_loc.h`
- `core/localization/pose_graph/pgo.h`
- `ui/pangolin_window.h`

Use the full system only as a reference for ROS topic wiring, Fast-LIO frontend
behavior, NDT OMP calling conventions, or Scan Context database format.
