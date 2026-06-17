# Quality Guidelines

The quality bar is a lightweight, deterministic fixed-map localization package
for embedded deployment. Prefer small, testable changes that preserve CPU
budget and keep dependencies explicit.

## Required Checks

Build C++ changes in the project Docker image, not directly on the host. The
container workspace root is `/root/slam_ws` and the source tree is selected with
`--base-path src/`:

```bash
docker run --rm -v /home/aecriclin/3d_slam_ws:/root/slam_ws -w /root/slam_ws lightning-jazzy:dev bash -lc 'colcon build --packages-select hikari_loclite --cmake-args -DCMAKE_BUILD_TYPE=Release --base-path src/'
```

When a change affects only docs or Trellis specs, run template-residue and index
checks instead of a full ROS2 build. For C++ changes, build at minimum.

## Forbidden Patterns

- Do not link `lightning.libs`.
- Do not add Pangolin, OpenCV highgui, miao optimizer, g2o, `rosbag2_cpp`, or
  `visualization_msgs` to the first-version runtime. KISS-Matcher is the default
  relocalization backend (user-approved 2026-06-16, see `build-and-dependencies.md`).
- Do not add PGO, dynamic maps, Pangolin UI, full LidarLoc state machine, or
  permanent SC/KISS background workers.
- Do not let current scan points grow the fixed map in fixed-map tracking.
- Do not replace project-specific iVox/NDT hot paths with a library migration
  unless profiling shows a local win and conversion costs are accounted for.
- Do not introduce broad namespace rewrites, unrelated formatting churn, or
  large full-framework copies while implementing a targeted feature.

## Required Patterns

- Keep dependencies at the package boundary in `CMakeLists.txt` and
  `package.xml`.
- Keep ROS2 callbacks short; copy or enqueue data under mutex, then run bounded
  processing.
- Gate external pose candidates through NDT before `ResetToMapPose()`.
- Keep Scan Context dormant in Good state and bounded in init/LOST state.
- Use Eigen/PCL aligned containers for custom point types, matching
  `include/hikari_loclite/common/point_def.h`.
- Preserve `PCL_NO_PRECOMPILE` while custom PCL point types are used.

## Review Checklist

- Does the change keep the package independent of full `lightning`?
- Does it preserve the fixed-map invariant?
- Are config defaults bounded for embedded CPU usage?
- Are failures represented as false/invalid results instead of crashes?
- Are map, pose, and frame IDs handled explicitly?
- Is the build command above still valid?
