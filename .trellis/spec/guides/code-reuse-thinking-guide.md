# Code Reuse Thinking Guide

Use this guide before adding helpers, constants, point types, filters, map
structures, or conversions. In this project, duplicated localization logic often
causes inconsistent frame handling, timestamp handling, or validation thresholds.

## Search First

Use `rg` before creating new code:

```bash
rg "topic_or_config_key|threshold|function_name|point_type" .
```

Search likely owners:

- `include/hikari_loclite/common/` for point types, Eigen aliases, constants,
  IMU/odom/nav-state data structures.
- `include/hikari_loclite/lio/` and `src/lio/` for preprocessing, deskewing,
  ESKF, and Scan Context behavior.
- `include/hikari_loclite/ivox3d/` for nearest-neighbor and voxel-map behavior.
- `include/hikari_loclite/ndt/` and `src/ndt/` for NDT and voxel covariance.
- `CMakeLists.txt`, `package.xml`, config, and launch files before changing
  dependencies, topics, frame IDs, or install paths.

## Reuse Rules

- Prefer extending existing point conversion and custom point definitions in
  `common/point_def.h` over adding one-off point structs.
- Prefer adding an option to the existing config model over hardcoding a second
  copy of a threshold.
- Prefer adapting current iVox/NDT/SC wrappers over introducing another
  nearest-neighbor or registration stack.
- If copying from `lightning-lm`, copy the smallest algorithm slice and document
  why the full framework dependency is still avoided.

## When To Abstract

Abstract when the same logic appears in multiple runtime paths:

- PointCloud2/Livox conversion preserves the same fields.
- Candidate pose validation is used by `/initialpose` and SC relocalization.
- Frame and timestamp handling repeats across publishers or callbacks.
- Config parsing repeats for the same YAML group.

Do not abstract a one-off hot-path loop if the abstraction adds allocation,
virtual dispatch, or unclear ownership.

## Checklist

- [ ] Searched for existing equivalent code.
- [ ] Constants and config keys have a single owner.
- [ ] New helpers live in the module that owns the concept.
- [ ] No new dependency duplicates existing iVox, NDT, PCL, Eigen, or nanoflann
      roles without profiling evidence.
- [ ] Copied full-framework code was trimmed to the minimum needed boundary.
