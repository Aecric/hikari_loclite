# Cross-Layer Thinking Guide

Use this guide when a change crosses ROS2 callbacks, sensor buffers, Fast-LIO,
fixed-map structures, NDT validation, Scan Context, state machine, TF, odometry,
or YAML configuration.

## Map The Runtime Flow

For normal tracking:

```text
Livox/PointCloud2 + IMU
  -> message conversion and buffering
  -> deskewing
  -> Fast-LIO fixed-map matching
  -> tracking quality/state machine
  -> pose smoother
  -> odom / TF / loc_state
```

For initialization and LOST recovery:

```text
/initialpose or Scan Context candidate
  -> NDT validation
  -> ResetToMapPose()
  -> fixed-map tracking
  -> Good state and relocalization disarm
```

Before implementing, write down which boxes your change touches.

## Boundary Questions

| Boundary | Check |
|----------|-------|
| ROS2 message -> internal cloud | Are timestamp, intensity, ring, and frame assumptions preserved? |
| IMU + cloud sync -> deskew | Are time units and ordering explicit? |
| Deskewed cloud -> fixed map match | Is current scan excluded from the fixed map? |
| Candidate pose -> NDT | Is validation done before mutating Fast-LIO state? |
| NDT -> smoother | Are correction gain and max deltas bounded? |
| State machine -> RelocManager | Is SC armed only in init/LOST and disarmed in Good? |
| State -> publishers | Are map frame, lidar/base frame, odom, and TF consistent? |
| YAML -> runtime defaults | Are config names, units, and fallback values consistent? |

## Common Mistakes

- Dropping per-point time during conversion, breaking deskew.
- Applying `/initialpose` directly without NDT validation.
- Letting fixed-map mode accumulate current scan points.
- Running Scan Context continuously after tracking is Good.
- Updating a config default in code but not in YAML or launch documentation.
- Publishing pose jumps that should have been rejected by the smoother.
- Adding a dependency in CMake without updating `package.xml`.

## Checklist

Before implementation:

- [ ] Mapped the sensor-to-pose or candidate-to-reset flow.
- [ ] Identified all frame IDs, timestamp units, and coordinate frames involved.
- [ ] Decided which module owns validation.
- [ ] Confirmed CPU-bound work is bounded or rate-limited.

After implementation:

- [ ] Empty/missing sensor data is handled without crashing.
- [ ] Bad candidate poses leave state unchanged.
- [ ] Good/Degraded/Lost transitions are deterministic.
- [ ] Config, launch, CMake, and package manifests remain consistent.
- [ ] The fixed-map invariant still holds.
