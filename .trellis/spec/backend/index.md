# Hikari LocLite Development Guidelines

This repository is a single ROS2 C++ localization package, not a web backend. Treat
the `backend` Trellis layer as the rules for all runtime C++ code, build files,
launch/config files, and SLAM/localization modules.

Primary project source:

- `hikari_loclite_build_2026-06-10.md`
- `CMakeLists.txt`
- `package.xml`
- `include/hikari_loclite/`
- `src/`

## Pre-Development Checklist

- Keep `hikari_loclite` independent from the full `lightning` framework.
- Read [Build And Dependencies](./build-and-dependencies.md) before changing CMake,
  package manifests, or dependencies.
- Read [Localization Architecture](./localization-architecture.md) before changing
  Fast-LIO, iVox, NDT, Scan Context, or map handling.
- Read [Runtime And Relocalization](./runtime-and-relocalization.md) before changing
  node orchestration, state transitions, `/initialpose`, SC, or LOST behavior.
- Read [Directory Structure](./directory-structure.md) before adding files.
- Read [Error Handling](./error-handling.md), [Logging Guidelines](./logging-guidelines.md),
  and [Quality Guidelines](./quality-guidelines.md) before reporting completion.

## Guidelines Index

| Guide | When to Use |
|-------|-------------|
| [Directory Structure](./directory-structure.md) | File placement, module ownership, namespace boundaries |
| [Build And Dependencies](./build-and-dependencies.md) | `CMakeLists.txt`, `package.xml`, third-party code, deployment constraints |
| [Localization Architecture](./localization-architecture.md) | Fixed-map Fast-LIO, iVox, NDT, Scan Context, point cloud processing |
| [Runtime And Relocalization](./runtime-and-relocalization.md) | ROS2 node behavior, state machine, pose gating, init/LOST relocalization |
| [Error Handling](./error-handling.md) | Return-value conventions and validation failures |
| [Logging Guidelines](./logging-guidelines.md) | Runtime logs and log-rate constraints |
| [Quality Guidelines](./quality-guidelines.md) | Verification, forbidden patterns, review checks |

## Non-Goals

Do not add Trellis frontend rules for this repository unless a real UI package is
introduced. The current product deliberately excludes Pangolin UI and web frontend
work.
