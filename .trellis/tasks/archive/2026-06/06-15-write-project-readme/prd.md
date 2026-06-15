# Write Project README

## Goal

Create a root README for `hikari_loclite` that introduces the ROS 2 fixed-map
LiDAR localization package and gives developers/operators enough context to
build, run, configure, and deploy it.

## What I Already Know

* The package name is `hikari_loclite`, version `0.1.0`.
* It is a lightweight fixed-map LiDAR localization package for embedded
  deployment.
* Online runtime entrypoint: `run_loclite_online`.
* Offline rosbag evaluation entrypoint: `run_loclite_offline`, compiled only
  for non-Release builds.
* Primary config: `config/loclite_livox.yaml`.
* Launch file: `launch/loclite.launch.py`.
* Docker packaging flow: `build.sh` + `docker2/Dockerfile`.

## Requirements

* Add a root `README.md`.
* Introduce project purpose and high-level architecture.
* Document dependencies, build commands, online/offline run commands, key topics,
  services, config files, map inputs, and packaging flow.
* Keep the README accurate to the current source tree and avoid unsupported
  claims.
* Write bilingual Chinese and English versions in the same root README, with
  command blocks in shell syntax.

## Acceptance Criteria

* [x] `README.md` exists at the repository root.
* [x] The README describes what the package does and when to use it.
* [x] The README includes build and run examples for online and offline modes.
* [x] The README lists important inputs, outputs, service, and map/config paths.
* [x] The README mentions Release/non-Release behavior for the offline node.
* [x] The README includes packaging/deployment notes for `.deb` builds.

## Out of Scope

* No code, build, launch, or config behavior changes.
* No exhaustive explanation of every YAML key.
* No new tests required beyond documentation review.

## Technical Notes

* Inspected `package.xml`, `CMakeLists.txt`, `config/loclite_livox.yaml`,
  `launch/loclite.launch.py`, `src/app/run_loclite_online.cpp`,
  `src/app/run_loclite_offline.cpp`, `src/system/loclite_node.cpp`,
  `build.sh`, and `docker2/hikari-loclite.service.in`.
