# add hikari loclite packaging ci

## Goal

Add Debian packaging support for `hikari_loclite`, modeled after the existing
`lightning-lm` Docker/buildx packaging flow, so the package can be built locally
with a shell script and in GitHub Actions as `.deb` release artifacts.

## What I already know

* The reference implementation lives in sibling repo paths:
  `/home/aecriclin/3d_slam_ws/src/lightning-lm/docker2`,
  `/home/aecriclin/3d_slam_ws/src/lightning-lm/.github/workflows/docker-build.yml`,
  and `/home/aecriclin/3d_slam_ws/src/lightning-lm/build.sh`.
* `hikari_loclite` is a standalone ROS2 `ament_cmake` package with package name
  `hikari_loclite`.
* Runtime binary for deployment is `run_loclite_online`.
* Release builds do not build `run_loclite_offline`; this keeps rosbag2 out of
  the deployed binary set.
* `package.xml` already declares `livox_ros_driver2` and `libcap2-bin`.

## Requirements

* Add a local `build.sh` that uses Docker buildx to export generated `.deb`
  files to a local output directory.
* Add `docker2/Dockerfile` for Debian package generation via `bloom-generate
  rosdebian` and `fakeroot debian/rules binary`.
* Add helper files under `docker2/` as needed for dependency injection and
  maintainer scripts.
* Add `.github/workflows/docker-build.yml` that builds amd64 and arm64 packages
  and uploads them to a GitHub Release.
* Adapt all package names, binary paths, Release text, and generated maintainer
  scripts from `lightning` to `hikari_loclite`.
* Preserve buildx multi-platform behavior and local override variables such as
  `ROS_DISTRO`, `UBUNTU_CODENAME`, `TARGETARCH`, `BUILD_JOBS`, and Livox package
  selection.

## Acceptance Criteria

* [x] `build.sh` is executable and points at `docker2/Dockerfile`.
* [x] `docker2/Dockerfile` references `hikari_loclite` paths and package names,
      not `lightning`.
* [x] GitHub workflow can compute release tag/version, patch `package.xml`, run
      matrix builds, list generated `.deb` files, and upload release artifacts.
* [x] Shell/YAML/Dockerfile syntax is locally checked where practical.
* [x] Existing unrelated worktree changes are not reverted or included.

## Out of Scope

* Changing localization runtime behavior.
* Adding new runtime dependencies beyond packaging-time requirements.
* Publishing or pushing commits.

## Technical Notes

* Relevant project specs: `.trellis/spec/backend/build-and-dependencies.md`,
  `.trellis/spec/backend/directory-structure.md`, and
  `.trellis/spec/backend/quality-guidelines.md`.
* The Docker build needs to install `ros-${ROS_DISTRO}-livox-ros-driver2`
  before CMake configure can find `livox_ros_driver2`.
* Maintainer scripts should configure dynamic linker paths for `/opt/ros` and
  set `cap_sys_nice+ep` on
  `/opt/ros/<distro>/lib/hikari_loclite/run_loclite_online`.
