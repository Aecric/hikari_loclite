---
description: "Build hikari_loclite inside the lightning-jazzy:dev Docker container via colcon. Defaults to Release. Pass 'debug' for Debug build or a custom base-path."
---

# Docker Container Build

Build hikari_loclite inside the `lightning-jazzy:dev` container. The host is ROS 2 humble; all builds must run in the jazzy container (see `.trellis/spec/backend/build-and-dependencies.md` §Build Contract).

## Parameters

- `$ARGUMENTS` — optional, space-separated:
  - `debug` → `CMAKE_BUILD_TYPE=Debug` (uses throwaway `build_debug`/`install_debug` bases)
  - `release` or omitted → `CMAKE_BUILD_TYPE=Release`
  - `clean` → remove build/install dirs before building
  - Any other token is treated as a custom `--base-paths` value (default: `src/hikari_loclite`)

## Steps

### 1. Determine build type and paths

Parse `$ARGUMENTS`:

```bash
BUILD_TYPE="Release"
BUILD_BASE="build"
INSTALL_BASE="install"
BASE_PATH="src/hikari_loclite"
CLEAN=0

for arg in $ARGUMENTS; do
  case "$arg" in
    debug)    BUILD_TYPE="Debug"; BUILD_BASE="build_debug"; INSTALL_BASE="install_debug" ;;
    release)  BUILD_TYPE="Release"; BUILD_BASE="build"; INSTALL_BASE="install" ;;
    clean)    CLEAN=1 ;;
    *)        BASE_PATH="$arg" ;;
  esac
done
```

### 2. Clean if requested

```bash
if [ "$CLEAN" -eq 1 ]; then
  rm -rf /home/aecriclin/3d_slam_ws/${BUILD_BASE} /home/aecriclin/3d_slam_ws/${INSTALL_BASE}
  echo "Cleaned ${BUILD_BASE}/ and ${INSTALL_BASE}/"
fi
```

### 3. Run colcon build in container

```bash
cd /home/aecriclin/3d_slam_ws && \
docker run --rm \
  -v /home/aecriclin/3d_slam_ws:/root/slam_ws \
  -w /root/slam_ws \
  lightning-jazzy:dev \
  bash -lc "source /opt/ros/jazzy/setup.bash && \
            colcon build \
              --base-paths ${BASE_PATH} \
              --packages-select hikari_loclite \
              --build-base ${BUILD_BASE} \
              --install-base ${INSTALL_BASE} \
              --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}" \
  2>&1 | tail -30
```

### 4. Report result

Check exit code and show the last few lines of build output. If Debug build with throwaway bases, remind the user:

> Debug build used `${BUILD_BASE}/` and `${INSTALL_BASE}/` (throwaway). Main Release outputs in `build/` and `install/` are untouched.

## Notes

- The workspace root has `AMENT_IGNORE`, so colcon does NOT scan `src/` from root — `--base-paths` is required.
- `livox_ros_driver2` is pre-installed in the image under `/opt/ros/jazzy`.
- For .deb packaging, use `./build.sh` instead — that's a different workflow.
