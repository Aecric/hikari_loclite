# Build And Dependencies

`hikari_loclite` is a standalone ROS2 package built with `ament_cmake`. The
package is intended to compile and deploy independently from the larger
`lightning` framework.

## Build Contract

- Use C++17.
- Default to Release builds when `CMAKE_BUILD_TYPE` is unset.
- Keep `add_compile_definitions(PCL_NO_PRECOMPILE)` because this project uses
  custom PCL point types in `include/hikari_loclite/common/point_def.h`.
- **Builds MUST run inside the `lightning-jazzy:dev` Docker container (ROS 2
  jazzy).** The host is ROS 2 humble and existing `build/`/`install/` outputs
  are jazzy container artifacts — building on the host both fails dependency
  resolution and contaminates the colcon caches.
- The workspace root contains an `AMENT_IGNORE`, so colcon does NOT scan
  `src/` from the root. Always pass `--base-paths src/hikari_loclite`
  explicitly.

```bash
docker run --rm -v /home/aecriclin/3d_slam_ws:/root/slam_ws -w /root/slam_ws lightning-jazzy:dev \
  bash -lc "source /opt/ros/jazzy/setup.bash && \
            colcon build --base-paths src/hikari_loclite --packages-select hikari_loclite \
            --cmake-args -DCMAKE_BUILD_TYPE=Release"
```

`livox_ros_driver2` is pre-installed in the image under `/opt/ros/jazzy`, so
the package builds standalone. When verifying a non-Release build type, use a
throwaway `--build-base`/`--install-base` (e.g. `build_debug`/`install_debug`)
and delete it afterwards so the main Release outputs stay clean.

Runtime entrypoint (lightning-compatible CLI, contract §21):

```bash
# Preferred: gflags-style CLI flags (hand-parsed, no gflags dependency).
# --map_path overrides the map directory from yaml (loads <map_path>/global.pcd).
ros2 run hikari_loclite run_loclite_online --config /path/to/loclite_livox.yaml --map_path /path/to/map_dir

# Fallback (kept only for loclite.launch.py compatibility):
ros2 run hikari_loclite run_loclite_online --ros-args -p config_path:=/path/to/loclite_livox.yaml
```

Map path resolution order: CLI `--map_path` > yaml `system.map_path` > yaml
`fixed_map.global_pcd`.

## Allowed First-Version Dependencies

Keep the first runtime small. The build document allows:

- `rclcpp`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`
- `std_msgs`
- `std_srvs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `pcl_conversions`
- `pcl_ros`
- `livox_ros_driver2`
- `visualization_msgs` — required by contract §21: `hikari_loc/loc_status`
  (`Marker`) and `hikari_loc/sc/candidates` (`MarkerArray`).
- `rosbag2_cpp` — **non-Release builds only**, for the `run_loclite_offline`
  evaluation node. CMake guards both `find_package(rosbag2_cpp)` and the
  target behind `if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")`; a Release build
  must contain neither the offline binary nor the rosbag2 link. (`package.xml`
  declares it unconditionally because package format 3 has no build-type
  conditionals — the CMake guard is the real gate.)
- `yaml-cpp`
- `PCL`
- `Eigen3`
- `OpenMP`

`package.xml` currently declares the ROS2 dependencies and `libcap2-bin` for
runtime capability setup. `CMakeLists.txt` currently finds Eigen3, PCL,
yaml-cpp, and OpenMP.

## Disallowed Dependencies

Do not add these to the package unless a later task explicitly changes scope:

- Pangolin (contract line 1234 asks for a non-Release Pangolin UI, but this
  conflicts with the package's embedded scope — unresolved, do not add without
  an explicit user decision)
- OpenCV highgui
- KISS-Matcher as a default runtime capability
- miao optimizer
- g2o

Do not link `lightning.libs`. Extract small, necessary algorithm files instead.

## Third-Party Code

- `thirdparty/nanoflann` is the local dependency for Scan Context ring-key
  search.
- `thirdparty/Sophus` is vendored into this package so `hikari_loclite` does not
  include headers from another ROS package.
- iVox lives in `include/hikari_loclite/ivox3d/`.
- NDT OMP and voxel covariance code live in `include/hikari_loclite/ndt/` and
  `src/ndt/`.
- Do not add include paths that point into sibling ROS packages; copy small
  header-only third-party dependencies into this package's `thirdparty/` tree.

## Deployment Notes

For CPU affinity or real-time scheduling, the build document uses:

```bash
sudo setcap cap_sys_nice+ep /home/aecriclin/3d_slam_ws/install/hikari_loclite/lib/hikari_loclite/run_loclite_online
```

Keep systemd, CPU affinity, watchdog, and log-rate work in productization tasks;
do not hide those assumptions inside algorithm code.
