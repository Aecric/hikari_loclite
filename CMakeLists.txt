cmake_minimum_required(VERSION 3.16)
project(hikari_loclite)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# KISS-Matcher 全局点云配准 (重定位粗 6DOF). 默认 ON; 关掉时整段 #ifdef 守卫降级,
# 不影响其余构建. KISS 源码 vendor 在 thirdparty/3rd/KISS-Matcher, 构建期 FetchContent
# 仅拉 ROBIN(v1.2.7 tarball, 内部已 pin PMC@4bbd40a). 详见本段下方 USE_KISS_MATCHER 接线.
option(USE_KISS_MATCHER "Build with KISS-Matcher global registration support" ON)
option(HIKARI_KISS_MATCHER_USE_SYSTEM_TBB "Use system TBB when building vendored KISS-Matcher" ON)
option(HIKARI_KISS_MATCHER_USE_SYSTEM_ROBIN "Use system ROBIN when building vendored KISS-Matcher" OFF)

if(UNIX AND NOT APPLE)
  set(CMAKE_INSTALL_RPATH "$ORIGIN")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

add_compile_definitions(PCL_NO_PRECOMPILE)

# --- Find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP)
# 异步 KISS 重定位 worker (std::async) + 工作线程降优先级 (pthread_setschedparam) 需显式链接 pthread
find_package(Threads REQUIRED)

# --- KISS-Matcher (vendored global registration) ---
# 模式照搬 lightning (src/lightning-lm/CMakeLists.txt:71-139): 优先系统包, 否则
# add_subdirectory 本包 vendored 源码. hikari 不链 lightning.libs, 自带独立一份 KISS.
if(USE_KISS_MATCHER)
  # 优先用系统已装的 kiss_matcher (一般没有, 留作 deb 部署/缓存场景的退路).
  # ---- robin::robin 缺失 bug 修复 (lightning 已踩并记录) ----
  # kiss_matcher 安装包的 Config.cmake 漏 find_dependency(robin), 而其 Targets.cmake 会
  # 先建出 kiss_matcher::kiss_matcher_core 这个引用 robin::robin 的 IMPORTED 半成品, 末尾
  # 校验 robin::robin 缺失则把 kiss_matcher_FOUND 置 FALSE; 增量/级联重编命中 cached
  # kiss_matcher_DIR 时, 半成品 target 已建 → 骗过下方 from-source 的 NOT TARGET 守卫 →
  # 两条路都不跑 → robin::robin 永久缺失. 修法: find_package(kiss_matcher) 前先找 robin.
  if(NOT TARGET robin::robin AND kiss_matcher_DIR)
    get_filename_component(_hikari_robin_dir "${kiss_matcher_DIR}/../robin" ABSOLUTE)
    find_package(robin QUIET PATHS "${_hikari_robin_dir}" NO_DEFAULT_PATH)
    if(NOT TARGET robin::robin)
      find_package(robin QUIET)
    endif()
    unset(_hikari_robin_dir)
  endif()
  find_package(kiss_matcher QUIET)

  # NOT TARGET 幂等守卫: colcon 两趟 cmake configure 会重复处理本段; KISS CMakeLists 的
  # kiss_matcher::kiss_matcher_core ALIAS 无 TARGET 守卫, 第二趟重复 add_subdirectory 会撞
  # duplicate ALIAS. 目标已定义则整段跳过.
  if(NOT kiss_matcher_FOUND AND NOT TARGET kiss_matcher::kiss_matcher_core)
    set(HIKARI_KISS_MATCHER_SOURCE_DIR
      "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/3rd/KISS-Matcher/cpp/kiss_matcher"
      CACHE PATH "Vendored KISS-Matcher core source directory")

    # 透传系统包选择给 KISS 子构建: TBB 用系统 (镜像已装), ROBIN 走 from-source
    # (KISS 子构建 FetchContent 拉 ROBIN v1.2.7 tarball, ROBIN 内部已 pin PMC@4bbd40a).
    set(USE_SYSTEM_TBB ${HIKARI_KISS_MATCHER_USE_SYSTEM_TBB}
      CACHE BOOL "Use system TBB for KISS-Matcher" FORCE)
    set(USE_SYSTEM_ROBIN ${HIKARI_KISS_MATCHER_USE_SYSTEM_ROBIN}
      CACHE BOOL "Use system ROBIN for KISS-Matcher" FORCE)

    # hikari_loclite_core 是 SHARED 库, 链接的 KISS (及其传递的 ROBIN/PMC) 都是 STATIC.
    # 静态库默认不带 -fPIC, 链进 .so 会触发 "relocation R_X86_64_PC32 ... recompile with -fPIC".
    # 在 add_subdirectory 前打开 PIC, 让 KISS 子树所有静态目标都用 -fPIC 编译.
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)

    message(STATUS "KISS-Matcher: building vendored source at ${HIKARI_KISS_MATCHER_SOURCE_DIR}")
    add_subdirectory(
      ${HIKARI_KISS_MATCHER_SOURCE_DIR}
      ${CMAKE_BINARY_DIR}/_deps/kiss_matcher-build)
  endif()

  add_compile_definitions(USE_KISS_MATCHER)
endif()

# --- Include directories ---
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/hikari_loclite
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  thirdparty/nanoflann
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
)

# --- Library ---
add_library(${PROJECT_NAME}_core SHARED
  src/common/nav_state.cc
  src/lio/eskf.cc
  src/lio/pointcloud_preprocess.cc
  src/lio/scan_context.cc
  src/lio/fast_lio_fixed_map.cpp
  src/ndt/ndt_omp.cpp
  src/ndt/voxel_grid_covariance_omp.cpp
  src/ndt/ndt_corrector.cpp
  src/reloc/reloc_manager.cpp
  src/reloc/kiss_global_matcher.cpp
  src/system/realtime_setup.cpp
  src/system/loclite_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  livox_ros_driver2
)

target_link_libraries(${PROJECT_NAME}_core
  yaml-cpp
  ${PCL_LIBRARIES}
  Threads::Threads
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_core OpenMP::OpenMP_CXX)
endif()

if(USE_KISS_MATCHER AND TARGET kiss_matcher::kiss_matcher_core)
  target_link_libraries(${PROJECT_NAME}_core kiss_matcher::kiss_matcher_core)
endif()

# --- Executable ---
add_executable(run_loclite_online
  src/app/run_loclite_online.cpp
)

target_link_libraries(run_loclite_online
  ${PROJECT_NAME}_core
)

ament_target_dependencies(run_loclite_online rclcpp)

# --- Offline 评估节点 (仅非 Release 编译) ---
# 契约文档 hikari_loclite_build_2026-06-10.md: "在 release 模式下 offline 节点不编译".
# 嵌入式 Release 部署不携带 rosbag2 依赖; 离线评估请用 RelWithDebInfo / Debug 构建.
if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
  find_package(rosbag2_cpp REQUIRED)

  add_executable(run_loclite_offline
    src/app/run_loclite_offline.cpp
  )

  target_link_libraries(run_loclite_offline
    ${PROJECT_NAME}_core
    yaml-cpp
  )

  ament_target_dependencies(run_loclite_offline
    rclcpp
    rosbag2_cpp
    sensor_msgs
    livox_ros_driver2
  )

  install(TARGETS run_loclite_offline
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# --- Install ---
install(TARGETS
  ${PROJECT_NAME}_core
  run_loclite_online
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# systemd unit 模板 (Phase6 产品化): .deb postinst 在安装时渲染 @ROS_DISTRO@ 并装到
# /lib/systemd/system, 不 enable 不 start. 开发 colcon 构建不自动接 systemd.
install(FILES docker2/hikari-loclite.service.in
  DESTINATION share/${PROJECT_NAME}/systemd
)

ament_package()
