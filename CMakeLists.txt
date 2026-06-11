cmake_minimum_required(VERSION 3.16)
project(hikari_loclite)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

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
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_core OpenMP::OpenMP_CXX)
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

ament_package()
