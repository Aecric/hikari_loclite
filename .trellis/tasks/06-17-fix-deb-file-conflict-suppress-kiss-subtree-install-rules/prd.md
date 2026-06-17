# 修复 .deb 与 lightning 撞包：抑制 KISS 子树 install 规则

## Goal

让 `ros-humble-hikari-loclite` 的 .deb 能在已装 `ros-humble-lightning` 的机器上**干净 `dpkg -i`**，
不再因两包都 ship vendored KISS-Matcher 的开发产物而冲突。修法：在 hikari 顶层 CMake
里用 `CMAKE_SKIP_INSTALL_RULES` 包住 KISS `add_subdirectory`，让整棵 KISS 子树（KISS core +
构建期 FetchContent 的 ROBIN/PMC/tinyply/teaserpp）**完全不向 .deb 安装任何 dev 产物**。
KISS 对 hikari 是静态链入 `run_loclite_online`，运行时不需要其头文件 / `.a` / cmake 导出，
故抑制安装不改变运行行为。

## 背景 / 根因（已确认）

dpkg 报错：
```
trying to overwrite '/opt/ros/humble/include/kiss_matcher/FasterPFH.cpp',
which is also in package ros-humble-lightning 0.1.7-0jammy
```

- hikari 与 lightning 都用 `add_subdirectory` 引入各自 vendored 的 KISS-Matcher。
- KISS 上游 `thirdparty/3rd/KISS-Matcher/cpp/kiss_matcher/CMakeLists.txt`(100–120 行) 自带 install：
  - `install(DIRECTORY core/kiss_matcher)` → `/opt/ros/humble/include/kiss_matcher/*`（含 `*.cpp` 源文件，报错点）
  - `install(TARGETS kiss_matcher_core)` → `/opt/ros/humble/lib/libkiss_matcher_core.a`
  - `install(EXPORT)` + `install(FILES *Config.cmake)` → `/opt/ros/humble/lib/cmake/kiss_matcher/*`
- 且不止 KISS 本身：ROBIN(`3rdparty/robin/robin.cmake:33` `FetchContent_MakeAvailable`)、
  PMC/tinyply(`3rdparty/teaserpp/teaserpp.cmake:29-30` `add_subdirectory` 未带 `EXCLUDE_FROM_ALL`)
  的 install 规则同样泄漏进 .deb。`FasterPFH.cpp` 只是 dpkg 撞上的第一个文件，`--force-overwrite`
  后还会撞 ROBIN/PMC 一串。
- KISS 是 `STATIC` 库并静态链入可执行（hikari CMakeLists.txt:146）→ .deb 不该 ship 任何 KISS/ROBIN/PMC dev 产物。

实证（已验）：`set(CMAKE_SKIP_INSTALL_RULES TRUE)` 包住 `add_subdirectory` 后，子目录
（含其内部 FetchContent 子树）的 install 规则全部不生成；恢复 `FALSE` 后父目录自身 install 照常。

临时解封（用户已确认可跑）：`sudo dpkg -i --force-overwrite <deb>` —— 仅作临时验证，记账不干净。

## Requirements

### R1 — 抑制 KISS 子树 install 规则
- 在 `src/hikari_loclite/CMakeLists.txt` 第 86 行 `add_subdirectory(${HIKARI_KISS_MATCHER_SOURCE_DIR} ...)`
  外层包：`set(CMAKE_SKIP_INSTALL_RULES TRUE)` 前置 / `set(CMAKE_SKIP_INSTALL_RULES FALSE)` 后置。
- 加一行中文注释说明意图（KISS 静态链入可执行，不向 .deb 安装其 dev 产物，避免与 lightning 撞包）。
- 仅作用于 KISS 子树；hikari 自身的 `install(TARGETS run_loclite_*)` / `install(DIRECTORY config launch)` /
  `install(FILES ...service.in)`（CMakeLists.txt:182-200）必须不受影响。

### R2 — 不引入回归
- 现有 `USE_KISS_MATCHER` 开关、`NOT TARGET` 幂等守卫、robin `find_package` 修复、PIC 设置均保持原样。
- KISS 仍正常静态链入 `run_loclite_online`（功能不变，重定位链路不受影响）。

## Acceptance Criteria
- [ ] 容器 Release 编译通过（lightning-jazzy:dev，`--base-paths src/hikari_loclite`，`USE_KISS_MATCHER` ON）。
- [ ] 重新生成的 .deb 内**不含** `/opt/ros/humble/{include,lib}/` 下任何 `kiss_matcher*` / `robin*` / `pmc*` /
      `teaser*` / `tinyply*` 文件（用 `dpkg-deb -c <deb>` 核对）。
- [ ] 在已装 `ros-humble-lightning` 的机器上 `sudo dpkg -i <deb>`（**不带** `--force-overwrite`）干净成功。
- [ ] `run_loclite_online` 仍能正常起、KISS 重定位路径不回归（行为终验由用户在真机/bag 完成）。

## Definition of Done
- CMake 改动落地；容器 Release 构建绿；`dpkg-deb -c` 核对无 KISS/ROBIN/PMC dev 文件；
  干净 `dpkg -i` 成功；trellis-check 通过。

## Technical Approach

```cmake
    message(STATUS "KISS-Matcher: building vendored source at ${HIKARI_KISS_MATCHER_SOURCE_DIR}")
    # KISS 静态链入可执行, 不需向 .deb 安装其 dev 产物(头/.a/cmake export);
    # 否则与 lightning vendored 的同名 KISS 文件撞包(dpkg overwrite error). 连同
    # 其内部 FetchContent 的 ROBIN/PMC/tinyply/teaserpp 的 install 规则一并抑制.
    set(CMAKE_SKIP_INSTALL_RULES TRUE)
    add_subdirectory(
      ${HIKARI_KISS_MATCHER_SOURCE_DIR}
      ${CMAKE_BINARY_DIR}/_deps/kiss_matcher-build)
    set(CMAKE_SKIP_INSTALL_RULES FALSE)
```
