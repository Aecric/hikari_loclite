# Research: KISS-Matcher 全局点云配准库集成 (hikari_loclite ARM64 重定位)

- **Query**: 调研 KISS-Matcher 用于嵌入式 ARM64 ROS2 固定地图 LiDAR 重定位（粗 6DOF → yaw 微扫 → NDT 精修 → ResetToMapPose）
- **Scope**: mixed（主要 internal：本 workspace 已 vendor KISS-Matcher v1.0.2 全套源码 + lightning 里已有验证过的 wrapper/PoC/CMake 接线；external 受限：本 session 无 web 工具）
- **Date**: 2026-06-16

> **核心发现（决定全局基调）**：本 workspace **已经把 KISS-Matcher v1.0.2 完整源码 vendor 进了**
> `src/lightning-lm/thirdparty/3rd/KISS-Matcher/`，并且 lightning 里**已经有一份针对"固定地图 +
> 累积查询云 + inlier 闸"完全相同用例**的、跑过的封装 + CMake 接线 + 离线评估 PoC。下面所有结论都
> 有本仓库源码出处（file:line），不是泛泛的网络资料。hikari_loclite 引入 KISS 本质上是**把这套已验证
> 的模式平移过来**，而不是从零集成一个陌生库。

---

## 关键资产索引（本 workspace 现成可抄）

| 文件 | 作用 |
|---|---|
| `src/lightning-lm/thirdparty/3rd/KISS-Matcher/` | vendor 的 KISS-Matcher v1.0.2 全套源码（git tag `v1.0.2`，commit `e3440b6`，2026-05-10） |
| `.../KISS-Matcher/cpp/kiss_matcher/core/kiss_matcher/KISSMatcher.hpp` | 核心 C++ API：`KISSMatcherConfig` + `KISSMatcher` 类 |
| `.../KISS-Matcher/cpp/kiss_matcher/core/kiss_matcher/KISSMatcher.cpp` | `estimate()` 实现（voxelize→FasterPFH→ROBIN→GNC solve） |
| `.../KISS-Matcher/cpp/kiss_matcher/core/kiss_matcher/GncSolver.hpp:30-37` | `RegistrationSolution{ valid, translation(Vec3d), rotation(Mat3d) }` |
| `.../KISS-Matcher/cpp/kiss_matcher/CMakeLists.txt` | 核心库构建（`kiss_matcher_core` STATIC，C++17，依赖清单） |
| `.../KISS-Matcher/cpp/kiss_matcher/3rdparty/{find_dependencies,robin/robin,tbb/tbb,teaserpp/teaserpp}.cmake` | 依赖 FetchContent 接线（robin/tbb/teaser/pmc/eigen） |
| `src/lightning-lm/src/core/localization/kiss_matcher_wrapper.{h,cc}` | **已验证的固定图全局配准封装**（`Match`/`MatchGlobal` + inlier 闸 + 大图 voxel/crop） |
| `src/lightning-lm/src/app/poc_kiss_global.cc` | **离线评估 PoC**：累积 K 帧→MatchGlobal→对 GT 算成功率/p50-p90-max 耗时 |
| `src/lightning-lm/CMakeLists.txt:6-139` | KISS 的 system-package / FetchContent 双路构建接线（可直接套到 hikari） |
| `src/lightning-lm/thirdparty/3rd/KISS-Matcher/ros/package.xml` | ROS wrapper（**不要引**，深绑 gtsam/visualization_msgs/pcl_ros） |

---

## 1. 算法与 API

### 事实

**算法管线**（`KISSMatcher.cpp:48-135`，逐级有 file:line）：
```
estimate(src, tgt):
  1. VoxelgridSampling(src/tgt, voxel_size)            # 可关 (use_voxel_sampling_)
  2. FasterPFH.ComputeFeature → keypoints + descriptors # Faster-PFH 特征 (论文卖点)
  3. ROBINMatching.establishCorrespondences            # 互检 + ratio test + ROBIN 图论剪枝
  4. RobustRegistrationSolver.solve                    # GNC-TLS (默认) 或 QUATRO 鲁棒求解
  → RegistrationSolution{valid, rotation(R), translation(t)}
```
- 特征：**Faster-PFH**（`FasterPFH.hpp/.cpp`，是 KISS 论文相对 FPFH 的加速版）。
- 外点剔除：**ROBIN**（Graph-theoretic，`max_core`/`max_clique` 模式，`ROBINMatching.hpp`），不是普通 RANSAC。
- 求解器：**GNC-TLS**（Graduated Non-Convexity Truncated Least Squares，默认）；`use_quatro_=true` 时切 **QUATRO**（只估 yaw，放弃 roll/pitch，适合主要绕 yaw 旋转的场景 —— 见 `KISSMatcher.cpp:37-43`，`GncSolver.hpp:231-239`）。底层 GNC/COTE 复用 **TEASER++**。

**核心 C++ API 签名**（`KISSMatcher.hpp`）：
```cpp
namespace kiss_matcher {
struct KISSMatcherConfig { /* 见下方参数 */ };
class KISSMatcher {
  explicit KISSMatcher(const float& voxel_size);          // 简易构造
  explicit KISSMatcher(const KISSMatcherConfig& config);
  RegistrationSolution estimate(const std::vector<Eigen::Vector3f>& src,   // ← 主入口
                                const std::vector<Eigen::Vector3f>& tgt);
  // 也接受 Eigen::Matrix<double,3,Dynamic> 重载
  size_t getNumRotationInliers();    // GNC 旋转内点数  ← 置信闸
  size_t getNumFinalInliers();       // COTE 平移内点数 ← 置信闸（更严）
};
}
// 输出：
struct RegistrationSolution {       // GncSolver.hpp:30-37
  bool valid = false;               // ← solver 是否给出有效解
  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;         // 注意可能非严格正交，需 Quaternion 归一化（见下）
};
```
- **输入类型**：`std::vector<Eigen::Vector3f>`（最常用）或 `Eigen::Matrix<double,3,Dynamic>`。**不是 PCL 类型**——需自己把 `pcl::PointCloud` 转成 `vector<Vector3f>`（wrapper 里就是手抄循环，`kiss_matcher_wrapper.cc:78-98`）。
- **输出**：`R(3x3) + t(3x1)`，自己拼 4x4。**没有内置返回 4x4/SE3**。
- **是否需要初始猜测**：**不需要。这是全局配准（global registration），无任何旋转/平移先验**——这正是它相对 SC+NDT 的核心优势（`kiss_matcher_wrapper.h:5-9` 注释明确写："KISS-Matcher 不依赖任何旋转先验"）。
- **置信/inlier 可作验证闸**：是。`getNumRotationInliers()` + `getNumFinalInliers()` + `sol.valid`。官方 `run_kiss_matcher.cc:143-152` 用 `final_inliers < 5` 判失败；本仓库 wrapper 用更保守的 `min_rotation_inliers=50` / `min_final_inliers=30`（`kiss_matcher_wrapper.h:38-42`）。
- **`R` 非正交陷阱**：`sol.rotation` 是 raw `Matrix3d`，直接构 `SE3(SO3(R), t)` 会触发 Sophus 断言；本仓库走 `Eigen::Quaterniond q(R); q.normalize();` 归一化（`kiss_matcher_wrapper.cc:195-198`）。hikari 用 tf2/Eigen，同样要先正交化 R 再转 quaternion。

**`KISSMatcherConfig` 关键参数**（`KISSMatcher.hpp:32-115`）：
| 参数 | 默认 | 说明 |
|---|---|---|
| `voxel_size_` | 0.3 | 主体素，FPFH 内部下采样；其它半径都按它倍数推（normal=3×，fpfh=5×） |
| `use_voxel_sampling_` | true | 输入是否内部 voxelize |
| `use_quatro_` | false | true=只估 yaw（roll/pitch 不主导时用） |
| `use_ratio_test_` | true | map 级建议 true；scan 级关掉略快 |
| `num_max_corr_` | 5000 | 最大对应数 |
| `robin/solver_noise_bound_gain` | 1.0/0.75 | noise bound = voxel×gain，>1.0 会被 clamp 到 1.0（大图经验） |

### 对本项目的落地判断

- **可行 / 强契合**：无需初值的全局 6DOF 正是 PRD Q1 决策（"真 KISS 出粗 6DOF → yaw 微扫 → NDT 精修"）的前半段。PRD Q6 的"KISS 置信闸"直接用 `rotation_inliers`/`final_inliers`，库原生支持。
- **推荐做法**：照抄 `kiss_matcher_wrapper.{h,cc}` 的 `RunKiss` 内核（estimate + valid 闸 + 双 inlier 闸 + quaternion 正交化），改命名空间 `hikari::loclite`、改日志为 `RCLCPP_*`、去掉 glog。
- **注意 PRD 用户描述的"每 3° 旋转看分值"不是 KISS 算法**（PRD:23 已标注）——KISS 是特征对应 + GNC，无 yaw 暴力扫描。yaw 微扫是 KISS 之后你自己加的精化步骤，不是库行为。

---

## 2. 依赖与许可证

### 事实

**License = MIT**（`KISS-Matcher/LICENSE`，Copyright 2025 MIT-SPARK）。**对闭源嵌入式产品友好**（仅需保留版权声明，无 copyleft 传染）。

**C++ 标准**：C++17（`cpp/kiss_matcher/CMakeLists.txt:33-35`，`CMAKE_CXX_STANDARD 17` + `CMAKE_CXX_EXTENSIONS OFF`）。hikari 本就 C++17，匹配。

**核心库 (`kiss_matcher_core`) 直接链接的依赖**（`cpp/kiss_matcher/CMakeLists.txt:84-86`）：
```
Eigen3::Eigen  robin::robin  flann::flann_cpp  OpenMP  TBB::tbb  lz4
```
**传递依赖链（要警惕的体积/许可证来源）**：
- **ROBIN** (MIT-SPARK, v1.2.7, FetchContent `robin.cmake:28`) — MIT。**ROBIN 又拉 PMC + TEASER++ + tinyply + googletest**（`teaserpp.cmake:5-45`）：
  - **PMC**（jingnanshi/pmc, `GIT_TAG master` —— **未 pin，浮动**）：并行最大团求解，BSD-2 系。生成 `libpmc`（运行期需 `ldconfig`，见 examples/README:45）。
  - **TEASER++** (MIT-SPARK, v2.0) — MIT。GNC/COTE 鲁棒求解器。
  - **tinyply / googletest** — 仅 IO/测试用。
- **TBB / oneTBB** (Intel oneAPI, `tbb.cmake:31` v2022.0.0) — **Apache-2.0**（嵌入式友好）。
- **flann** — BSD。`lz4` — BSD。`Eigen` — MPL2（嵌入式可接受）。

许可证总览：MIT / BSD / Apache-2.0 / MPL2，**无 GPL/LGPL，对闭源 ARM64 产品全部友好**。

### 对本项目的落地判断

- **许可证：无风险**。全是 permissive，可闭源分发。
- **依赖体积是真痛点**：引一个 KISS = 同时引 **ROBIN + PMC + TEASER++ + TBB + flann + lz4**。这跟 hikari `CLAUDE.md` "Do NOT add: KISS-Matcher / miao / g2o" 的初衷（保持依赖最小）正面冲突——**需要按 [[pangolin-ui-scope-decision]] 先例由用户拍板重评禁令**（PRD:24 已点名）。
- **PMC `GIT_TAG master` 未 pin** 是供应链隐患：联网 FetchContent 时 master 漂移可能破坏可复现构建。**推荐 vendor 时连同 ROBIN/PMC/TEASER 一起锁版本进 thirdparty/**（见第 3 点）。
- **TBB 是额外运行期 .so**：嵌入式镜像需确保 `libtbb` 可用（jazzy 容器里大概率有；否则 FetchContent 自编 v2022.0.0）。

---

## 3. ARM64 / 嵌入式编译可行性

### 事实

**能否 vendor 进 colcon 包**：**能，且 lightning 已经这么干了**。`src/lightning-lm/CMakeLists.txt:71-139` 是一套完整的"system-package 优先 → 否则 FetchContent/vendored 源码 add_subdirectory"双路接线，关键点：
- vendored 源码就在 `thirdparty/3rd/KISS-Matcher`，`add_subdirectory(.../cpp/kiss_matcher ...)`（CMakeLists.txt:132-135）即可。
- `add_compile_definitions(USE_KISS_MATCHER)`（:138）做编译开关；wrapper 里用 `#ifdef USE_KISS_MATCHER` 守卫（`kiss_matcher_wrapper.cc:9-11`），**没编 KISS 时降级返回 nullopt**——这套"可选编译"模式 hikari 可直接复用。
- `UPDATE_DISCONNECTED TRUE` + pin `v1.0.2`（:124-126）避免每次 reconfigure 撞 GitHub 认证墙。
- 已踩并修复的坑（CMakeLists.txt:73-97 大段中文注释）：增量重编时 `kiss_matcherConfig.cmake` 漏 `find_dependency(robin)` 导致 `robin::robin` 缺失——修法是 find_package(kiss_matcher) 前先手动找 robin。**hikari vendor 时会遇到同样的坑，注释里有现成解法。**
- colcon 两趟 configure（rosidl）重复 add_subdirectory 撞 duplicate ALIAS——用 `if(NOT TARGET kiss_matcher::kiss_matcher_core)` 守卫（:106）。

**header-only？否**。`kiss_matcher_core` 是 **STATIC 库**（`cpp/kiss_matcher/CMakeLists.txt:76` `add_library(... STATIC ...)`，编 4 个 .cpp：ROBINMatching/FasterPFH/KISSMatcher/GncSolver）。必须编译，不能纯头文件用。

**交叉编译 / QEMU buildx 先例**：lightning 已有 `src/lightning-lm/build.sh` + `docker2/Dockerfile` 跑 QEMU + docker buildx 交叉编 arm64 .deb（见顶层 CLAUDE.md "ARM64 / Jetson .deb packaging"）。hikari 的构建走 `lightning-jazzy:dev` 容器（jazzy，[[docker-jazzy-build-procedure]]）。**KISS 已经在 lightning 的 arm64 构建链路里被编过**（USE_KISS_MATCHER 默认 ON）——这是 hikari ARM64 可行性的最强证据。

**二进制体积 / 内存**：核心库本身小（STATIC，4 个 .cpp）。**编译期内存是风险**——CMakeLists.txt:155-160 注释提到这套构建"OOM-constrained, fragile"，TBB+TEASER+PMC 一起编很吃内存（lightning build.sh 按内存推荐 `BUILD_JOBS` 默认仅 2）。运行期：TBB 是额外 .so 依赖。

### 对本项目的落地判断

- **可行，路径明确**：vendor `thirdparty/3rd/KISS-Matcher`（连同 ROBIN/PMC/TEASER pin 版本）进 hikari，照搬 lightning 的 add_subdirectory + `USE_KISS_MATCHER` 守卫 + robin find_package 修复 + NOT TARGET 幂等守卫。
- **强烈推荐离线 vendor 而非联网 FetchContent**：嵌入式构建 + GitHub 认证墙 + PMC master 浮动 → 把 KISS-Matcher / ROBIN / PMC / TEASER++ 全部 pin 版本下载进 thirdparty/，`add_subdirectory` 本地源码。lightning 的 `LIGHTNING_KISS_MATCHER_SOURCE_DIR` 指 vendored 目录就是这个思路。
- **最大编译风险 = OOM**：在 `lightning-jazzy:dev` 容器里控制 `BUILD_JOBS`，按内存调。Jetson 上若本机编更要限 job。
- **TBB 运行期依赖**：确认目标镜像有 `libtbb`，或静态链/FetchContent 自带。

---

## 4. 大图缩放（百万点 global.pcd）

### 事实

**本仓库已直面这个问题并给了护栏**（`kiss_matcher_wrapper.{h,cc}`）：
- **target 必须先预下采样**：`target_pre_voxel=0.2m`，加载 global.pcd 后一次性 VoxelGrid 降采样并缓存复用（`kiss_matcher_wrapper.cc:41-53`，懒加载 `LazyLoadGlobal`）。
- **两种 target 策略**：
  - `Match()`：围绕 query 位置 **crop 半径 `crop_radius_m=30m`** 取局部子图（:74-91）——室内推荐。
  - `MatchGlobal()`：**整张预下采样 global.pcd 不裁剪**（冷启动/无先验，:106-148）。
- **点数护栏**：`max_target_pts=800000`，预下采样后仍超此值则**直接跳过并告警**（`kiss_matcher_wrapper.cc:122-128`），注释明写"大图需分块（暂为后续 task）"——即**百万点全图直接喂 KISS 是已知会爆/超时的**，必须 voxel + crop 压到 ~80 万点以内。
- **PoC 评估框架**（`poc_kiss_global.cc`）：已写好离线测 **整图无先验 MatchGlobal 的成功率 + 耗时分布（p50/p90/max）**，默认 `pre_voxel=0.2 / kiss_voxel=0.3 / accum_k=10`（累积 ~3s≈10 关键帧合成稠密 source）。**这就是 PRD Q2 该跑的实验，工具现成。**

**典型耗时量级（external 受限，未在仓库找到硬数字）**：`KISSMatcher::print()` 会逐段打印 Voxelization/Extraction/Pruning/Matching/Solving 各段秒数（`KISSMatcher.cpp:185-211`）——**实测数字需在目标 bag + 目标 ARM64 上跑 PoC 得出，标"未确认"**。论文（arXiv:2409.15615）主打"map-level"百万点配准秒级，但 x86 多核数据不能直接外推 Jetson。

### 对本项目的落地判断

- **百万点全图直喂 = 不可行**（仓库护栏已确认）。**必须**：(a) target 预下采样到 0.2~0.3m leaf；(b) 优先 crop 局部子图（室内 20~30m，PRD Q2 倾向）；(c) 整图无先验仅在冷启动用，且控制在 ~80 万点以内。
- **耗时是头号未知数**：PRD 允许重定位高 CPU、一次性，但**单次是否秒级 vs 十秒级在 Jetson 上未确认**——**强烈建议先在 `lightning-jazzy:dev` 容器（或目标板）上用 `poc_kiss_global` 跑真 global.pcd + 真累积云，量出 p50/p90/max 再定 Q2**。这是引入与否的决定性数据。
- **推荐做法**：沿用现有"累积 20 帧 + 重力对齐"的 query 云（PRD Q2/现有 RelocManager 资产），target 走 crop 子图（poses.txt 关键帧位置或上次已知位姿做 crop 中心；冷启动无中心时退整图 MatchGlobal）。**poses.txt 因此值得保留**（PRD Q4 待定项 → 倾向保留，用于 crop 锚点）。

---

## 5. 退化 / 对称场景（走廊）已知局限

### 事实

- KISS 是 **feature-correspondence + 几何一致性（ROBIN 图论 + GNC）**。比 SC 的 ring-key 极坐标高度图判别力强得多——SC 在走廊沿轴 ring-key 近乎相同导致"收向原点"（[[sc-reloc-domain-mismatch]] / [[corridor-drift-frontend-architecture]]），KISS 用真实 3D 几何特征对应，**不会退化成 SC 那种伪命中**。本仓库 wrapper 注释（`kiss_matcher_wrapper.h:5-9`）正是冲着"对称场景下 SC yaw 估计不准"来的。
- **但走廊沿轴几何退化是结构性的，任何单帧/单点配准都治不了**（PRD:29 / Out-of-Scope 明确）：自相似走廊里多个沿轴平移位置的局部几何近乎全等，feature-correspondence 仍可能给出沿轴滑动的多解。
- **库提供的"低置信"识别手段**：
  - `RegistrationSolution.valid`（solver 无解直接 false）。
  - `getNumRotationInliers()` / `getNumFinalInliers()`——走廊退化时内点数会偏低/解不稳定，可设阈值拒绝（wrapper 用 50/30）。
  - `getFinalCorrespondences()` / `getInitialCorrespondences()`（`KISSMatcher.hpp:218-228`）可取最终/初始对应数算 overlap 比例做更细的闸。
  - **无内置 "overlap ratio" 单一标量**——需自己用 (final_inliers / matched pairs) 估算。
- `thr_linearity_`（FasterPFH，默认 1.0=不启用线性度过滤）可过滤线状/退化关键点（`FasterPFH.hpp:57 is_planar` / `KISSMatcher.hpp:41`），但默认关。

### 对本项目的落地判断

- **KISS 显著优于 SC，但非走廊根治**（PRD 决策 ADR 已认知："KISS 比 SC 判别力强但非根治"）。
- **必须保留 NDT 二次验证 + inlier 闸**（PRD 要求"配准结果仍过 NDT 验证再 ResetToMapPose"）：KISS 出粗解 → inlier 闸先拒明显低置信 → yaw 微扫 → NDT 精修 + NDT 自身的 confidence/delta 闸兜底。走廊里若 KISS 给沿轴滑动多解，靠 inlier 偏低 + NDT 残差被双重拦截。
- **推荐做法**：用 `final_inliers` 做主置信闸（保守阈值，宁可拒绝重定位也别接受错解），并在走廊场景 PoC 里专门统计 false-accept 率。走廊轴向多解的根治（运动累积/外部先验）已被 PRD 划为 Out-of-Scope，不在本任务。

---

## 6. 集成模式

### 事实

**最小 C++ 集成示例（非 ROS，本仓库现成）**：
- 官方裸示例：`.../KISS-Matcher/cpp/examples/src/run_kiss_matcher.cc`（PCL 读云 → `convertCloudToVec` → `KISSMatcherConfig(resolution)` → `matcher.estimate(src,tgt)` → 取 R/t 拼 4x4 → `final_inliers<5` 判失败）。
- **更贴本用例的封装**：`src/lightning-lm/src/core/localization/kiss_matcher_wrapper.cc` 的 `RunKiss`（:150-206）——固定图懒加载 + crop/voxel + valid 闸 + 双 inlier 闸 + quaternion 正交化 + 完整日志。**这就是 hikari 该抄的模板。**

**官方 ROS2 wrapper：有，但不要引**。`.../KISS-Matcher/ros/`（`README_ROS2_REGISTRATION.md`）。其 `package.xml` 深绑 `rclcpp_components / visualization_msgs / tf2_* / pcl_ros / gtsam`（`ros/package.xml`）——**与 hikari "Do NOT add: visualization_msgs / g2o" 直接冲突**。只取核心库 `cpp/kiss_matcher`，完全绕开 `ros/`。

**版本 / 仓库 / 活跃度**：
- 仓库：`https://github.com/MIT-SPARK/KISS-Matcher`（MIT-SPARK 实验室，KISS-ICP 同门）。
- vendored 版本：**v1.0.2**（git tag，commit `e3440b6`，提交日期 **2026-05-10**，PR #103）。注意 `cpp/kiss_matcher/CMakeLists.txt:2` 的 `project(... VERSION 0.3.0)` 是内部版本号未同步，**实际 release tag 是 v1.0.2**。
- lightning 已 pin `LIGHTNING_KISS_MATCHER_GIT_TAG "v1.0.2"`（CMakeLists.txt:11）。
- 活跃度：到 v1.0.2 已发到 PR #103，2026-05 仍在维护，ROS2 Humble badge，有 ICRA 2025 论文背书——**活跃、可靠**。

### 对本项目的落地判断

- **集成模式清晰**：(1) vendor `cpp/kiss_matcher`（+ROBIN/PMC/TEASER pin）进 hikari `thirdparty/`；(2) CMake 抄 lightning 的 add_subdirectory + USE_KISS_MATCHER 守卫 + robin 修复；(3) 抄 `kiss_matcher_wrapper` 的 RunKiss 内核，改 `hikari::loclite` 命名空间、`RCLCPP_*` 日志、去 glog/Sophus（hikari 用 Eigen/tf2，R 正交化用 `Eigen::Quaterniond`）；(4) 绝不碰 `ros/` 子目录。
- **不引 ROS wrapper** 与 hikari "只要核心库" 诉求一致。
- 版本就用 lightning 已验证的 **v1.0.2**，保持两包一致。

---

## 总结论

**值得引入：是（有条件）。** KISS-Matcher 是 MIT 许可、C++17、无需初值的全局 6DOF 配准，原生提供 inlier 置信闸，**正好补 SC 在走廊"收向原点"的结构性失效**，且与 PRD Q1 决策（KISS 粗解 → yaw 微扫 → NDT 精修）完全对齐。最强论据是：**本 workspace 已 vendor v1.0.2 全套源码，lightning 里已有一份针对"固定图 + 累积查询云 + inlier 闸"完全相同用例、跑过 ARM64 构建链路的封装/PoC/CMake 接线** —— hikari 引入 = 平移已验证模式，工程风险大幅低于从零集成。

**引入路径（推荐 vendor 方式）**：
1. 离线 vendor `cpp/kiss_matcher` + ROBIN(v1.2.7) + PMC(pin commit，勿用 master) + TEASER++(v2.0) 进 hikari `thirdparty/`（**不联网 FetchContent**，避 GitHub 认证墙 + PMC master 漂移）。
2. CMake 照搬 `src/lightning-lm/CMakeLists.txt:71-139`：`add_subdirectory` vendored 源码、`USE_KISS_MATCHER` 编译开关 + `#ifdef` 守卫、robin find_package 修复、NOT TARGET 幂等守卫。
3. 复用 `kiss_matcher_wrapper.cc:RunKiss` 内核（estimate + valid + 双 inlier 闸 + quaternion 正交化），改命名空间/日志/去 glog。
4. target 走"预下采样(0.2m) + crop 子图(20~30m)"，冷启动退整图 MatchGlobal 但限 ~80 万点；query 沿用现有 20 帧累积 + 重力对齐。保留 poses.txt 做 crop 锚点。
5. 绝不引 `ros/` wrapper（绑 gtsam/visualization_msgs）。

**最大风险**：
1. **依赖膨胀 vs hikari 最小依赖契约**：引 KISS = 连带 ROBIN+PMC+TEASER+TBB+flann+lz4，直接违反 `hikari_loclite/CLAUDE.md` "Do NOT add: KISS-Matcher / miao / g2o" 禁令。**需用户按 [[pangolin-ui-scope-decision]] 先例正式重评禁令**（PRD:24 已点名为前置条件）。
2. **Jetson 单次重定位耗时未确认**：百万点全图直喂必爆（仓库护栏已证），即使 voxel+crop 后，ARM64 上单次是秒级还是十秒级**没有实测数据**。这是引入与否的决定性未知数 —— **务必先用现成的 `poc_kiss_global` 在目标板/容器上量 p50/p90/max 再定**（PRD Q2/Q5 待定项）。
3. 次要：编译期 OOM（TBB+TEASER+PMC 同编很吃内存，控 BUILD_JOBS）；PMC master 未 pin 的供应链隐患（vendor 时锁版本即可消除）；走廊轴向退化 KISS 仍非根治（靠 inlier 闸 + NDT 双重拦截，根治超出本任务范围）。

## Caveats / Not Found（未确认项）

- **目标 ARM64（Jetson）上单次重定位实测耗时**：未确认。仓库无硬数字；需跑 `poc_kiss_global`。论文宣称 map-level 秒级但为 x86 多核，不能外推 Jetson。
- **目标 global.pcd 实际点数 / 预下采样后点数**：未确认（取决于具体地图），决定走 crop 还是整图。
- **PMC 浮动 master 的具体 commit**：vendored 树未含 PMC 源码（FetchContent 时才拉），未确认当前 master commit；vendor 时需主动 pin。
- **external 网络资料（第三方 ARM64/Jetson 编译踩坑帖、社区 issue）**：本 session 无 web 搜索工具，未检索；如需补充建议另起带 web 工具的调研。所有上述结论均来自本 workspace vendored 源码与 lightning 已验证集成，出处可核。
