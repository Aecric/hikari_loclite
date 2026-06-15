# Journal - aecric (Part 1)

> AI development session journal
> Started: 2026-06-10

---



## Session 1: 对外接口同步至契约第21节 (lightning 可替换)

**Date**: 2026-06-11
**Task**: 对外接口同步至契约第21节 (lightning 可替换)
**Branch**: `master`

### Summary

在 P1-P4 包骨架上将 hikari_loclite 对外接口与 lightning 定位模式对齐: 8 个 hikari_loc/* 话题(path/loc_state/ndt_status/loc_status Marker/sc 四件套)、map→base_link 与 lidar_frame→level_frame 两条 TF、/initialpose 指令语义(NDT 验证+限次重试+blackout 记录+契约日志关键字)、hikari_loc/sc_reloc 服务、--config/--map_path CLI、run_loclite_offline 离线节点(仅非 Release)。构建验证迁移到 lightning-jazzy:dev 容器(root 有 AMENT_IGNORE 须 --base-paths), Release/RelWithDebInfo 双构建通过; spec build-and-dependencies.md 同步更新。遗留: NdtCorrector 仍是 stub(Validate 恒过/Align 恒无效), Pangolin UI 与包约束冲突待决策。

### Main Changes

(Add details)

### Git Commits

| Hash | Message |
|------|---------|
| `7808274` | (see git log) |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 2: Phase5 SC relocalization implementation

**Date**: 2026-06-11
**Task**: Phase5 SC relocalization implementation
**Branch**: `master`

### Summary

Implemented RelocManager module and wired Scan Context into hikari_loclite for bounded init/LOST relocalization. SC database + poses.txt loading, NDT validation gate, /initialpose blackout, manual sc_reloc service, debug topics. Quality check found and fixed 8 issues (hardcoded cooldown, unenforced max_runtime, missing disable_after_good check, stale TODOs, missing pose output in LOST, marker positions).

### Main Changes

(Add details)

### Git Commits

| Hash | Message |
|------|---------|
| `35b6dd8` | (see git log) |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 3: NDT Phase 2 退化检测实现

**Date**: 2026-06-11
**Task**: NDT Phase 2 退化检测实现
**Branch**: `master`

### Summary

实现 NDT 退化检测: 新建 degeneracy_check.hpp (Hessian 特征值分析), 扩展 NdtResult/NdtCorrector (Align 后退化判定, Validate 退化拒绝), loclite_node MaybeNdtCorrectGood 退化跳过, 3 个 yaml knobs. Docker Release 构建通过.

### Main Changes

(Add details)

### Git Commits

| Hash | Message |
|------|---------|
| `5358cb0` | (see git log) |
| `4ce6aea` | (see git log) |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 4: Phase6 产品化 + ZUPT 标定收尾

**Date**: 2026-06-15
**Task**: Phase6 产品化 + ZUPT 标定收尾
**Branch**: `master`

### Summary

Phase6 产品化运行时优先三项+薄 systemd unit: CPU亲和/SCHED_FIFO(realtime_setup, 无cap优雅降级)、输入掉线watchdog+/hikari_loc/status富状态话题、LOG_EVERY_T时间式日志限频、hikari-loclite.service.in(@ROS_DISTRO@占位避开Dockerfile sed, postinst渲染不enable)。trellis-check 0 issue, 容器Release编译通过。ZUPT 静止检测改 Schmitt 死区双阈值+2026-06-15现场标定回填。混在工作树的 config 调优按任务拆 3 提交: zupt/sc-reloc(auto_on_lost等)/mid360-init。spec 更新: @ROS_DISTRO@占位约定 + LOG_EVERY_T。未做: 完整.deb export构建+真机实测。

### Main Changes

(Add details)

### Git Commits

| Hash | Message |
|------|---------|
| `f09a7de` | (see git log) |
| `d0d69fc` | (see git log) |
| `4ac7ad9` | (see git log) |
| `1d930e0` | (see git log) |

### Testing

- [OK] (Add test results)

### Status

[OK] **Completed**

### Next Steps

- None - task complete
