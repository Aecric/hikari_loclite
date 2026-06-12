# SC blackout clock fix + Phase 3 SC relocalization domain alignment

## Goal

让 LOST 态 SC 重定位真正可用。2026-06-12 bag 实测 (zt_5201, /tmp/loc_p2.log) 确认了两个阻断点：

- **A. blackout 时钟域 bug** — bag 回放下发过一次 `/initialpose` 后，SC 自动注入被永久 blackout，一次都没跑。
- **B. SC 查询域不匹配** — 查询用原始倾斜 (≈62°) body 系单帧；DB 是 lightning 用水平系点云 + 不同 radius/height 参数建的 → 空候选/错候选。

执行顺序：先 A（小改动，可单独提交），后 B。

## 实测事实（已核查，不要重新调研）

### A. blackout bug
- `src/system/loclite_node.cpp:432`：`blackout_deadline_sec_ = this->now().seconds() + external_pose_blackout_sec_` —— **墙钟**（实测 1781226305）。
- `src/system/loclite_node.cpp:949`：`if (!manual && ts < blackout_deadline_sec_)` —— `ts` 是 **scan/bag 时间戳**（实测 1779240xxx，早墙钟约 23 天）。
- bag 回放（无 sim time）下 `ts < deadline` 永真 → 日志反复 `[SC-Reloc] in blackout window`，SC 永不尝试。真机上 scan ts≈墙钟所以不显形。
- 同函数内 cooldown 检查用 ts vs ts（自洽，不要动）；`wall_now = this->now().seconds()` 用于 runtime 限制（墙钟 vs 墙钟，自洽）。

### B. SC 查询域
- `src/reloc/reloc_manager.cpp:205`：`sc_manager_.QueryTopK(scan, sc_top_k_)` —— scan 为原始 body 系**单帧**，未重力对齐、未累积。
- lightning 参照实现：`src/lightning-lm/src/core/localization/localization.cpp:1647 TryScanContextRelocalization` —— 查询前对累积点云做重力对齐（去 yaw：`R_body_level = R_yaw_only.transpose() * R_world_body`，`pcl::transformPointCloud`），用 RollingScanBuffer 累积。lightning 在同一张图上能定位。
- DB 建库参数（lightning `default_livox.yaml` scan_context 段）：`pc_max_radius=15.0, lidar_height=1.0, pc_num_ring=20, pc_num_sector=60, sc_dist_thres=0.18, sc_top_k=5`。**DB 文件存 ring/sector 但不存 radius/height** → 查询侧必须靠配置对齐。
- hikari `src/reloc/reloc_manager.cpp:54-58` **已经读** `reloc.sc_pc_num_ring / sc_pc_num_sector / sc_dist_thres / sc_pc_max_radius / sc_lidar_height`，但 `config/loclite_livox.yaml` 一个都没写 → 落在错误默认值 80.0 / 2.0 / 0.13。参数对齐是**纯配置修复**。
- 用户指定：SC 累积帧数 **20**。
- `current_imu_pose`（含姿态）已传入 `RunScanContextOnce`，重力对齐所需的旋转现成。
- 现有 init-accum 管线（`loclite_node.cpp:565-700`，`init_accum_*` 成员）已实现 deskewed scan 累积+体素降采样，模式可参考；debug publisher `hikari_loc/sc/accum_cloud` 已存在（`loclite_node.cpp:267`）。

## Requirements

### A. blackout 时钟域修复（先做，允许单独 commit）

1. `loclite_node.cpp:949` 的比较改为墙钟对墙钟：用 `this->now().seconds()` 与 `blackout_deadline_sec_` 比较（deadline 本来就是墙钟）。
2. 审计 `TryScRelocalize` + `/initialpose` 回调内剩余的 ts/墙钟混用：blackout 相关逻辑必须单一时钟域；cooldown（ts vs ts）与 runtime 限制（墙钟 vs 墙钟）已自洽，保持不动。
3. blackout 跳过日志同时打印 now 与 deadline 两个值，便于现场判断时域。

### B. SC 查询域对齐

1. **配置对齐**（`config/loclite_livox.yaml` reloc 段新增，键名按 reloc_manager.cpp:54-58 已有读取逻辑）：
   ```yaml
   sc_pc_max_radius: 15.0   # 必须与建库参数一致 (lightning scan_context); DB 不存该参数
   sc_lidar_height: 1.0     # 同上
   sc_dist_thres: 0.18      # lightning 实测可用阈值 (代码默认 0.13 过严)
   sc_top_k: 5              # 原 1; 多候选 + 重力检查 + NDT 验证逐个过滤
   ```
2. **重力对齐**：QueryTopK 之前把查询点云变换到水平 (level) 系，公式与 lightning localization.cpp:1647 一致：从 LIO 最新姿态 `R_world_body` 提取 yaw，`R_body_level = R_yaw_only^T * R_world_body`，对点云做该旋转。实现放在 RelocManager（`RunScanContextOnce` 已有 `current_imu_pose`）。
3. **20 帧滚动累积**：
   - 新 yaml 键 `reloc.sc_accum_frames`，默认 20。
   - armed（init/LOST）期间维护一个小环形缓冲：每帧存体素降采样后的 deskewed scan + 对应 LIO 位姿（LIO 全局漂了但短时局部自洽，可用相对位姿拼接）。
   - SC 尝试时把缓冲内各帧用相对位姿统一到**最新帧 body 系**，合并、降采样，再重力对齐、查询。
   - 帧数不足 `sc_accum_frames` 时跳过本次尝试（日志说明 frames=N/20），等下个 cooldown 周期。
   - Good/disarm 时清空缓冲。缓冲只在 armed 时维护，每帧开销仅为降采样+入队。
4. 有订阅者时把合并后的查询云发到现有 `hikari_loc/sc/accum_cloud`（level 系，frame_id 用现有约定）。

### 约束

- 不新增依赖、不链 lightning.libs；RollingScanBuffer 思路可最小化重写，**不得**抽取 lightning 的 `localization.*` 文件。
- SC 保持有界：无持久 worker；累积/对齐/查询只在 SC 尝试节奏（cooldown 5s）发生，不逐帧跑。
- 嵌入式预算：缓冲存降采样副本（leaf 可复用 `init_accum_voxel_leaf` 或新键），20 帧合计点数控制在 ~几万点级。

### 验收

- 容器 Release 构建通过：`docker exec lightning-jazzy bash -lc 'source /opt/ros/jazzy/setup.bash; cd /root/slam_ws; colcon build --base-paths src/hikari_loclite --packages-select hikari_loclite --cmake-args -DCMAKE_BUILD_TYPE=Release'`。
- bag 冒烟（AI 自测）：LOST 后日志必须出现 SC 实际尝试（无永久 blackout），QueryTopK 收到累积+水平系点云，有候选或干净拒绝日志。
- 行为终验由用户完成。

## C. 冒烟回归修复（2026-06-12 第一轮 bag 冒烟后追加）

第一轮冒烟 (/tmp/loc_p3.log) 证实 A/B 主链路生效：冷启动 SC 累积 20 帧（16068 点）查到正确候选
kf_id=1（sc_dist=0.142 < 0.18，pos 与真值差 ~0.3m），NDT 收敛 TP=3.09。但暴露 3 个本任务范围内的缺陷：

### C1. SC 候选验证 delta 门限照搬 /initialpose 量纲，过紧

- 实测：正确候选被拒，`dr=10.55deg > ndt.max_delta_rot_deg=10.0`（TP=3.09 极好，dt=0.299m）。
- SC sector 分辨率本身 360/60=6°，yaw_diff 量化误差 + 斜装近似意味着正确候选 delta_rot 常落 6~12°；
  关键帧间距也使 delta_trans 可超 1m（同轮 /initialpose FC 第一次也被 delta=1.16m>1m 拒过）。
- 修复：新增 `reloc.sc_max_delta_trans_m`（默认 2.0）、`reloc.sc_max_delta_rot_deg`（默认 15.0），
  **仅用于 SC 候选验证**（NdtCorrector::Validate 增加可选门限重载/参数），/initialpose 验证保持 ndt.* 原门限。

### C2. 帧数不足的 skip 消耗了 cooldown，LOST 5 秒窗口内抢不到一次真查询

- 实测时间线：Arm@974.14 清空缓冲 → 首次尝试 974.2 报 `frames=1/20` skip 但已写 `last_sc_attempt_ts_`
  → 下次尝试要等 cooldown 5s（ts 域）→ 977.5 恰好被 lost_timeout 切 WAIT + Disarm 抢先，SC 一次真查询都没跑。
- 修复：`frames < sc_accum_frames` 的 skip **不更新** `last_sc_attempt_ts_`（cooldown 只在真正执行
  QueryTopK 后起算）。这样 Arm 后约 2s 缓冲攒满即查，落在 5s LOST 窗口内。

### C3. LIO 发散后相对位姿拼接污染累积云

- 实测：LOST 后 LIO 速度积分跑飞（eff=0、update_accepted=0、每帧位移 ~5m），此时按相对位姿拼接的
  累积云完全糊掉，"LIO 短时局部自洽"假设失效。
- 修复：`AccumulateScan` 增加发散守门——与上一入队帧的相对平移超过
  `reloc.sc_accum_max_rel_trans_m`（默认 1.0，yaml 新键）时**不入队**（节流日志说明丢帧原因）。
  连续超限说明 LIO 已不可用于拼接，缓冲保持旧帧直到淘汰（环形 pop 自然换血），不额外清空。

### C 部分验收

- 容器 Release 构建通过。
- bag 冒烟复跑：冷启动 SC 候选应通过 C1 新门限完成 ResetToMapPose（日志出现 SC accept / reset）；
  LOST 5 秒窗口内应出现至少一次真 SC 查询（`SC query: accum_frames=...`）。
- 已知遗留（**不在本任务修**，留给用户决策）：lost_timeout=5s 后 Disarm+WAIT 即放弃自主恢复（WAIT 态
  SC 分支要求 Armed），以及 LIO 发散本身（ESKF eff=0 纯预测速度无界）——属状态机策略/Phase 2 续题。
