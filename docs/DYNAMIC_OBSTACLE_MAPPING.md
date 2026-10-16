# Orange Pi 5 Plus 动态障碍地图（本地候选）

2026-10-16 方案批准，2026-10-17 完成本地实现与检查。用户明确不推送、不编译，只做逻辑检查与记录。本轮没有部署、启动 ROS、仿真或硬件；不能称已消除现场鬼影或满足实时指标。

## 范围与默认状态

- 保留 FAST-LIO、FreeDOM、现有 SDF/ESDF、轨迹服务器及任务权威；不增加神经网络、人体语义模型或第二套规划器。
- 修正当前实机入口 FreeDOM 的 TF 为点云原生 `body`；飞控/视觉的 `mapping_imu` 及安装外参保留。
- 新增 C++ 观测层、地面地图重建服务、任务准入和轨迹失效联动。
- **新观测层默认关闭**：`observation_map_enabled: false` / `observation_enabled:=false`。这是按“源码逻辑检查→另行编译→回放→板端地面验证”的阶段开关，不能把未编译代码自动切入下一次现场飞行。
- 无论开关是否开启，空地图刷新修复、FreeDOM 源时间戳/帧名检查和地面重建服务的拒绝保护均随源码交付；关闭时不允许使用协调重建服务。

## 数据流

```text
/cloud_registered_body + 同帧 camera_init→body
  ├─ FreeDOM → /freedom/static_pointcloud（算法原始静态候选/诊断）
  └─ 有界 C++ worker：全部有效命中 + 限量空闲射线 + 候选掩码
        → 不可变快照 → 现有 SDF/ESDF → 现有规划与碰撞检查
        → /navigation/observed_obstacles（当前占据，含临时障碍）
        → /navigation/observation_ready（源时间和代次凭据）
        → /navigation/observation_diagnostics（1 Hz 诊断）
```

FreeDOM 候选只影响稳定障碍的柱化资格，不能生成真实命中，也不能刷新 hit/free 时间。候选点持续发布不会复活已被射线清除的位置。

`/navigation/observed_obstacles` 不是“永久静态场地 PCD”：它同时包含近期行人和稳定障碍。FreeDOM 原图、FAST-LIO `/Laser_map`、当前占据及最终膨胀图含义不同。原 WorkBench 分支仍按其原协议导出最后非空 FreeDOM 原图，本轮没有修改其他分支；评估本功能必须看当前占据和最终规划图，不能只看原图导出。

## 观测规则

1. 所有有限、量程内的当前回波先标记占据，再进行清理；相同帧的重复点只记一次命中。
2. 清理只使用实际回波前方的射线，在端点前留 0.20 m 保护距离；当前帧端点附近一格邻域保护。遇到前景命中时停止该射线，避免用背景回波穿透前景障碍。更远的真实背景回波可证明本地量程内的自由空间，但不在裁剪端点制造假障碍。
3. 每格至少 3 个不同源帧的有效清理观测才成为 FREE；同帧千条射线不算千次证据。遮挡长于 free lease 后，未完成的清理计数重新累计。
4. 近期命中租期 1 s，空闲证据租期 2 s；失去观测转为 UNKNOWN，绝不因时间过去转为 FREE。
5. 候选掩码存在且真实命中连续支持约 2 s，才有禁越柱资格；单次命中、缺失/过期候选、临时障碍不柱化。停止的人可能最终被视为稳定障碍，这是几何方案的能力边界，其位置仍须避障。
6. 保留遮挡/未知状态。用 UNKNOWN 的三维前缀和，O(1) 检查每次规划查询位置的机体盒是否均有观测证据；有效盒还必须处于当前 SDF 实际重建范围内。未覆盖时占据查询返回阻塞，距离查询返回 0。
7. 机体盒默认 XY 半宽 0.275 m、上/下各 0.20 m，可按标定配置；不以“启动若干秒”或固定空白泡假定机体周边已空闲。

因此，在地面盲区、低纹理/稀疏回波或裁判持续遮挡下，可能一直无法 READY。不能为使程序启动而关闭覆盖保护；应在地面验证可观测的起飞/初始路线、标定盒和参数。该保守限制是本轮尚未验收的重点，不保证从一个起飞点就能确认全场。

## 算力与内存控制

- 沿用 0.10 m 体素和 LIO 已有三线程、大核绑定；不改 RKNN 推理链。
- 只有启用新功能时，mapping 入口的 FreeDOM 候选线程数默认 2，支持 `freedom_threads:=4` 对照；关闭时保留原 8。未实测哪组最快。
- 一个持久 worker，输入队列只留最新帧；工作上限 10 Hz，清理 5 Hz、1500 条射线，射线选择随帧轮换。命中记录不按射线抽样。TF 只在该 worker 等至多 10 ms，不采用 latest TF 替代源时间。
- 单帧原始输入限 100000 点/4 MiB，候选限 600000 点/16 MiB；仅支持小端、未组织 XYZ float32 点云。超限/格式错误整帧拒绝并等待新鲜证据，不能截断危险端点后仍报告 READY。
- 体素数上限 600000、射线数上限 4000；非法配置拒绝初始化。使用连续数组、活跃候选索引和不可变快照，复用原 SDF/ESDF。主回调只交接候选，几何观测在 worker；SDF/ESDF 和任务状态机仍在原单线程回调执行，未启用全局 AsyncSpinner。
- 原图转存、当前障碍发布、ROS 传输和 ESDF 仍有开销；600000 格限制不是进程 RSS 上限。实际默认 10×6×3.8 m、0.1 m 为 228000 格；Cell 元数据约 24 B/格，另有保护数组、前缀和、快照、临时点云。原方案“新增常驻目标 30 MB”仍须实测；大场地和高占据率可能超过目标，不宣称已达到。
- 诊断给出近 100 次应用样本的 worker、SDF 应用和源年龄 P95。目标 worker P95≤15 ms、危险回波进入碰撞检查端到端 P95≤200 ms 尚未验证；`map_age_p95_sec` 只统计到地图应用，不能替代轨迹碰撞检查端到端测量。

配置在 `patrol_uav_ws-patrol_planner/src/uav_mission/config/observation_map.yaml`。普通/高位/coverage 量程 6 m、走廊 5 m；没有扩大为旧版 30/40 m。关掉本功能无需改变飞行场地、速度、高度或 LIO 参数。

## 入口与地图准入

- 试飞 settings 可填写 `observation_map_enabled: true`；`run_trial.py` 和共享 `hardware_session.py` 会传给 mapping 入口。默认缺省为 false，严格验证布尔类型。
- 两套 mapping/localization launch、仿真静态入口以及旧 `competition_hardware.launch` 暴露 `observation_enabled`。直接调用 mapping 时用 `observation_enabled:=true`，加载相同参数文件。开关在进程启动读取，不能在线只改一个 ROS 参数来切换。
- 应先完成另行授权的构建/回放/板端地面验证，再启用；本轮没有执行这些操作。
- 地面 warmup 最少 2 s，要求当前机体盒覆盖且无占据；READY 只能使用真实源时间。空点云可以是有效的“当前没有占据”，但仍需同一批次的新鲜观测凭据。
- MissionManager 在启用时订阅当前占据；凭据与地图源时间必须一致。ROS1 可能重写 PointCloud2 `header.seq`，因此不使用该字段承载代次；代次放在单独 Header 凭据的 seq。跨话题短暂未配对只等待，不能启动任务。
- 超龄、时间回退、TF 缺失、重建失败均不刷新 READY。规划器 odom 同样必须同坐标系、源时间有效且在 0.35 s 内，未收到有效 odom 时不能应用快照/计算 ESDF。规划 FSM 在观测过期或代次改变时取消旧轨迹/恢复动作；任务层沿用原 ABORT/保持路径，不增加自动解锁、降落或其他硬件控制。

## 地面地图重建

协调服务为 `/navigation/rebuild_map`（std_srvs/Trigger）。调用只准备新地图，不起飞。

```bash
# 仅在已验证的软件、实际地面未解锁状态由操作员使用。
rosservice call /navigation/rebuild_map "{}"
```

执行顺序：

1. 要求收到新鲜的 FC connected、disarmed、ON_GROUND 状态。
2. 调用 MissionManager `/navigation/prepare_map_rebuild`；只允许 IDLE/COMPLETE/ABORTED 且没有活动事务，锁住任务启动。
3. 撤销 READY，设置协调重建标志；调用 FreeDOM `/freedom/reset_map`。FreeDOM 自己再次核对地面状态与协调标志；服务和积分共用回调队列，重建 owning map，不浅拷贝体素指针。
4. FreeDOM 与观测层分别推进源时间下界。清空旧候选、观测快照、SDF/ESDF 及恢复证据；规划 FSM 看到新代次后丢弃旧轨迹。
5. 只有新代次、重建后源时间、覆盖/warmup 均满足时恢复 READY；Manager 才解除启动锁。

任何一步失败都保持启动禁止；在地面检查后重新调用协调服务。不要直接调用 FreeDOM 内部 reset 服务。FC 检查是遥测检查，不会锁住外部 RC：操作员必须保持未解锁，服务过程中不手动起飞。ROS 服务调用只发生在已核对的地面流程中；未承诺远程服务故障下的调用完成时间。

## 验证范围

本轮结果及执行命令见 [离线检查报告](verification/dynamic_mapping_20261016/REPORT.md)。Python 测试验证任务准入/代次/源时间逻辑；C++ 源码只做阅读和静态结构检查。C++ 场景测试已登记但按用户要求未编译执行。

待验证：真实树箱/细杆保留、行人走停/遮挡、A→B→A 搬迁、未知覆盖可用性、长时间满视觉/记录负载、散热降频、内存及端到端制动余量。离线通过不代表板端实时或实飞验收。
