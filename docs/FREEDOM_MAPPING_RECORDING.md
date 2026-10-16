# FreeDOM 自动建图记录：板端使用说明

2026-10-16 本地动态地图候选额外记录 `/navigation/observed_obstacles`、`/navigation/observation_ready`、`/navigation/observation_diagnostics` 及有效观测参数。原 FreeDOM 点云继续记录；WorkBench 分支的既有 PCD 导出未修改，不能将该导出当作观测层的当前占据。[接口与验证边界](DYNAMIC_OBSTACLE_MAPPING.md)。

2026-10-15 源码交付。目标设备为 Orange Pi 5 Plus（RK3588），运行环境为现有 ROS1 Noetic 系统 Python。未在板端部署或运行验收，本轮不执行编译。

## 配套版本

- 板端：[liftrace_board_trial_20260928 分支](https://github.com/sakelier/liftrace-controlwork/tree/liftrace_board_trial_20260928)。
- 工作台：[WorkBench 分支](https://github.com/sakelier/liftrace-controlwork/tree/WorkBench)。[工作台使用与下载说明](https://github.com/sakelier/liftrace-controlwork/blob/WorkBench/MAPPING_RECORDING.md)。

两端须配套升级。已有 Python 包 uav_mission 的新模块随源码交付；需使用指向本 checkout 的 ROS overlay，避免实际运行旧包。实际部署、环境变更和实机验收另行授权。本次提交推送不启动飞行、ROS 节点或硬件。

## 自动开启范围

以下入口在进入 flight 模式时自动启动独立记录进程：

| 入口 | 实现位置 |
| --- | --- |
| 专项试飞 | deployment/board_trials_4x4/common/uav_board_trials/scripts/run_trial.py |
| 正式流程 | patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/hardware_session.py |
| 低空观察 | deployment/low_hover_observation/flight.py |

preview、配置检查、localization-check 不启动新记录；通过其他未接入入口启动的程序不自动获得该功能。保持既有解锁、模式切换、控制发布和安全检查行为。观察入口无地图消息时可记 not_applicable；若收到外部地图，则检查输入和 TF 证据。

共享逻辑为 uav_mission.freedom_recording。它只订阅数据、读取参数、写本轮文件，不发布控制指令，也不调用 FreeDOM save_map。新记录导入、启动、运行、容量或封闭失败只输出 MAPPING_RECORDING_WARNING 并标记失败/不完整，不作为阻止飞行的新条件。既有飞行入口自身的检查仍可能按原规则拒绝运行。

## 依赖与配置

复用已有 rospy、rosbag，补齐 roslib、roslz4 的运行依赖声明。系统环境缺少依赖时只告警，不在飞行入口内自动安装。工作台使用其已有 rosbags 依赖在 PC 导出地图。

对应试飞 settings 或观察 profiles YAML 的顶层可配置：

```yaml
mapping_recording:
  input_topic: /cloud_registered_body
  buffer_bytes: 67108864
  max_message_bytes: 33554432
  split_bytes: 536870912
  min_free_bytes: 2147483648
  max_total_bytes: 8589934592
```

所有数值单位为字节，必须为正整数，单消息上限不得大于应用队列。默认 64 MiB 队列、32 MiB 单消息、约 512 MiB 分卷、2 GiB 剩余磁盘底线和约 8 GiB 每轮总量。空间与总量每秒检查，存在一个采样周期及写入块的余量。阈值到达只停本记录，不删除历史文件。

独立进程使用 nice=10，非阻塞持有 logs/.freedom_mapping_recording.lock，防止同一日志根目录下重复开启完整档。锁冲突只使新记录失败。队列过载/大消息丢弃和可见的点云序列缺口进入统计；原频订阅不保证任意负载下零丢包。

## 数据与文件

输入跟随 /freedom/pointcloud_topic，默认 /cloud_registered_body；话题变化后补订阅并标记可能的覆盖缺口。输出包括 /freedom/static_pointcloud、/sdf_map/occupancy、/sdf_map/occupancy_inflate。上下文包括 /tf、/tf_static、/Odometry、/laserMapping/realtime、MAVROS vision/local pose、local odom/state 和 /board_trials/run_metadata。保留原始 ROS 序列化数据、连接头及消息内时间戳，bag 记录时间取接收时 ROS 时间。

每轮写入 logs/<run>/mapping/：

| 文件 | 用途 |
| --- | --- |
| metadata.json | 独立 UUID、启动配置、运行上下文、容量限制 |
| owner.json / ready.json | PID 与启动时间身份、订阅就绪标记 |
| versions.json | 源码 HEAD 和记录模块指纹，HEAD 不代表源码一定已提交 |
| effective_parameters.json | 白名单实际 ROS 参数快照 |
| configs/ | 已生成的 runtime/control/overrides/profile 等配置与运行元信息 |
| manifest.json / summary.json | 实时/最终状态、封闭文件清单、计数和缺失原因 |
| freedom_*.bag.active | 尚未成功封闭索引的活动文件 |
| freedom_*.bag | 成功封闭并改名的 LZ4 分卷 |
| recorder.log | 记录进程输出与故障原因 |

参数白名单为 /freedom、/laserMapping、/common、/preprocess、/mapping、/publish、/lio_external_pose。配置与参数约每秒更新；metadata 的启动快照与最终有效值须结合查看。

停止入口会排空队列并封闭分卷，父进程最多等约 30 秒；未完成时子进程继续持锁收尾，不强杀写入进程。父进程消失由子进程的 PID/启动时间检查触发收尾。断电或 SIGKILL 不保证软件清理执行，遗留 .active 不自动转换成可下载 bag。

## 状态与取回

complete 表示记录自身的覆盖和落盘检查通过，不证明地图精度或飞行质量。缺少输入、静态 TF、静态地图，或丢弃、序列缺口、写入/封闭失败，会记 partial / failed；not_applicable 表示该轮未观测到需要检查的建图输出且没有记录故障。

工作台日志页按轮次读取清单：录制中只取回已封闭分卷，结束后可准备整轮 ZIP64。整轮包保存 bag、配置、元信息和下载报告，并在 PC 导出最后收到的非空静态地图的有限 XYZ PCD，附带 frame 和时间；它不保存 FreeDOM 的全部内部状态。

下载验证源文件身份、长度和 SHA256；失败保留 .part，不发布成品。新 mapping/ 目录排除在默认飞行日志 Wi-Fi 同步之外，需在独立建图区域手动下载。不自动清理板端记录。

## 验证与后续验收

[离线验证报告](verification/freedom_mapping_recording_20261015/REPORT.md)：板端 16 项，配套工作台 22 项新增、16 项同步和 6 项本机离线回归，共 60 项 Python 测试通过，另有 DOM、AST/XML、Node 与空白检查。

未编译，未执行 ROS1 原生 Bag/LZ4、真实话题联通、Orange Pi 满负载吞吐或实机飞行。后续审核和配套部署后，先在地面只读数据源下验证覆盖、分卷、退出、磁盘阈值、故障只告警及完整下载；飞行验收须单独授权，不自动解锁或起飞。
