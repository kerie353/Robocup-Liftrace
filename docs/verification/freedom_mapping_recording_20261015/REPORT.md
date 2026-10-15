# FreeDOM 自动记录 · 2026-10-15

## 改动

共享模块 `uav_mission.freedom_recording` 接入专项 run_trial、正式 hardware_session、低空观察 flight 的真实 flight 分支，独立进程仅订阅并原样写入 ROS bag。启动/运行/容量/收尾失败只输出 MAPPING_RECORDING_WARNING；原控制发布、解锁、模式切换和安全检查保持原有行为。

新增 logs/<run>/mapping/，含 UUID、原始序列化消息、配置/运行元信息、白名单 ROS 参数、源码 HEAD/记录模块指纹、统计清单、封闭分卷。输入随 /freedom/pointcloud_topic，默认 /cloud_registered_body；输出为 FreeDOM static_pointcloud 与 SDF occupancy/inflate；同时记录 TF、Odometry、EV/FC pose、odom/state 与任务元信息。工作台配套流式取回并在 PC 导出 PCD，不在飞行中调用 save_map。

默认 LZ4，分卷约 512 MiB、应用队列 64 MiB、单消息 32 MiB、剩余空间底线 2 GiB、累计约 8 GiB 后停止新记录。顶层 mapping_recording 可以覆盖 input_topic 及上述 *_bytes 值。每秒采样空间，阈值允许一个检查周期与写入块的余量。独立 nice=10 进程和有界队列不替代板端满负载性能验收。

新增运行依赖 roslib、roslz4；沿用已有 rospy/rosbag。参数按当前 Noetic 环境读取，不安装依赖、不执行构建。板端使用说明见 [自动建图记录](../../FREEDOM_MAPPING_RECORDING.md)，配套工作台说明见 [WorkBench 分支](https://github.com/sakelier/liftrace-controlwork/blob/WorkBench/MAPPING_RECORDING.md)。

## 验证

`PYTHONDONTWRITEBYTECODE=1 python3 patrol_uav_ws-patrol_planner/src/uav_mission/test/test_freedom_recording.py -v`：16 项通过，输出见 board_tests.txt。覆盖写入/封闭失败、缺失输入、队列/磁盘/总量、原始消息与连接头、分卷、停止排空、异常放行、独立进程 flock 互斥与退出释放、不替换锁 inode、观察入口有/无地图。

配套工作台：22 项新回归、16 项飞行同步、6 项既有本机离线回归，以及前端 DOM / Node 语法均通过；两端共 60 项 Python 离线测试。详细输出在 WorkBench/docs/verification/mapping_recording_20261015/。Python AST、XML 解析和 diff 空白检查通过，无编译。

## 状态与限制

只有正常封闭且覆盖检查通过才为 complete；不保证任意负载下零丢包，也不表示地图或飞行质量合格。未运行 FreeDOM 的观察轮次可标 not_applicable；确实收到地图时仍检查完整证据。未封闭的 .bag.active 不提供下载；断电/SIGKILL 不承诺软件收尾。PCD 是最后收到的非空静态点云的有限 XYZ 点，frame/时间随附，不是 FreeDOM 内部状态归档。

本轮未编译、部署、连接设备、启动 ROS/Gazebo/PX4 或执行实机动作，2026-10-15 按用户授权提交并推送本板端分支，配套工作台提交到 WorkBench 分支；源码发布不等同板端部署或实机验收。板端 rosbag 原生写入、LZ4 及 Orange Pi 5 Plus 实际带宽/负载需单独授权验收。源码实施前原件在 logs/mapping_feature_20261015/before/，基线在同目录旁 index.json；不入库大日志/bag/缓存。

下一步：审核两端代码并配套升级，先地面核对真实话题覆盖、容量与停止封闭，再单独授权飞行验收。


## 同期远端更新合并

首次推送遇到并发远端更新，取回板端 32a15f48 和工作台 e12fca3 后合并。保留远端新增的默认 10×10 场地、地图高度、配置命名、文件选择及任务参数编辑功能；run_trial.py 和 server.py 自动合并，板端变更台账的追加段冲突保留双方完整记录。合并后重跑本功能的 60 项 Python 离线回归和前端检查，不编译、不部署、不改变远端新增业务。
