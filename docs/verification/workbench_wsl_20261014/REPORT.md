# Windows工作台与WSL SITL联调（2026-10-14）

范围仅新增 `deployment/workbench_sim` 独立适配，不改硬件入口及飞行业务逻辑。Windows工作台位于 F:/UAV/飞行工作台，使用独立配置、个人状态目录和8781端口；WSL为 U20.04-UAV，源码副本 `/home/lh/liftrace-workbench-sim-20261014`。

源码clone基线1e37bfca，并复制本轮开始时用户未提交的trial_config.py；原仓库后续并行修改不自动同步。原WSL工程与用户未提交文件保留。

视觉工作区构建通过。导航工作区第一次因重复测试注册失败，关闭测试又触发既有无条件gtest注册错误；仅在WSL副本删掉一条重复测试注册，开启测试目标后实际构建通过。未改原仓库CMakeLists。

用户20261011候选五类.pt已加载，在隔离Python3.10环境进行CPU YOLO推理；现场RKNN权重不用于SITL。复用既有PX4 SITL、MID360仿真插件和模型资源，不修改共享PX4源码或固件。

验证通过：Windows页面、SSH、真实配置展开、参数读取/覆盖、mock模拟解锁起飞、SEARCH航线、位姿及任务回传、虚拟相机图像、YOLO诊断、日志索引与HTTP下载。最终运行20261014_200150有相机与检测消息，YOLO诊断backend=ultralytics、device=cpu、message=ok；阶段SEARCH，观测零碰撞。主动停止，未完成整场验收。

最终bag约76MiB，bag_recording.json为PASS/normal_shutdown/exit0。停止后ROS/Gazebo/PX4/RViz/录制残留检查通过。完整日志在WSL副本logs/board_full_mission_wsl_*，失败启动记录保留。

仿真差异：固定4×4场景；航点编辑不重建墙/门/目标。默认vh/POSCTL转换为non_vh/AUTO.LAND并明确记录请求/实际值，OFFBOARD候选保留；本轮不替代实机H交接、RKNN实时性能和机械实投。后续可独立执行完整任务验收与场景重建。

## 独立正赛仿真追加（2026-10-14）

按用户要求新增第二张“独立正赛仿真”卡片，复用真实独立正赛的YAML选项、地图导入/编辑、投递/H参数与各开关。仿真使用原competition_config.generate以及原CompetitionManager/HardwareFullRuntime，单独shim建立SITL接线；硬件入口的拒绝仿真及真实投递约束未修改。

默认field_10x10_default.yaml为几何/飞行参数权威：高位2.6m、速度1.2m/s、投递FC AGL0.3m、5个走廊点、H(8.3,-3.5)。场景按flight_bounds、路线与墙面参数构建边界、分隔墙和两道0.80m通口；四组树箱和室内点云背景沿用已有世界并旋转到+X向前坐标。seed11布置bridge/panzer/pillbox/tent/red_cross五类靶标。

靶标真值仅存在scene.world/scene.json，不进入导航配置或识别控制。原场地site_confirmed=false保持不变，仅生成器内存副本被标记可生成；请求与实际actuator/H策略明确记录。模拟投递只含许可、服务和ACK，不包含物体下落、落点、RKNN性能或硬件PWM。

通过2项WSL离线集成检查、工作台命令/边界及浏览器命令一致性检查、ROS launch完整参数展开（退出0）。实际Windows HTTP验证：两张卡片、正赛配置摘要、原参数读取、地图草稿＋飞行参数＋开关覆盖组合，配置进程退出0、drop_agl=0.35、motion/resume/columns三个显式关闭均生效；实际生成5靶/4树/2通口。真实投递请求返回400。未自动启动新正赛飞行，无ROS/Gazebo/PX4/录包进程残留；完整任务动态验收待后续授权运行。

物理布局是假设场景：树箱不是现场测量结果，通口宽度沿用0.80m，墙面位置/开口根据路线推导；不能把场景与航点一致等同于真实赛场完整复刻。Python/配置适配未改C++，无需重新编译。
