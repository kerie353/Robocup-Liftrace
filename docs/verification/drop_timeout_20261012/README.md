# 投递逻辑、释放高度与超时强制投递交接（2026-10-12）

本次基于 `liftrace_board_trial_20260928` / `bf096143`。正常投递条件和高度配置保持原值，新增外部任务的两种强制投递路径。修改先交用户审核；审核通过后才推送 `origin/liftrace_board_trial_20260928`。没有启动仿真、实机、舵机或部署。

2026-10-12补充：按用户最新要求，第一个锁定超时由30秒调整为20秒；低于60cm/15%/连续超过5秒及其余逻辑保持不变。

## 1. 当前投递任务如何释放舵机

以下指正式/板端 `external_mission_mode` 且 `compensated_alignment` 开启的当前链。旧独立固定航线仍保留历史行为，不能把其70次/50次计数当成正式链释放条件。

1. Mission Core选择已确认语义目标、保留载荷槽位，发APPROACH。Planner Bridge到达接近点后进入CAPTURE，取得同身份的新鲜地图目标，冻结事务地图点，发送ALIGN与AlignmentTargetContext。红十字使用drop_cross，其余有效比赛靶使用drop_circle。每次上下文续租仍属同一个任务，不重新选靶。
2. 视觉捕获：目标存在、已确认、中心精修、几何置信度至少0.6、观测不超过0.5秒、身份/关联上下文有效。冻结前图像中心距主点不超过30像素，累计不同图像的稳定帧；正式及试飞生成器将stable_frames设为5，底层文件的3是未覆盖默认。控制器需要drop_ready及有效补偿反馈，才冻结实际靶心和经机体姿态/槽位杆臂补偿后的FC目标。
3. 下降前门控：FC与真实投口相对目标的XY误差都≤0.04m，FC与投口水平速度的最大值≤0.10m/s，连续≥1秒；至少3个不同odom样本，相邻样本间隔≤0.20秒。位姿/odom须同坐标系、有效且年龄≤0.20秒。这个窗口不要求Z误差、竖直速度或窗口位移达标。误差/速度超限、陈旧反馈、无效飞行状态、身份变化会重计。
4. 下降：默认模式1沿用XYZ位置控制；可选模式2冻结XY并走独立Z轨迹，模式3给VX=VY=0并走独立Z轨迹。目标Z由release_setpoint_height给出。下降阶段单向锁定，不因视觉缺帧回升捕获高度。
5. 控制器正常释放：当前FC高度位于release_min_height～height_threshold的闭区间。release_height_only默认true，因此下降后不会再等水平误差、低速或连续稳定窗口；设false可恢复红十字XY<4cm、标准靶XY<6cm检查。仍检查新鲜odom、已冻结补偿目标、有效当前事务及飞行状态。
6. 任务许可：release_permission_arbiter还检查Aligning状态及其0.5秒新鲜度、飞机位姿0.5秒新鲜度、drop_circle/drop_cross模式、严格视觉身份或已有承诺、独立许可高度窗、槽位顺序和未重复投递。许可单条有效期0.25秒，控制器要求收到的许可年龄≤0.20秒，且mission/decision/attempt/slot/target/first_seen/class/mode完全匹配当前ALIGN。
7. 执行：控制器最后检查MAVROS新鲜、连接、armed、OFFBOARD与近墙中心边界，将许可epoch/revision、动作编号和身份固定进异步ServoAction。guarded_servo_proxy复核许可与动作互锁，调用/legacy/Servo_raw。服务传输成功不等于投递完成；只有匹配的终态成功ACK才置drop_complete、提交槽位。
8. 投后恢复：红十字/标准靶按各自恢复设定高度爬升；达到external_recovery_height后控制器进入Run_point并等待任务层接续。Bridge还要求投后新鲜反馈、Run状态、align_mode=disabled及原有恢复稳定/运动交接条件，再报告RECOVERY成功。任务层随后选下一目标；满足投递次数后继续投后路线/H任务。

主要代码：`patrol_control.cpp`的WayPointDetectDone、CrossDetectionDone、compensatedDropSettled、executeDropAction；视觉drop_aligner.py；任务release_permission_arbiter.py、guarded_servo_proxy.py、navigation_planner_bridge.py。当前外部链的旧十字30秒限制被alignmentWindowOpen跳过，既有Manager目标动作默认90秒期限只负责任务失败收尾，原先没有锁定超时强制投递。

## 2. 高度阈值在哪里，如何修改

高度是**飞控中心离地高度（FC AGL）**，不是相机或起落架离地高度。运行控制器比较任务坐标Z；生成器使用同一个地面标定ground_z换算：`local_z = ground_z + AGL`。

| 入口/参数 | 当前值 | 作用 |
| --- | --- | --- |
| 正式field.example及主要候选、01/02/05/06/08试飞配置 drop_agl | 0.40m | 下降指令终点 |
| 同上 release_min_agl / release_max_agl | 0.35 / 0.45m | 正常释放高度闭区间 |
| 对应任务预授权高度 | 0.27～0.47m | 许可层较宽窗口，不代替控制器最终窗口 |
| 历史field_20261007_validated.yaml drop_agl | 0.35m | 沿用未显式窗口时的0.35～0.45m；未修改此历史文件 |
| control_base.yaml / vcl06_horizontal_control.yaml未生成时 | 目标0.10m，上界0.20m | 本地Z底层默认，不是当前试飞的0.40m AGL |

推荐修改**实际选择的入口YAML**，例如08的`deployment/board_trials_4x4/08_full_mission/settings.yaml`或正式`--site-config`指向的文件。08使用competition速度档时，还须核对同目录`competition_speed.yaml`中的高度覆盖；只改settings可能被该档覆盖。

```yaml
drop_agl: 0.45
release_min_agl: 0.40
release_max_agl: 0.50
```

上面只是修改示例，本次没有替换现有高度。min/max必须成对配置且min≤drop_agl≤max；生成器会同时更新控制器下降目标、释放下/上界和许可高度窗。换算后的本地Z须满足原控制器正高度与恢复高度校验，否则配置应拒绝。

底层对应字段为`drop_system/release_setpoint_height`、`drop_system/release_min_height`、`drop_system/height_threshold`；许可层为`/release_permission_arbiter/min_release_altitude`和`max_release_altitude`。只改height_threshold只改了正常释放上界，既不改下降目标，也不改变许可层范围。不要只修改已经生成的一份control.yaml；下一次生成会覆盖。

离线检查（不会启动ROS）：

```bash
python3 patrol_uav_ws-patrol_planner/src/uav_mission/scripts/competition_supervisor.py preview \
  --root "$PWD" --site-config deployment/competition/field.example.yaml --check-config
python3 deployment/board_trials_4x4/common/uav_board_trials/scripts/run_trial.py full_mission preview \
  --root "$PWD" --check-config
```

生成实际部署文件时使用入口的`--output-dir/--fc-reference`或`--generate-config/--reference-fc`以及实测场地配置；部署与飞行仍需单独安排。

## 3. 新增超时语义

两条件是**或**关系，只限于有严格有效AlignmentTargetContext的外部投递。旧无上下文入口、搜索、APPROACH飞往目标、H降落不启用。

- **锁定20秒**：从许可节点首次接收Bridge冻结地图点的有效ALIGN上下文开始计时。到20.0秒还未投递完成，发forced_drop_lock_timeout许可。同一身份重复ALIGN或上下文续租不重置开始时间；新任务、撤销清空。这里“锁定”明确指开始末端ALIGN事务，不是搜索候选首次发现，也不是稍后通过5帧捕获的compensated_goal_frozen。既有Manager更早到期时仍优先终止任务，不延长其deadline。
- **低于60cm且高度变化率低于15%超过5秒**：按用户确认，`rate(t)=abs(h(t)-h(t-1s))/h(t-1s)`，h为FC AGL，使用位姿话题并减去生成器给出的同坐标地面ground_z。对不整齐采样在线性插值得到1秒前的高度。仅当前FC AGL严格<0.60m时累计；连续rate<0.15且时长严格>5.0秒，发forced_drop_height_stall许可；等于15%重置，等于5秒不触发。升高/降低均看变化量绝对值。
- **60cm是严格下界入口**：等于或高于0.60m时5秒计时清零；高位水平补偿悬停不触发5秒中断。下降到0.60m以下后，以低变化率首个有效样本开始5秒；回升至0.60m及以上后再次下降，必须重新累计。最近1秒高度历史可包含跨过60cm前的样本，计时本身绝不计入高于等于60cm的时段。首次没有1秒历史时先等采样齐备。20秒计时与60cm无关。
- 例如FC AGL0.50m时最近1秒变化0.05m，rate=10%，连续超过5秒会中断；变化0.075m恰好15%，不累计。
- 重复/乱序帧不累计；无效、过期、未来或低于等于5cm的高度、断流/间隙超过pose_timeout会清空5秒窗口，不把缺帧当作高度不动。ROS时钟回退锁住该事务的计时输出，直到新身份；ROS时间暂停不累计。
- 超时许可在正常视觉/捕获/水平到位/高度窗之前生效；控制器在当前XYZ保持位置、停止独立下降轨迹，以事务slot调用原异步ServoAction一次。超时已触发的原因保持到该事务撤销，避免两条超时互相重发。
- **仍保留**新鲜任务身份/上下文/位姿/odom、Aligning状态、armed/OFFBOARD、近墙中心边界、代理动作与槽位防重。撤销、任务期限到、手动接管、掉线、失效反馈不会被超时绕过。
- 收到匹配成功ACK后按现有恢复高度接续Bridge/Mission Core，下一投递使用下一槽位。调用已开始但未得到ACK、明确失败或结果未知时不会再调用，也不会伪报成功；由原任务deadline及隔离/返航逻辑收尾。20秒是请求时机，不是舵机硬件完成的保证；20秒前已发出的调用不会因超时再次执行。

参数在`patrol_uav_ws-patrol_planner/src/uav_mission/config/release_guard.yaml`：

```yaml
drop_timeout:
  enabled: true
  lock_timeout_sec: 20.0
  height_rate_threshold: 0.15
  stall_start_height_m: 0.60
  stall_timeout_sec: 5.0
  ground_z: 0.0  # 正式及九组试飞生成器会自动覆盖
```

回退方式：enabled改为false并重启许可节点即可恢复正常门控。新增功能不变更ROS消息/服务MD5；部署必须同步新Python模块、许可节点及重新编译的控制器，重新生成配置以传入ground_z。不能只覆盖控制器或只覆盖许可节点。

## 4. 验证与审核边界

新增超时边界、许可/执行代理/Bridge/Core流程回归与生产C++强制投递代码编译运行均已通过。离线生成正式+九组试飞×两个地面参考共20份配置，与基线比较只有新增超时ground_z，其余结构/数值相同。具体最终测试计数见BUILD.md。

环境为当前Linux系统Python、g++及现有YAML/numpy；没有安装包。当前机器无ROS/catkin/Eigen，无法完成patrol_control整节点Catkin构建、原完整几何C++回归或ROS消息集成。提取C++函数的UBSan编译运行使用位姿/传输替身，不能替代整节点构建、PX4实机或SITL验收。之前文档里的WSL编译报告属于历史机器/版本。

修改前控制包归档在工作区外`/home/ray/TOUDI/.drop_timeout_work_20261012/patrol_control_before.tar`，基准仍可由bf096143审计。无H/起飞/路线/当前释放高度运行逻辑改动。异步测试夹具补齐当前既有H模式4字段与禁用通道替身，用于运行原24项异步回归；没有改H业务代码。

用户审核重点：低于60cm的严格门槛与连续5秒是否符合预期、是否接受20秒超时在当前位置及非正常高度投递、无ACK继续沿原故障隔离而不虚报下一任务。用户审核前只保留本地提交，不推送。审核通过后推送远端同名分支；完整ROS构建及飞行效果属于明确尚未完成的验证范围。
