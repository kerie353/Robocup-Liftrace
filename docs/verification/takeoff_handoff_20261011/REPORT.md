# 起飞完成交接修复与领先量默认回退

2026-10-11；导航来源 `/home/xhj/liftrace-controlwork-worktrees/board-trial-20260928`，
分支 `liftrace_board_trial_20260928`，修复前 HEAD `b19010c381a720565a94d3f64b2e36c7733e69d9`。

## 确定问题及现场边界

`patrol_control::positionCallback` 在距原起飞终点小于 0.3 m 时切换到外部
`Run_point`，原逻辑未初始化起飞完成后的保持目标。无有效 planner 时复用
`last_mavros_point_cmd`，经过通用三维限幅后再次写回 last。这样可能永久保存
中间插值点，连带改变起飞 XY 保持点。

已导出 bag：`/home/xhj/liftrace-deliverables/takeoff_failure_20261011/extract.json`。
新轮 OFFBOARD 136.887 s，143.475 s 起飞完成；143.520 s 同时出现
`no planner command received` 和 `Target point adjusted to max distance limit`。
发布终点从 1.187780 m 变为 1.144407 m，差约 4.34 cm，XY 也有变化。
内存数值复核：当前 `(0.02,0.06,0.90)` 距终点约 0.295 m，满足完成容差；
转为 Run_point 后若通用前视 0.25 m，首拍 Z 约 1.14417 m，吻合截断机制。
这说明原始任务目标和发布后的限距指令不应共用 last 作为唯一保持权威。

该缺陷不等于全部现场高度异常。主代理补充新轮实际 local Z 峰值约 1.408 m、
设定约 1.162 m，持续正误差明显大于 4.34 cm；未证明本补丁能消除这部分超差。
起飞完成不包含停稳/垂直速度检查，此补丁不改变原完成条件。

## 自动启动等待，不能归因于用户未点启动

`run_trial.py:258–293` 的 supervisor 在 `auto_start_after_arm=true` 时自动调用
`/navigation/start_mission`。它要求 OFFBOARD/armed/state 新鲜，最近一秒至少
10 个位姿样本、最新样本年龄小于 0.3 s、距 `reference['takeoff_z']` 小于
0.15 m、最近一秒 XYZ 各轴峰峰值均小于 0.12 m；再连续满足一秒才调用服务。
控制 ready 与这个自动启动 gate 是两个条件。服务产生的 `manual start` 日志
名称不能说明是用户手动点击。

主代理已核对：新轮 ready 143.475 s，任务启动 162.229 s，约 18.75 s 等待；
期间 probe 为 IDLE。第二轮取消起飞设置后 ready 118.965 s、启动 121.608 s，
等待约 2.64 s。结合持续偏高，首轮应按自动高度/稳态 gate 延迟放行分析，
不是规划慢或用户未点启动。具体哪项逐时阻断仍由主代理结合完整 bag 和启动日志复核。

控制输入在实际板端 launch 被 remap 到 `/navigation/local_pose`，输出到
`/navigation/setpoint_mission`，再由 `navigation_frame_adapter` 按时间戳做
`camera_init ↔ map` 变换。完成回调使用相同任务参考保存终点，没有新增跨参考
直接拷贝错误；仅导出 raw MAVROS pose/setpoint 不足以定量排除 TF/LIO/飞控参考问题。
原始 MAVROS 高度不能直接当作任务坐标高度与 supervisor 门槛比较。

## 最小实现

- 仅在外部起飞完成回调、进入 Run_point 前，用原 `takeoff_point` 和首航点 yaw
  初始化 `patrol_cmd`、`mavros_point_cmd`、`last_mavros_point_cmd`。
- 设置既有 `external_waiting_for_motion_=true`。等待时每拍从 `patrol_cmd`
  重新生成指令，原三维限幅、yaw 限幅和软件限高继续生效，首拍发布值仍可能
  被限距，但不会取代原始保持终点。没有复用超限恢复 hold 标志，没有新增状态机。
- 等待期间忽略未授权旧 planner；原 SEARCH/APPROACH/RESUME/RETURN_HOME
  分支清除 waiting 和旧 planner 标志，随后新有效 planner 正常接管。
- `competition/control_base.yaml` 与 `08_full_mission/settings.yaml` 默认
  `takeoff_z_lead_m=0`。`competition_speed.yaml` 未定义/覆盖这个参数，实际
  limited/competition 两档均继承 settings 的 0，无需修改速度 overlay。
  生成器和最终 launch 保留显式 0.4～0.6 试验选项。全局前视、投递、H 与启动 gate 未改。

生产默认回退属于保守处置，不是证明 0.6 是全部偏高根因。

## 保护及验证

改动前旧包快照 `legacy_baseline/20261011/takeoff_handoff_hold/`：58 个源文件，
含 `MANIFEST.txt`、`SHA256SUMS`、`SOURCE_HEAD.txt`；不包含可再生字节码，不参与编译。

| 检查 | 结果 |
| --- | --- |
| 新连续交接回归，生产回调/选择分支/最终限幅提取编译与 UBSan | 5/5 PASS |
| 原起飞 XY、Z 领先数值与配置/实际 launch 回归 | 12/12 PASS |
| 正式实际 launch 参数回归 | 6/6 PASS |
| 原高度保持与恢复保持回归 | 7/7 PASS |
| 合计 | 30/30 PASS |
| 用新连续回归检查未修复 b19010c3 | 正确检出等待目标缺失 |
| 主审导航 0928 实际 catkin_make patrol_control | exit 0，成功链接 |

连续回归覆盖 lead=0/0.4/0.6、通用限距 0.2/0.25/0.4、Takeoff 完成首拍、
无 planner 连续后续帧、上冲后仍以原终点计算输出、原 yaw 保留、旧 planner
不得接管、四类运动命令解除等待、新有效 planner 接管、待任务 hold 优先级、
过期/过远 planner 回退以及旧模式。原高度保持夹具补齐 b19010c3 新增的 validator
和参数定义，断言不改；此前该夹具因缺少定义无法编译。

构建日志：`/home/xhj/liftrace-deliverables/takeoff_failure_20261011/build_nav0928.log`，
由主审执行并提供 exit 0，本代理核对成功链接日志，不重复构建。
测试使用既有 rl_drone/Noetic，独立字节码缓存路径；没有启动 ROS 节点、仿真或实机动作。

下一步由主审同步必要分支和确认现场部署；默认回退/交接保持需重新生成运行配置。
完整高度跟踪超差、TF/LIO/飞控参考以及第二轮投后转靶慢由主代理继续核对。
本提交不 push、不部署，不以离线测试作为现场问题全部消除的证明。
