# H 降落高低位模式开关（2026-10-12）

## 使用

试飞 03/04/08 的 `settings.yaml` 或正式比赛场地配置中设置：

```yaml
landing_alignment_mode: low  # 默认，改为 high 开启高位对准
landing_handoff_mode: POSCTL
```

板端 `run_trial.py` 也可用 `--landing-alignment-mode high` 覆盖，仅允许 H 降落模块。
开关经过两套生成器写入 `control.yaml` 的 `external_landing/alignment_mode`。
需要重新生成运行配置；已有 `logs/`、`deployment_results/` 内的生成结果不会自动更新。
正式运行需使用更新后的控制器和 `navigation_frame_adapter.py`，并启用适配器的
`enable_setpoints`（现有板端/正式控制入口已有此配置）。

## 行为

| 阶段 | low（默认，保留当前捕获） | high |
| --- | --- | --- |
| H 捕获 | 保留 `alignment_tolerance`（当前模板 0.08 m）及 10 个新鲜合法 H 帧 | XY ≤0.06 m、水平速度严格 <0.1 m/s，连续 ≥1 s |
| 数据 | 原视觉门控 | 位姿/里程计有效、同坐标系；图像与里程计源/接收时间 ≤0.2 s；断流、超差或无效反馈重计 |
| 锁定 | 冻结最后合格 H 中心 | 同左；原 10 帧数量门槛不叠加到高位模式 |
| 下降 | 冻结 H 的 XY 位置控制 | 不进行 XY 位置/速度反馈控制，只调整 Z 至目标高度，并保持航向 |
| 交接 | 低位检查通过，请求并确认 POSCTL，飞手完成下降 | 同左 |

高位锁定仍需有新的合法 H 帧，图像及里程计的时间均须覆盖 1 s；重复时间戳不能推进窗口。
高位模式只允许与 POSCTL 配合，避免改变旧 AUTO.LAND 路径。

两模式共用的当前 POSCTL 模板：FC 中心相对地面高度目标 0.35 m，允许范围
0.30–0.40 m；相对冻结 H 的 XY 误差 ≤0.05 m；水平速度 ≤0.1 m/s；
不检查竖直速度大小，但数据仍必须为有限有效值。位姿/里程计新鲜度及样本间隔
0.2 s；连续 ≥0.15 s 且至少 3 个不同源时间戳的里程计样本；H 已锁定，
飞控 connected/armed/OFFBOARD，沿用飞控状态有效性检查。
`max_vertical_speed_mps` 保留为兼容字段，H 低位交接不使用；共享 helper 的投递调用默认仍检查竖直速度。

高位允许 6 cm、交接要求 5 cm，并且下降期间不会纠正漂移，因此可能一直等不到交接；
失败/取消沿用原控制器位置保持处理，POSCTL 确认后仍由飞手完成落地，未增加自动落地或解锁指令。

## 垂直指令接口

控制器在高位锁定后只发布 `/navigation/landing_setpoint_vertical`（PoseStamped）：
Z 是目标高度，XY 仅用于坐标转换，不作为水平设定点。此时不发布原 XYZ 位置指令。
适配器按原消息时间查询 mission→MAVROS TF，保留时间戳，只接受重力方向一致的坐标系，
输出 `/mavros/setpoint_raw/local`（PositionTarget，type_mask=2363）。

掩码忽略 PX/PY、VX/VY/VZ、AFZ 和 yaw_rate，启用 Z、yaw 及零水平加速度。
MAVROS 完成 ENU→NED 转换。之所以仍提供 ax=ay=0，是因为 PX4 要求每个轴至少有
一个有效的位置、速度或加速度输入；将 XY 所有输入全部忽略会被拒绝。
零水平加速度不使用 XY 位置/速度误差闭环，不能抵御风或消除残余水平漂移。
适配器两类命令的转换/发布加锁并共用时间戳水位，拒绝晚到的旧类指令；反馈专用模式不创建控制发布者。
两种录包入口已加入垂直中间指令及 MAVROS raw 输入话题。

接口依据：PX4 v1.14.4 的
[MavlinkReceiver](https://github.com/PX4/PX4-Autopilot/blob/v1.14.4/src/modules/mavlink/mavlink_receiver.cpp)
逐轴解析 type_mask，以及
[PositionControl::_inputValid](https://github.com/PX4/PX4-Autopilot/blob/v1.14.4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp)
的逐轴有效性检查。未确认实机固件版本，也未将源码核对视为实机兼容验收。

## 本次检查

- 配置生成回归 10 项通过：低位默认、两生成器高位开关、参数透传、非法组合、非 H 隔离及历史配置快照。
- 无 ROS 的垂直接口检查 5 项通过：掩码、倾斜坐标拒绝、跨指令时间戳排序、拒绝后的水位、反馈专用模式。
- 补充生产 C++ 提取回归：高位连续窗口/严格速度/视觉过期/超差重置，低位放宽高度和水平速度、竖直速度免检；更新原 stub 依赖。
- 补充 ROS TF→raw 完整转换检查，供具备 ROS 环境时运行。
- 按用户要求未编译，因此 C++ 回归未执行；ROS 集成用例未执行。未启动 ROS、仿真或硬件，未部署、未实飞。
- 修改前快照：`snapshot/h-landing-before-high-align-20261012`，指向 `be527477`；当前工作分支保持不变。

后续需要编译并运行 C++/ROS 回归，再重新生成配置和部署；实机验证另行安排。
