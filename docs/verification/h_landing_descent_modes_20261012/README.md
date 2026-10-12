# H 降落三种下降模式

本次取代前一版高位对准后默认忽略 XY 位置/速度、发送零水平加速度的下降策略。
`landing_alignment_mode: low/high` 只选择锁定 H 之前的视觉对准条件；
`landing_descent_mode: 1/2/3` 独立选择锁定之后的下降策略，默认 1。

| 模式 | 水平控制 | 高度控制 |
| --- | --- | --- |
| 1 | 原三轴位置控制，目标为冻结 H 的 XY | 原最终 Z 目标及通用三维位置领先限幅 |
| 2 | 固定发送冻结 H 的 XY，持续位置闭环纠偏 | 独立 Z 轨迹，绕过通用三维位置领先限幅 |
| 3 | 忽略 XY 位置，发送 VX=0、VY=0，飞控抑制水平运动 | 同模式 2 的独立 Z 位置轨迹 |

模式 2/3 默认轨迹下降速度 0.20 m/s，每次减去 speed×dt，到目标后停止减小；
这是 Z 设定值的变化率，不保证实测下降速度。起点取新鲜里程计高度，目标仍为 FC
离地参考 0.35 m。反馈或控制权无效时暂停轨迹，间隔超过 0.20 秒不补走失去的时间。
新 LAND、取消、失败清除 H 轨迹；请求 POSCTL 后保持原交接高度，不继续推进轨迹。
数学轨迹复用 `DropDescentProfile`，但 H 的参数与实例独立，未修改投递业务逻辑。

模式 3 的 raw `PositionTarget` 掩码为 **2531**：忽略 PX/PY、VZ、全部加速度、yaw_rate；
启用 VX/VY=0、Z 位置、yaw。MAVROS 完成 ENU→NED 转换；仍要求重力方向一致的 TF。
模式 3 会主动抑制水平速度，但不会返回冻结 H 的位置，仍可能因距 H 超过 5 cm 而无法交接。
现有低位 POSCTL 的高度范围、水平误差、速度、样本、时间及有效性检查均保持不变。

## 配置入口

试飞 03/04/08 或正式比赛场地 YAML：

```yaml
landing_alignment_mode: high  # low/high，保留原先两种对准条件
landing_descent_mode: 2       # 1/2/3
landing_descent_speed_mps: 0.20  # 只影响模式2/3
landing_handoff_mode: POSCTL
```

板端 `run_trial.py` 支持 `--landing-descent-mode 2 --landing-descent-speed-mps 0.20`。
两套生成器写入 `external_landing/descent/{mode,speed_mps,max_step_dt_sec}`。
开关必须重新生成配置才生效；低位默认仍为模式 1，投递使用自己的 `drop_descent_mode`。

## 验证与边界

- 12 项配置回归通过，含两套生成器×两种对准×三种下降组合，其他生成字段（包括投递）一致。
- H raw 接口 5 项、既有投递下降接口 6 项通过，共 23 项离线 Python 检查。
- 增加实际生产 H 命令块的 C++ 提取测试，覆盖三模式输出、独立轨迹、反馈暂停、断流、目标钳制与交接保持；维护原控制器测试的 stub/提取位置。
- 按用户要求不编译，C++ 及 ROS 集成回归未执行；未启动 ROS、仿真、实机，未部署或推送。
- 修改前快照：`snapshot/h-before-three-descent-modes-20261012`（`2b83d5b0`）。
