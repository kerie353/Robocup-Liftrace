# 投递模式四：冻结XY与垂直速度加速制动

2026-10-13。在本地当前水平补偿双模式/下降超时改动之上新增可选下降模式4，未更改这些改动、投递许可或H降落业务；修改前快照在工作区外 `C:/Users/lihaoEN/Desktop/ssh/tmp/drop_mode4_20261013/before`。

## 正常路径

水平补偿通过当前所选门控后，冻结的补偿FC XY不变，切换为垂直速度下降。最高向下速度0.60m/s，指令加速上限0.60m/s²；每周期使用当前有效高度及垂直速度计算剩余制动距离。根据 `sqrt(2*braking_mps2*剩余距离)` 限制向下速度，并预留当前采样周期的运动距离和终点切换容差。制动加速度是指令规划参数，不是飞控实际减速度保证。

进入当前释放高度窗口且原许可成立时，沿已有异步ServoAction投递，不额外等VZ变零或停稳。服务等待期间继续执行下降制动；到目标上方1.5cm以内（或已低于目标）时切回冻结XY/终点Z位置保持，终点保持锁定，后续高度回弹不重启速度下降。收到匹配成功ACK后沿原投后恢复流程上升。现有超时兜底仍按当前工作区业务执行，本次不更改其触发和释放条件。

模式1/2/3实现保持；默认 `drop_descent_mode: 1` 保持。模式4与H模式4各自独立。

## 入口配置

在实际选择的现场YAML或08 settings设置并重新生成：

```yaml
drop_descent_mode: 4
drop_descent_mode4_speed_mps: 0.60
drop_descent_acceleration_mps2: 0.60
drop_descent_braking_mps2: 0.60
drop_descent_hold_tolerance_m: 0.015
```

原 `drop_descent_speed_mps: 0.20` 仍仅用于模式2/3的Z位置轨迹。模式4不修改0.40m目标/0.35～0.45m释放高度，也不修改PX4参数。C++直接配置对应 `drop_system/descent/{velocity_speed_mps,acceleration_mps2,braking_mps2,hold_tolerance_m}`。

## 发送与保护

- 新源话题 `/navigation/drop_setpoint_velocity`，消息 `mavros_msgs/PositionTarget`，经既有帧适配器输出 `/mavros/setpoint_raw/local`。
- mask=2524：启用PX/PY、VZ、yaw；忽略PZ、VX/VY、加速度和yaw_rate。下降VZ为ENU负值，MAVROS执行ENU→NED，业务层不再翻转符号。Z字段仅作为TF锚点。
- 适配器要求重力对齐TF、有限XY/Z/yaw/VZ、VZ在[-1,0]、有效帧/时间与TF新鲜度。包含零VZ制动指令。
- 新通道共享已有时间戳仲裁，晚到的旧位置/H/模式3命令不能覆盖新命令；终点普通位置保持也经过同一仲裁。
- 无有效反馈/控制状态时暂停下降、使用原位置保持通道；超出最大周期则VZ归零，不追赶跳变，恢复后重新加速。撤销、边界保护、成功结束、重置沿原事务路径停用新通道。
- 正式和试飞bag主题包含新指令源及既有raw输出，控制器记录 `[DropDescent4]` 高度、VZ和终点保持状态。

## 已执行验证

- `test_drop_descent_modes.py`：10项Python检查通过，包括旧模式3、新mask、速度符号、零速度、坏参数/TF/掩码及通道时间仲裁。
- `test_drop_velocity_descent_config.py`：2项通过，实际生成正式/试飞×补偿模式1/2×下降模式1/2/3/4共16份配置；除下降mode值外，控制配置完全相同。
- 已有 `test_landing_vertical_setpoint.py`：5项通过。
- Python AST/YAML、`git diff --check` 及作用域差异检查。

生产C++提取回归中新增3项：速度加速/制动与终点锁定、长间隔/无效反馈暂停和重启、冻结XY及下降期间释放/终点位置通道。按用户要求未编译，未运行需要编译的C++用例，未启动ROS/Gazebo、未飞行或部署。0.60m/s为期望速度，实际速度受PX4速度/加速度限制及闭环跟踪影响；新策略的动态效果尚待验证。
