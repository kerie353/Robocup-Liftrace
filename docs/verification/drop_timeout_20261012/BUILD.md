# 验证记录（2026-10-12）

基准`bf096143`，分支`liftrace_board_trial_20260928`。最终实现包含用户补充的**FC AGL严格低于60cm才开始5秒计时**。

## 已通过

| 回归 | 用例数 |
| --- | ---: |
| 超时策略、60cm边界/回升重计、许可/代理/Bridge/Core下一目标、配置生成 | 20 |
| 原投递执行事务与迟到/撤销/不确定回执 | 61 |
| 原投递承诺门控 | 5 |
| 原Mission Core调度与槽位事务 | 49 |
| 原高度配置与参数校验 | 5 |
| 生产C++强制投递路径及RPC准入的提取编译运行 | 8 |
| 原异步worker/控制器提交、回执、互锁、不中断指令发布 | 24 |
| **总计** | **172** |

没有跳过用例。C++使用`g++ -std=c++14 -O1 -fsanitize=undefined -fno-sanitize-recover=all`，异步检查额外使用pthread。都是生产函数提取执行，ROS位姿/时间/服务边界使用替身，不会驱动真实设备。异步1秒mock等待期间实际指令发布最大间隔约0.0501秒，raw调用1次。

另行执行正式入口+九组试飞×两个静止FC参考，共20份真实配置生成。与bf096143基线生成输出逐字段比较，只有新增`/release_permission_arbiter/drop_timeout/ground_z`，高度、H模式、航点、速度、控制和任务参数均保持原值。Python AST和`git diff --check`通过。

原始日志与对照脚本位于工作区外`/home/ray/TOUDI/.drop_timeout_work_20261012/`，包括`test_drop_timeout.py.log`、`test_forced_drop.py.log`、`test_async_servo.py.log`以及其余测试日志。未将可再生大日志加入仓库。

复跑示例（仓库根，ROS包的离线检查，不启动ROS节点）：

```bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/patrol_uav_ws-patrol_planner/src/uav_mission/src:$PWD/vision_ws/src/uav_high_view/src${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -s patrol_uav_ws-patrol_planner/src/uav_mission/test -p test_drop_timeout.py
python3 -m unittest discover -s patrol_uav_ws-patrol_planner/src/patrol_control/test -p test_forced_drop.py
python3 -m unittest discover -s patrol_uav_ws-patrol_planner/src/patrol_control/test -p test_async_servo.py
```

## 未完成的验证

当前Linux有g++、系统Python及既有YAML/numpy，未安装额外包；没有ROS/catkin/Eigen。
因此**patrol_control整节点Catkin构建、原完整补偿几何C++回归和ROS消息集成尚未验证**。提取C++函数的UBSan编译运行不能替代这些检查。之前WSL构建报告属于历史机器/版本。

没有启动SITL/实机、舵机、ROS master、部署或推送远端。本地候选交用户审核，部署前需要在完整ROS环境补构建和集成验证。

## 2026-10-12补充：第一个中断条件调整为20秒

按用户最新要求，锁定超时的策略默认、许可节点缺省参数和release_guard.yaml统一由30秒改为20秒；其余条件未改变。20秒边界/续租、高度门控、许可/代理/Bridge/Core下一目标及配置生成20项，原执行事务61项，共81项相关离线回归通过；YAML逐字段检查仅lock_timeout_sec变化，AST与差异检查通过。C++运行源码未变，本次没有重跑旧C++编译或启动ROS/飞行。日志在`/home/ray/TOUDI/.drop_timeout_work_20261012/update20/`。本次不推送、不创建PR。
