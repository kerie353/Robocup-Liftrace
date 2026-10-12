# 编译与回归结果（2026-10-12）

- 源码：`9d1f80ce`，分支 `liftrace_board_trial_20260928`。
- 环境：WSL `U20.04-UAV`，Ubuntu 20.04 / ROS Noetic / GCC 9 / 系统 Python 3.8。
- 结果：当前 checkout 的 `navigation_recovery_msgs`、`uav_vision`、`patrol_control`、`uav_mission` Catkin 构建成功，`patrol_control` 实际编译、链接完成。未重编整个导航/驱动栈，`plan_manage` 等未修改依赖使用已有工作区 overlay。
- 独立构建目录：`/tmp/h_landing_build_20261012_fw1pPq`。src 链接当前 F 盘源码，未覆盖原工作区 build/devel。
- 控制器：`/tmp/h_landing_build_20261012_fw1pPq/devel/lib/patrol_control/patrol_control`（WSL x86_64 产物，不是 OrangePi ARM 二进制）。

最终构建参数：

```bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -Dcv_bridge_DIR=/opt/ros/noetic/share/cv_bridge/cmake \
  -DCATKIN_ENABLE_TESTING=ON -j2
```

首次构建成功，但旧 overlay 的 cv_bridge 引入 OpenCV 3.4/4.2 混用警告；最终显式使用系统 ROS cv_bridge 重新构建成功，无该链接警告。
控制器 ldd 无缺库；视觉落地检测器在 `LD_LIBRARY_PATH=本次devel/lib:/opt/ros/noetic/lib` 下确认仅使用系统 cv_bridge/OpenCV 4.2。
旧工作区的运行环境未修改，部署时仍应避免其旧 cv_bridge 覆盖系统库。

## 回归

149 个不同用例全部通过：

- 控制器 117 项：H 三下降模式、低位交接、高位对准、交接事务、投递几何/下降、起飞交接。包括生产 C++ 提取编译运行。
- 任务及 ROS 接口 32 项：实际 ROS 类型/TF 转换、H 配置与 raw 掩码、投递下降配置和接口。

第一轮控制器测试 116 项通过，一个 ROS 消息序列化测试因未加载新消息包失败；加载本次 `devel/setup.bash` 后该用例补测通过。测试只进行序列化/替身接口检查，未启动 ROS master 或飞行节点。

日志存于工作区忽略目录 `deployment_results/h_landing_build_20261012/`：
`build_system_cv_bridge.log`（最终构建）、`control_tests.log`（首轮）、
`transport_test.log`（补测）、`mission_tests.log`、`linkage.log`、`vision_linkage.log`。

本次无需修复业务源码；未部署、未启动仿真、未实飞。
