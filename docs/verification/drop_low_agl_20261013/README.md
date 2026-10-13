# 放行0.15/0.20/0.40m投递高度

> 2026-10-14后续调整为最低0.10/目标0.15/最高0.43m，下限同步降到0.15m，见[最新记录](../drop_015_agl_20261014/README.md)。下文为上一版归档。

用户指定最低释放AGL 0.15m、下降目标AGL 0.20m、最高释放AGL 0.40m。已在香橙派 `10.189.79.232` 的 `/home/orangepi/liftrace_board_trials_20260928` 修改、验证，并同步本地生成器及控制器代码；板端08的普通/competition速度入口均保留这组三参数。本地入口默认高度本轮未修改。

## 校验变化

- 正式及试飞生成器的目标AGL下限降为0.20m，原上限1.0m保留。
- 投递任务Z不再要求大于0.05m；Z经过地面标定换算，符号不能代表实际离地高度。生成器已有有限静置参考、`0 < min <= target <= max <= 1.1`和地面标定检查继续保留。
- 控制器仅对 `external_mission_mode && compensated_alignment && drop_metric_scale_enabled` 的标定投递改用AGL校验：有限ground_z、最低AGL>0、目标AGL>=0.20m、最高AGL<=1.1m，同时仍要求有序高度和原恢复高度约束。目标边界使用1e-9m数值余量避免浮点换算误差。
- 未标定旧投递入口保留原本地正Z限制。H的本地高度校验、起飞、导航、XY/速度门控、20秒兜底、舵机及恢复业务不变。

## 实际生成及构建

使用已有 `board_full_mission_20261013_184429/ground_reference.json` 中的静置参考（ground_z=-0.21532178812660277m），生产入口 `full_mission preview --check-config --generate-config --reference-fc ... --speed-profile competition` 成功生成：

| 参数 | FC AGL | 任务坐标Z |
| --- | ---: | ---: |
| release_min_height | 0.15m | -0.0653217881m |
| release_setpoint_height | 0.20m | -0.0153217881m |
| height_threshold | 0.40m | 0.1846782119m |

生成目录：`/home/orangepi/.codex_backups/drop_height_20261013_192149/generated_check`。这是离线验证产物，未覆盖历史运行目录，下一次实际启动仍需重新采集静置参考并按原入口生成。

已提前告知用户需要编译控制器，板端执行目标构建 `cmake --build patrol_uav_ws-patrol_planner/build --target patrol_control -- -j2`，构建成功；输出为 `devel/lib/patrol_control/patrol_control`。没有启动该程序或任何飞行/舵机动作。

- 板端12项相关测试通过，包含正式/试飞配置生成、生产C++启动校验提取测试（5种原点、3种旧入口及9种无效几何）和已有高度配置检查。
- 本地9项Python配置测试通过。
- 5个原控制/降落函数与修改前快照一致；差异检查通过。
- 更新旧测试中“本地Z<=0.05必拒绝”的预期，改为检查标定AGL有效性；未改变任务许可高度余量。

板端源码、原二进制、构建日志及检查结果：`/home/orangepi/.codex_backups/drop_height_code_20261013_192847`。本地源码快照位于工作区外 `tmp/drop_height_board_20261013/source_before_20261013_192846`。

软件放行不会改变当前静置FC离地0.22m的标定，不能把配置通过理解为飞机物理上能达到比静置FC更低的高度。本轮不调整标定，不进行飞行或载荷落点验收。
