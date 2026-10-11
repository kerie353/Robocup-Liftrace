# 运动优化保留门前后锚点（2026-10-11）

已在导航工作树 board-trial-20260928 定向修复 optimize_post_route，全局 merge_collinear_relays 保持开启。本任务只修改该模块、两份相关测试与本说明，不提交、不更新共享日志、不运行仿真或上板。工作树另有其他 agent 的起飞相关改动，本任务未修改或暂存它们。

原逻辑只检查水平共线，会删除同一直线上的门前后点。现在逐个已知墙面扫描原始完整路线，保护每次侧别变化的最近严格前侧与后侧点。保护按原始零基索引保存，删点和对角入口移除同步维护索引，重复经过同一坐标也按各次经过独立处理。多墙、反向、斜线和单段跨多墙均适用。墙面点（轴向距离不超过 1e-6 m）保守保留，连续墙面点跨墙时也保留其两侧锚点；只碰到墙面后原侧返回不记录为穿墙。完整路线包含 H 后缀，跨墙延伸到 H 后缀时仍保护最近前侧点；H 后缀本身不合并。非保护点沿用原水平共线、同向判断，保留转折、折返、高度变化。正常高低入口仍可对角合并；入口点恰在墙面时保留显式高度变化。

元数据保留原字段，新增 wall_axis、protected_original_indices、protected_crossings、retained_original_indices。protected_crossings 指明墙坐标、每次跨墙前后原始索引及墙面点索引，可从 original 还原保护原因。接口签名与开关不变。

## 现场七点示例

墙面 Y=0 / -1.4，FC AGL 均为0.9 m，XY输入及输出均为：

~~~text
(6.5, 2.1) -> (7.7, 2.1) -> (7.7, 1.0) -> (7.7, -0.7)
           -> (7.7, -1.1) -> (7.7, -2.5) -> (7.7, -3.45)
~~~

| 墙面 | 保护门前点 | 保护门后点 | 原始索引 |
| --- | --- | --- | --- |
| Y=0 | (7.7, 1.0) | (7.7, -0.7) | 2 / 3 |
| Y=-1.4 | (7.7, -1.1) | (7.7, -2.5) | 4 / 5 |

七点全部保留。正式配置生成测试另在 Y=-2.5 与 -3.45 间增加普通共线点 Y=-3.0：该点仍合并，8个走廊点优化后为7个，四个门点和完整H后缀不变；零参考与非零参考平移均通过。

## 实际生成流程与验证结果

rl_drone 中 test_motion_optimization + test_motion_profiles 共33项全部 PASS，无跳过；git diff --check 通过。包括原有边界制动、动态恢复和九种试飞配置兼容测试。正式生成测试原有软件限高硬编码断言由3.0改为读取模板 max_agl（当前3.2），没有改配置值。

调用真实 generate 生成临时 runtime/control/overrides，检查实际参数阶段列表，而非只比较优化函数航点列表。下表数量均不含H后缀：

| 生成入口 / 测试夹具 | 原走廊数 | 对角入口 | 优化后走廊数 / H阶段完成数 | 低入口完成数 | H阶段 FC AGL 限高 |
| --- | --- | --- | --- | --- | --- |
| competition_config.generate | 8 | 开 | 6 / 6 | 1 | 1.3 m |
| competition_config.generate | 8 | 关 | 7 / 7 | 2 | 1.3 m |
| trial_config.generate -> trial_motion | 6 | 开 | 4 / 4 | 1 | 2.9 m |
| trial_config.generate -> trial_motion | 6 | 关 | 5 / 5 | 2 | 2.9 m |

正式夹具仅在测试内把H捕获 AGL设为1.2，验证走廊1.0限高到H阶段1.3确实恢复；不改运行YAML。试飞夹具验证 external_planner_max_command_z 与 planner_bridge execution/max_goal_z 两项在完成走廊前保持1.0，并恰在优化后走廊数量处恢复。两条生成链均验证H后缀与优化前一致、入口计数正确；对角入口移除与普通中继合并同时发生也不会错移阶段。无需修改 competition_config.py 或 trial_motion.py 接线。

复现（WSL内执行，无ROS节点）：

~~~bash
cd /home/xhj/liftrace-controlwork-worktrees/board-trial-20260928
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
export PYTHONPATH="$PWD/patrol_uav_ws-patrol_planner/src/uav_mission/src:$PWD/vision_ws/src/uav_high_view/src:$PWD/deployment/board_trials_4x4/common/uav_board_trials/scripts:$PWD/patrol_uav_ws-patrol_planner/src/uav_mission/test:$PWD/deployment/board_trials_4x4/common/uav_board_trials/test:/opt/ros/noetic/lib/python3/dist-packages"
export PYTHONPYCACHEPREFIX="$(mktemp -d /tmp/door-route-pycache.XXXXXX)"
python -m unittest -v test_motion_optimization test_motion_profiles
~~~

用独立临时字节码目录绕过工作树已有 mission_core 的 bad marshal data 缓存，未删除或改写该缓存。验证覆盖几何保留与离线生成的阶段接线，不等价实飞过门成功或H阶段实际执行验收。墙保护依赖提供的 wall_axis / wall_coordinates，不会自动识别门洞横向中心，也不代替实体避障。

本任务文件：

- patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/motion_optimization.py
- patrol_uav_ws-patrol_planner/src/uav_mission/test/test_motion_optimization.py
- deployment/board_trials_4x4/common/uav_board_trials/test/test_motion_profiles.py
- docs/verification/door_waypoint_protection_20261011/REPORT.md
