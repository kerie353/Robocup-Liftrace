# 2026-10-11 板端 YOLO v8 模型切换

用户授权将最新候选模型作为板载默认模型，旧权重与元数据保留原位。

- 新模型：`runtime_models/flight_5cls_new_video_v8_20261011/flight_5cls_new_video_v8_20261011_fp16.rknn`
- 配套元数据：`vision_ws/src/uav_vision/config/flight_5cls_new_video_v8_20261011_metadata.yaml`（与模型目录 metadata.yaml 内容一致）
- FP16，640×640，RGB/NHWC，输出 [1,9,8400]，五类 bridge/panzer/pillbox/tent/red_cross。
- 只切换试验入口、比赛入口、视觉 launch 及部署打包选择；检测阈值、速度优化、相机、飞控、投递逻辑未修改。
- 完整原始包保存在 runtime_models/flight_5cls_new_video_v8_20261011/，权重/训练产物不提交 Git。

## 验证

在当前香橙派、现有运行库上离线运行保存图像 5 张和黑/白/噪声图 3 张；加载与 NPU 初始化成功，全部输出形状/类别契约正确且数值有限，标准输入与 native FP16 输入输出逐元素完全相同。没有启动相机、任务、解锁或舵机，没有编译或升级运行库。

运行库报告模型 toolkit 2.3.2 与 librknnrt 1.5.2 不匹配警告；本次静态模型推理通过。离线烟测不等于精度或实飞验收；原包仍标记 candidate，已有帐篷裁剪图误判碉堡的回归记录仍需现场复核。

静态 shell/XML/Python 与配套元数据检查通过；检测契约及执行优化测试共 12 项通过；第五组 --check-config 通过且未启动 ROS 节点。

## 备份与回滚

备份目录：`/home/orangepi/board_deploy_backups/yolo_v8_20261011/`，含切换前配置、旧模型/元数据、HEAD、manifest.json，以及 check_model.py 和 equivalence.json。

旧模型：`runtime_models/flight_5cls_20260928_fp16.rknn`。
旧元数据：`vision_ws/src/uav_vision/config/flight_5cls_20260928_metadata.yaml`。

需要完整恢复默认配置时，在不运行任务的情况下执行 `python3 /home/orangepi/board_deploy_backups/yolo_v8_20261011/rollback.py`；该脚本仅恢复本次备份的选择配置，不删除新旧模型，执行后 sync。恢复结果会作为未提交更改保留，便于审核。
