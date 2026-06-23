# reports 报告目录说明

本目录保存人工整理后的总结，不直接保存大量原始数据。

## 文件说明

- `calibration_summary.md`：汇总多次标定结果，记录最终选用哪一次。
- `aruco_validation_summary.md`：汇总 ArUco 位姿误差验证结果。
- `visual_servo_summary.md`：汇总视觉伺服 dry-run 和水下测试结论。
- `figures/`：报告用图，例如误差曲线、距离误差图、控制响应曲线。

## 使用规则

报告中引用实验结果时，应写明对应的 `data/<task>/<run_id>/`，避免后续复现实验时找不到数据来源。
