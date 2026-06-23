# data/visual_servo 视觉伺服数据说明

本目录保存视觉伺服 dry-run 和水下测试记录。

## 每次测试目录内容

- `raw_video/`：原始相机视频。
- `debug_video/`：叠加检测结果和控制状态的视频。
- `visual_servo_log.csv`：视觉测量、误差和目标状态日志。
- `control_log.csv`：控制输出、RC 命令、限幅状态日志。
- `servo_report.md`：人工整理的测试结论。

## 使用规则

第一次水下测试建议只做 dry-run 或小限幅控制，确认 marker 稳定后再逐步增大控制量。
