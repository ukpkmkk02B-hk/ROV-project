# data 数据目录说明

本目录保存实验原始数据和每次运行产生的结果。

## 子目录

- `calib/`：棋盘格标定数据和结果。
- `aruco_validation/`：ArUco 位姿验证数据和结果。
- `visual_servo/`：视觉伺服 dry-run 和水下测试日志。

## 版本规则

每次实验都新建独立目录，目录名包含日期、实验类型、关键条件。不要覆盖旧实验目录。

示例：

- `calib/2026_05_19_underwater_20_50cm_1080p/`
- `aruco_validation/2026_05_19_aruco_100mm_20_50cm/`
- `visual_servo/2026_05_20_servo_baseline_p_control/`

大型图片、视频和自动生成的 CSV/YAML 默认不提交到 GitHub。
