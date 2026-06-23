# calibration 标定模块说明

本目录负责棋盘格标定和误差评估。

## 文件说明

- `chessboard_calibrator.py`：查找棋盘格角点、执行相机标定、保存标定结果和报告。
- `reprojection.py`：计算每张图片的重投影误差。

## 典型输出

执行 `scripts/02_calibrate_chessboard.py` 后会在对应 `data/calib/<run_id>/` 下生成：

- `calib_result.yaml`：该次标定的完整结果。
- `calib_report.csv`：标定概要指标。
- `per_image_error.csv`：每张有效图像的误差。
- `corners_debug/`：角点检测可视化图片。
- `accepted_images/`：通过角点检测的图片副本。
- `rejected_images/`：未通过检测的图片副本。
- `experiment_log.md`：该次标定实验日志。
