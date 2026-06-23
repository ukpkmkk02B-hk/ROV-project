# data/calib 标定数据说明

本目录保存棋盘格标定相关数据。

## 每次标定目录结构

每次标定使用一个独立的 `<run_id>/` 目录，例如：

`2026_05_19_underwater_20_50cm_1080p/`

目录内通常包含：

- `raw_images/`：原始采集图片。
- `accepted_images/`：成功检测角点的图片副本。
- `rejected_images/`：角点检测失败或质量不合格的图片副本。
- `corners_debug/`：绘制角点后的调试图片。
- `calib_result.yaml`：本次标定完整结果。
- `calib_report.csv`：标定概要。
- `per_image_error.csv`：每张图片重投影误差。
- `experiment_log.md`：实验记录。

## 防混乱规则

不要把不同距离、不同分辨率、不同防水舱状态的图片混到同一个目录。
