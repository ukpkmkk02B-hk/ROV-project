# scripts 脚本说明

本目录保存按实验顺序编号的入口脚本。建议始终从项目根目录运行脚本，不要进入 `scripts/` 目录运行。

## 脚本顺序

- `01_capture_calib_images.py`：打开相机并采集棋盘格标定图片，输出到 `data/calib/<run_id>/raw_images/`。
- `02_calibrate_chessboard.py`：读取棋盘格图片，执行 OpenCV 标定，输出标定结果、误差 CSV、角点调试图和实验日志。
- `03_check_reprojection_error.py`：读取某次 `calib_result.yaml`，打印重投影误差等级。
- `04_capture_aruco_validation.py`：只采集 ArUco/AprilTag 验证图像或视频，不做检测和位姿估计。
- `05_aruco_pose_validation.py`：下一阶段位姿验证脚本。目录和文件名仍按 `aruco` 记录，但当前配置使用 AprilTag `tag36h11` 字典。
- `06_visual_servo_dryrun.py`：只计算视觉伺服控制量，不发送给 ROV，用于调参和安全检查。
- `07_visual_servo_water_test.py`：水下测试入口，当前版本仍为日志模式，不直接发送真实控制命令。

## 05 位姿验证模式

静态距离验证，输出 `aruco_static_validation.csv`：

```bash
python scripts/05_aruco_pose_validation.py \
  --mode static \
  --true-z-m 0.30 \
  --frame-count 300 \
  --output data/aruco_validation/2026_05_19_aruco_100mm_20_50cm/aruco_static_validation.csv
```

横向/方向验证，输出 `aruco_direction_validation.csv`：

```bash
python scripts/05_aruco_pose_validation.py \
  --mode direction \
  --position left \
  --frame-count 120 \
  --output data/aruco_validation/2026_05_19_aruco_100mm_20_50cm/aruco_direction_validation.csv
```

重复性验证，输出 `aruco_repeatability_validation.csv`：

```bash
python scripts/05_aruco_pose_validation.py \
  --mode repeatability \
  --frame-count 300 \
  --output data/aruco_validation/2026_05_19_aruco_100mm_20_50cm/aruco_repeatability_validation.csv
```

## 同时保存调试视频

`05_aruco_pose_validation.py` 支持用 `--save-video` 保存叠加检测框和坐标轴后的调试视频。正式实验建议同时保存 CSV 和视频，方便后续复查位姿异常、反光、遮挡和检测丢帧。

示例：

```bash
python scripts/05_aruco_pose_validation.py \
  --mode static \
  --true-z-m 0.30 \
  --frame-count 300 \
  --output data/aruco_validation/2026_05_19_aruco_100mm_20_50cm/aruco_static_validation.csv \
  --save-video data/aruco_validation/2026_05_19_aruco_100mm_20_50cm/debug_video/static_030m.avi
```

无显示器的 Linux/NUC 上运行时增加 `--no-preview`。如果只想保存间隔调试帧，可以使用：

```bash
--debug-dir data/aruco_validation/<run_id>/debug_frames --save-debug-every 30
```

## 导入说明

脚本中会先导入 `_bootstrap.py`，它会把 `src/` 加入 Python 搜索路径。因此从命令行运行脚本时可以正常导入 `camera`、`utils` 等模块。
