# perception 感知模块说明

本目录负责 ArUco 检测和位姿估计。

## 文件说明

- `aruco_detector.py`：创建 ArUco 检测器，支持字典选择和角点亚像素优化。
- `pose_estimator.py`：根据 marker 角点、marker 尺寸、相机 K/D 计算 rvec、tvec 和 yaw。

## 典型输出

执行 ArUco 验证脚本后，结果通常写入 `data/aruco_validation/<run_id>/`，包括：

- `aruco_static_validation.csv`：静态距离和位姿验证记录。
- `aruco_direction_validation.csv`：方向变化验证记录。
- `debug_frames/`：带角点和坐标轴的调试帧。
