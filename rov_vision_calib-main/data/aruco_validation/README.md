# data/aruco_validation ArUco 位姿验证数据说明

本目录保存第二阶段位姿验证数据。当前实际使用 AprilTag `tag36h11`，但为了以后换回 ArUco 后目录不变，文件夹和 CSV 名称仍保留 `aruco`。

## 每次验证目录内容

- `raw_video/`：原始视频或采集帧，通常由 `04_capture_aruco_validation.py` 采集。
- `debug_frames/`：带 marker 边框、角点、坐标轴的调试图。
- `debug_video/`：由 `05_aruco_pose_validation.py --save-video` 保存的带检测框和坐标轴的视频。
- `aruco_static_validation.csv`：静态距离验证记录。
- `aruco_direction_validation.csv`：横向/方向验证记录。
- `aruco_repeatability_validation.csv`：重复性验证记录。
- `validation_report.md`：人工整理的验证结论。

## 当前下一步验证

Step 1 静态距离验证：

- 固定 tag 在已知距离，例如 `0.20 m / 0.30 m / 0.40 m / 0.50 m`。
- 每个距离采集约 300 帧。
- 记录检测率、`x/y/z/yaw` 均值和标准差，以及 `z_error_m`。
- 建议同时保存 `debug_video/static_020m.avi`、`debug_video/static_030m.avi` 等调试视频。

Step 2 横向方向验证：

- 把 tag 放在画面 `center / left / right / up / down`。
- 检查像素误差方向和控制方向符号是否符合预期。
- 例如 tag 在左侧时，`ex_px < 0`，`control_x_sign` 应为 `positive`。

Step 3 重复性验证：

- tag 和相机都保持静止。
- 采集约 300 帧。
- 重点看 `x_std_m / y_std_m / z_std_m / yaw_std_deg`，判断位姿输出是否稳定。

## CSV 字段

`aruco_static_validation.csv`：

```text
date,calib_file,resolution,marker_size_m,true_z_m,frame_count,detect_count,detect_rate,x_mean_m,x_std_m,y_mean_m,y_std_m,z_mean_m,z_std_m,z_error_m,yaw_mean_deg,yaw_std_deg
```

`aruco_direction_validation.csv`：

```text
date,position,image_u,image_v,ex_px,ey_px,tvec_x_m,tvec_y_m,tvec_z_m,control_x_sign,control_y_sign,result
```

`aruco_repeatability_validation.csv`：

```text
date,calib_file,resolution,marker_size_m,frame_count,detect_count,detect_rate,x_mean_m,x_std_m,y_mean_m,y_std_m,z_mean_m,z_std_m,yaw_mean_deg,yaw_std_deg
```

## 注意事项

- `marker_size_m` 必须是单个 tag 黑色外框的实际边长，不是整张纸的尺寸。
- 如果一张图里同时出现多个 tag，默认只使用 `config/aruco_config.yaml` 中的 `marker_id`。需要临时使用任意检测到的 tag 时，可以加 `--allow-any-marker`。
- `--save-video` 保存的是叠加检测结果后的调试视频，不是原始未处理视频。需要原始视频时使用 `04_capture_aruco_validation.py`。
