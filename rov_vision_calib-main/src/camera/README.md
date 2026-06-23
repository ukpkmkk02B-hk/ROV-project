# camera 相机模块说明

本目录负责相机相关功能。

## 文件说明

- `uvc_camera.py`：根据 `config/camera_rov_front.yaml` 打开 USB/UVC 相机，设置分辨率、帧率、MJPG、曝光、增益等参数。
- `camera_params.py`：读取标定文件中的相机内参 K 和畸变 D，并提供分辨率变化时的内参缩放函数。

## 可能输出

本模块本身不直接写实验结果。它主要向采集脚本和视觉算法提供相机帧、内参和畸变参数。
