# ROV 视觉标定项目

本项目用于水下 ROV 前视相机的第一阶段视觉实验，当前路线是：

1. 水下等效针孔标定
2. ArUco 位姿误差验证
3. 视觉伺服 dry-run
4. 水下闭环测试记录

## 当前实验条件

- 相机：Sony IMX291，USB/UVC，Linux 设备节点 `/dev/video0`
- 平台：NUC / Ubuntu / Linux，当前代码也可在 Windows 下编辑
- 标定分辨率：1920 x 1080
- 标定板：9 x 12 方格，单格 15 mm
- OpenCV 初始内角点：8 x 11
- 第一次标定距离范围：20-50 cm

## 目录说明

- `config/`：相机、标定、ArUco、视觉伺服配置。
- `scripts/`：按实验顺序编号的入口脚本。
- `src/`：可复用代码模块。
- `data/`：每次实验的数据和结果，按实验日期/任务分目录保存。
- `reports/`：人工整理后的总结、图表和报告。

## Windows 虚拟环境

```powershell
cd F:\vscode_else\arv_biaoding_sifu\rov_vision_calib
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

`.venv/` 已写入 `.gitignore`，不会提交到 GitHub。

## 常用流程

采集棋盘格图像：

```powershell
python scripts/01_capture_calib_images.py --camera config/camera_rov_front.yaml --output data/calib/2026_05_19_underwater_20_50cm_1080p/raw_images
```

执行标定：

```powershell
python scripts/02_calibrate_chessboard.py --camera config/camera_rov_front.yaml --calib-template config/calib_underwater_20_50cm_1080p.yaml --images data/calib/2026_05_19_underwater_20_50cm_1080p/raw_images --output data/calib/2026_05_19_underwater_20_50cm_1080p --update-latest config/calib_underwater_20_50cm_1080p.yaml
```

检查重投影误差：

```powershell
python scripts/03_check_reprojection_error.py --result data/calib/2026_05_19_underwater_20_50cm_1080p/calib_result.yaml
```

## 标定版本管理规则

- 每次重新标定都新建一个 `data/calib/<日期_条件>/` 目录。
- `data/calib/<run_id>/calib_result.yaml` 是该次实验的永久结果。
- `config/calib_underwater_20_50cm_1080p.yaml` 是当前默认使用的最新标定参数。
- 只有确认某次标定可用后，才把该次结果同步到 `config/`。
