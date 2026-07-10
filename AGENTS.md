# AGENTS.md

## 项目真实状态

这个仓库是母鱼 ROV / ARV 与子机器人相关的 Python 控制软件。

根据当前真实代码结构判断：

* 当前主工程是 Python 硬件控制工程。
* 主入口是 `main.py`。
* 当前未发现 ROS1 / ROS2 / CMake 工程标识文件，例如 `package.xml`、`CMakeLists.txt`、`setup.py`、`pyproject.toml`、`launch/`。
* 因此不要默认把本项目当作 ROS catkin、ROS2 colcon 或普通 CMake 工程处理。

根项目主要负责：

* ROV / ARV 主流程启动。
* Pixhawk / ArduSub MAVLink 通信。
* 子机器人串口通信。
* 充电模块串口通信。
* 上位机 TCP 通信和状态上报。
* ArUco 单目视觉跟踪、位姿估计和对接控制。
* 对接、充电、子机器人控制等任务调度。

`rov_vision_calib-main/` 是相机标定、ArUco 位姿验证和视觉伺服实验资料目录，不是当前主控制程序入口。

## 运行环境

最终目标运行环境是 Linux 小电脑，不是 Windows。

Windows 只用于临时阅读、分析和编辑代码。最终依赖安装、摄像头访问、串口访问、Pixhawk 通信、运行验证和硬件验证都必须回到 Linux 目标机器上完成。

目标机器信息：

```text
User/host: cekong418@cekong418-NUC11PAHi7
OS: Ubuntu 22.04.5 LTS
Release codename: jammy
Kernel: 6.8.0-79-generic
Architecture: x86_64
Original path: /home/cekong418/arv_project
```

不要将 Linux 特定逻辑改成 Windows 特定逻辑。除非用户明确要求，否则不要修改：

* Linux 路径。
* shell 命令为 PowerShell 命令。
* `/dev/...` 设备路径。
* Linux 服务、权限或硬件访问逻辑。

Windows 解压副本中的 `.venv` 可能缺失符号链接，这是正常现象。不要修复或依赖 Windows `.venv`。

## 目录和入口

当前关键目录和文件：

```text
main.py                         主程序入口
config/settings.yaml            根项目硬件和通信配置
requirements.txt                根项目 Python 依赖声明
modules/comms/                  Pixhawk、子鱼、充电、深度计、上位机通信
modules/perception/             感知相关代码，当前主线是 ArUco 标志跟踪和位姿估计
modules/controller/             对接控制器
modules/tasks/                  对接、充电、子机器人控制任务
modules/states_machine/         任务调度状态机
modules/state/                  状态估计/预测预留模块，当前部分文件为空
modules/telemetry/              视频转发、OpenMV 接收、状态上报
tests/                          手动连接或硬件测试脚本，不等同于完整自动化测试
rov_vision_calib-main/          相机标定、ArUco 验证、视觉伺服实验目录
```

`config/settings.yaml` 中包含真实硬件相关配置，例如：

* Pixhawk 串口：`/dev/ttl_pixhawk`
* 子机器人串口：`/dev/ttl_fish_main`
* 充电模块串口：`/dev/ttl_charger`
* 主摄像头设备：`/dev/camera_main`
* OpenMV / 蓝牙串口：`/dev/ttl_bluetooth`
* 深度计串口：`/dev/ttyDEPTH`

这些配置属于硬件接口边界，不能随意修改。

## 硬件安全约束

本项目可能访问真实 ROV / ARV 硬件，包括摄像头、串口设备、Pixhawk、推进器、深度传感器、子机器人、无线充电模块和对接相关设备。

任何可能改变硬件行为的修改都必须谨慎。未经明确要求和风险说明，不要修改：

* 串口名称。
* 波特率。
* MAVLink 连接方式。
* 自定义串口协议。
* TCP 通信协议。
* 视觉标志尺寸。
* 坐标系定义。
* 控制方向符号。
* RC / PWM 通道映射。
* PWM / 推力限制。
* 电机混合或推进控制逻辑。
* 控制频率。
* Pixhawk 模式切换逻辑。
* 解锁 / 上锁逻辑。
* 启动行为。
* 停止行为。
* 安全检查。
* 急停或失控保护逻辑。

以下脚本或入口可能访问硬件、连接 Pixhawk、发送 RC/MAVLink 或执行危险动作。不要把它们作为普通验证命令随意运行：

```text
main.py
test_pix.py
modules/comms/111111.py
modules/comms/22222.py
modules/comms/pixhawk_comm.py
tests/pixhawk.py
```

如果任务涉及硬件控制行为，先说明风险、预期效果和验证方式，再进行编辑。

## 视觉与标定说明

当前真实硬件和运行主线已经确定使用 ArUco 标志，不再以 AprilTag 作为视觉跟踪或对接方案。

根项目运行链路：

* 文件：`modules/perception/marker_tracker.py`
* 类：`ArucoMarkerTracker`
* 方法：OpenCV `VideoCapture` 读取摄像头。
* 检测：OpenCV `cv2.aruco` ArUco。
* 配置：`config/settings.yaml` 中的 `vision_tracking`。
* 当前配置：`marker_type: aruco`、`DICT_4X4_50`、marker id `20`、marker size `0.04 m`。
* 主摄像头设备：`/dev/camera_main`。
* 水下标定文件：`rov_vision_calib-main/config/calib_underwater_20_50cm_1080p.yaml`。
* 位姿解算：`cv2.solvePnP`。

`modules/perception/camera.py` 中的 `AprilTagCameraInterface` 和 `config/settings.yaml` 中的 `AprilTagCamera_Main` 是遗留线索，不代表当前视觉跟踪主线。除非用户明确要求清理遗留代码，否则不要仅为统一命名而删除或重构它们。

标定和视觉实验目录：

* 目录：`rov_vision_calib-main/`
* 用途：相机标定、ArUco 位姿验证、视觉伺服 dry-run 和水下实验记录。
* 相机：Sony IMX291，USB / UVC，Linux `/dev/video0`。
* 标定：水下等效针孔模型。
* ArUco 配置：`rov_vision_calib-main/config/aruco_config.yaml`。
* 当前实验配置示例：`DICT_4X4_50`，marker id `20`，marker size `0.04 m`。

修改单目跟踪、对接识别或视觉伺服时，应以 ArUco 主线为准，并保持当前标志字典、marker id、物理尺寸、相机内参来源和坐标系定义；只有新的实机证据和用户明确要求才能变更这些硬件边界参数。

## 修改规则

编辑前必须先阅读相关文件，并根据真实代码结构判断影响范围。

一般规则：

1. 先检查 `AGENTS.md`。
2. 检查项目结构和入口点。
3. 阅读相关源码和配置。
4. 简要说明计划改动。
5. 保持改动最小化、局部化。
6. 优先基于现有代码补功能，不要推翻重写。
7. 除非任务明确要求，否则保留现有行为。
8. 除非任务明确要求，否则不要大规模重构。
9. 未经确认，不要删除已有功能或安全逻辑。
10. 除非必要，不要新增生产依赖。
11. 如果行为不确定，向用户索要 Linux 构建日志、运行时日志、硬件现象或相关源文件。

不要修改、提交或依赖以下生成目录和运行时文件：

```text
.venv/
build/
devel/
install/
log/
__pycache__/
*.pyc
```

## 验证规则

最终功能验证必须在 Linux 目标机器上完成。Windows 只能作为阅读、编辑和静态分析环境。

当前项目默认不要使用以下命令作为构建验证：

```bash
catkin_make
colcon build
cmake ..
make -j$(nproc)
```

只有未来项目中真实出现 ROS1、ROS2 或 CMake 标识文件时，才根据实际结构选择对应构建流程。

### 文档类修改

如果只修改 `AGENTS.md` 或其他文档，不要运行硬件入口。验证内容应限制为：

```powershell
Get-Content -Encoding UTF8 -LiteralPath AGENTS.md
git diff -- AGENTS.md
git diff --name-only
```

### Python 功能类修改

如果修改 Python 功能代码，应优先在 Linux 目标机器上做静态检查和导入检查，例如：

```bash
cd /home/cekong418/arv_project
python3 -m py_compile main.py
python3 -m py_compile modules/perception/camera.py
python3 -m py_compile modules/tasks/docking_task.py
python3 -m py_compile modules/comms/pixhawk_comm.py
```

依赖导入检查示例：

```bash
cd /home/cekong418/arv_project
python3 - <<'PY'
import cv2
import numpy
import yaml
import serial
from pymavlink import mavutil
PY
```

根 `requirements.txt` 当前可能不完整。源码中还能看到 `cv2`、`numpy`、`crcmod`，以及遗留 AprilTag 文件使用的 `pupil_apriltags` 等导入；后续如需整理依赖，应先在 Linux 上确认实际环境和导入结果。

### 硬件相关修改

如果修改会影响摄像头、串口、Pixhawk、推进器、对接或充电模块，必须在 Linux 目标机器上做硬件 dry-run 或受控测试。

在没有用户明确确认前，不要运行会解锁、发送 RC、发送 MAVLink 控制、启动推进器或访问真实硬件的命令。

## Codex 工作流程

当用户要求分析项目时：

1. 先读取 `AGENTS.md`。
2. 检查真实项目结构，不盲目沿用历史假设。
3. 明确主入口、配置文件、硬件接口和相关源码。
4. 区分根控制工程和 `rov_vision_calib-main/` 标定实验目录。
5. 只输出分析结论，不修改代码，除非用户明确要求。

当用户要求修改代码或文档时：

1. 先确认修改范围。
2. 只修改必要文件。
3. 不触碰无关硬件参数和安全逻辑。
4. 修改前说明将要改什么。
5. 修改后给出实际执行过的验证命令。
6. 如果无法在本地验证，明确说明需要用户在 Linux 目标机器上运行的命令和需要提供的完整日志。

当用户要求实现单目视觉跟踪、预对接或视觉伺服时：

1. 以已经确定的 ArUco 视觉标志为准，不再把 AprilTag 作为待选方案。
2. 先确认相机内参、畸变参数和标志物实际尺寸。
3. 先确认相机坐标系到母机器人坐标系的转换关系。
4. 先做 dry-run 日志，不直接发送推进控制。
5. 再小步接入滤波、目标丢失处理、状态机和控制输出。
6. 最后在 Linux 目标机器上进行受控硬件验证。
