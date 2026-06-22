# test_docking_task.py
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import yaml
import time
import logging
from modules.perception.camera import CameraInterface
from modules.comms.pixhawk_comm import PixhawkComm
from modules.states_machine.state_machine import TaskScheduler
from tasks.docking_task import DockingTask

# 日志配置（根据需要可设置为 DEBUG）
logging.basicConfig(level=logging.INFO)

def load_config(path="config/settings.yaml"):
    with open(path, 'r') as f:
        return yaml.safe_load(f)
    
def main():

    # 加载配置文件
    config = load_config()

    # === 初始化摄像头接口 ===
    cam_cfg = config.get("camera", {})
    camera = CameraInterface(config=cam_cfg)
    camera.start()  # ✅ 启动摄像头线程
    
    # === 初始化 Pixhawk 通信 ===
    pix_cfg = config.get("pixhawk", {})
    pixhawk = PixhawkComm(config=pix_cfg)
    pixhawk.start()  # ✅ 启动 Pixhawk MAVLink 通信线程
    
    # 初始化任务调度器
    scheduler = TaskScheduler()
    scheduler.register_task("docking", DockingTask, camera=camera, pixhawk=pixhawk)
    scheduler.start()  # ✅ 启动调度线程
    
    # 启动对接任务
    scheduler.start_task("docking")
    
    try:
        while True:
            time.sleep(1)
            status = scheduler.get_system_status()
            print("=== 任务状态 ===")
            print(f"系统状态: {status['system_state']}")
            if status['current_task']:
                print(f"当前任务: {status['current_task']['name']}")
                print(f"阶段: {status['current_task']['stage']}")
                print(f"状态: {status['current_task']['status']}")
                print(f"最后位姿: {status['current_task']['last_pose']}")
            print("================\n")

    except KeyboardInterrupt:
        print("⏹️ 中断：正在停止任务和通信线程...")
        scheduler.stop()
        camera.stop()
        pixhawk.stop()

if __name__ == "__main__":
    main()
