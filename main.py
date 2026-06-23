# main.py

import time
import logging
import json
import os
import sys
from modules.comms.surface_comm import SurfaceComm
from modules.telemetry.openmv_receiver import OpenMVReceiver
from modules.telemetry.rov_video_forwarder import ROVVideoForwarder
from modules.telemetry.status_reporter import StatusReporter

from modules.comms.pixhawk_comm import PixhawkComm
from modules.comms.fish_comm import FishComm
from modules.comms.charging_comm import ChargingComm
from modules.comms.depth_forwarder import DepthReader

from modules.perception.camera import AprilTagCameraInterface
from modules.perception.marker_tracker import ArucoMarkerTracker
from modules.states_machine.state_machine import TaskScheduler
from modules.tasks.docking_task import DockingTask
from modules.tasks.charging_task import ChargingTask
from modules.tasks.fish_control_task import FishControlTask
import yaml

# 强制设置工作目录到rov_project
if not os.path.exists("config/settings.yaml"):
    # 获取当前脚本的真实路径
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    
    # 打印调试信息
    print(f"[DEBUG] 当前工作目录: {os.getcwd()}")
    print(f"[DEBUG] 脚本所在目录: {BASE_DIR}")
    print(f"[DEBUG] 配置文件状态: {os.path.exists(BASE_DIR + '/config/settings.yaml')}")
    
    # 修改工作目录
    os.chdir(BASE_DIR)
    print(f"[FIX] 已修正工作目录到: {os.getcwd()}")
# ✅ 日志设置
logging.basicConfig(level=logging.INFO)


def create_main_camera(config, surface):
    """Create the runtime vision interface from settings.yaml."""
    vision_config = config.get("vision_tracking", {})
    marker_type = vision_config.get("marker_type", "apriltag").lower()
    if marker_type == "aruco":
        return ArucoMarkerTracker(vision_config, surface=surface)
    return AprilTagCameraInterface(config["AprilTagCamera_Main"], surface=surface)


def main():
    # def handle_surface_command(msg):
    #     print(f"✅ 收到指令: {msg}")

    # sc = SurfaceComm({"host": "0.0.0.0", "port": 9000})
    # sc.register_command_handler(handle_surface_command)

    # # 让主线程保持运行
    # import time
    # while True:
    #     time.sleep(1)
    print("[MAIN] 🚀 系统启动")
 # === 加载统一配置 ===
    with open("config/settings.yaml", "r") as f:

        config = yaml.safe_load(f)
    # === 1. 初始化通信模块 ===
    surface = SurfaceComm({"host": "0.0.0.0", "port": 9002})  # TCP 接收来自上位机的控制指令
    camera = create_main_camera(config, surface=surface)
    camera.start()

    # reader = DepthReader(serial_port="/dev/ttyDEPTH", baudrate=19200)
    # reader.start()  # 启动深度计读取线程
    
    pixhawk = PixhawkComm(config["pixhawk_comm"])
    pixhawk.start()

    fish = FishComm(config["fish_comm"])

    charger = ChargingComm(config["charging_comm"])

    

    # === 2. 初始化视频转发模块 ===
    openmv_receiver = OpenMVReceiver(config["openmv_receiver"], surface=surface)
    # openmv_receiver.start()

    # === 3. 初始化任务调度系统 ===
    scheduler = TaskScheduler()
    scheduler.register_task(
        "docking",
        DockingTask,
        camera=camera,
        pixhawk=pixhawk,
        tracking_config=config.get("vision_tracking", {}),
    )
    scheduler.register_task("charging", ChargingTask, charging_comm=charger, fish_comm=fish)
    scheduler.register_task("fish_control", FishControlTask, fish_comm=fish)

    # === 4. 状态上报模块 ===
    reporter = StatusReporter(surface=surface, pixhawk=pixhawk, fish=fish, charger=charger, scheduler=scheduler)
    reporter.start()

    scheduler.start()


    # === 5. 控制流程主循环 ===
      # === 上位机指令处理回调 ===
    # def handle_surface_command(msg):
    #     print(f"[MAIN] 📥 上位机指令: {msg}")
    #     if msg == "start_docking":
    #         scheduler.start_task("docking")
    #     elif msg == "start_docking":
    #         scheduler.stop("docking")

    #     elif msg == "start_charging":
    #         charger.start_charging()
    #     elif msg == "stop_charging":
    #         charger.stop_charging()
    #     elif msg == "release":
    #         charger.send_release_command()
    #     elif msg == "connect": 
    #         charger.send_connect_command()

    #     elif msg == "start_fish_control":
    #         fish.send_command("D1")
    #     elif msg == "stop_fish_control":
    #         fish.send_command("D0")

    #     elif msg == "Z0":
    #         pixhawk.send_velocity_command({"vx": 0, "vy": 0.1, "vz": 0, "yaw_rate": 0})

    #     elif msg == "Z1":
    #         pixhawk.send_velocity_command({"vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0})
    #     elif msg == "Z2":
    #         fish.send_command("Z2")
    #     elif msg == "Z3":
    #         fish.send_command("Z3")
    #     elif msg == "Z4":
    #         fish.send_command("Z4")
    #     elif msg == "Z5":
    #         fish.send_command("Z5")
    #     elif msg == "Z7":
    #         fish.send_command("Z7")


    #     elif msg == "reset":
    #         scheduler.reset_error_state()
    #     else:
    #         print("[MAIN] ⚠️ 未知指令")
    # 全局通道状态，初始化为中值
    rc_state = {
        'ch1': 1500,
        'ch2': 1500,
        'ch3': 1500,
        'ch4': 1500,
        'ch5': 1500,
        'ch6': 1500,
        'ch7': 1500,
        'ch8': 1500
    }

    def handle_surface_command(raw_msg):
        # global rc_state
        if not raw_msg or raw_msg.strip() == "":
            return
        
        try:
            msg = json.loads(raw_msg)
        except json.JSONDecodeError:
            print(f"[MAIN] ❌ 无法解析消息: {raw_msg}")
            return
        
        if msg.get("type") != "command":
            return
        
        data = msg.get("data", {})
        
        status_update = {}  # 用于 send_status
        # ========== ROV 控制 ==========
        if "rov" in data:
            rov_cmd = data["rov"]
            print(f"[MAIN] 📥 ROV 指令: {rov_cmd}")
            
            # === 模式切换 ===
            if rov_cmd in ["MANUAL", "STABILIZE", "ALT_HOLD"]:
                success = pixhawk.set_mode(rov_cmd)
                time.sleep(2)
                status_update["mode_change"] = {"target_mode": rov_cmd, "success": success}
            
            # === 上锁/解锁 ===
            elif rov_cmd == "arm":
                pixhawk.arm_vehicle()
                time.sleep(1)
                status_update["armed"] = pixhawk.is_armed()
            elif rov_cmd == "disarm":
                pixhawk.disarm_vehicle()
                time.sleep(1)
                status_update["armed"] = pixhawk.is_armed()
            
            # === 方向控制 (RC override) ===
            elif rov_cmd == "forward":
                rc_state['ch3'] = 1500
                rc_state['ch4'] = 1500
                rc_state['ch5'] = 1600
                rc_state['ch6'] = 1500
            elif rov_cmd == "backward":
                rc_state['ch3'] = 1500
                rc_state['ch4'] = 1500
                rc_state['ch5'] = 1400
                rc_state['ch6'] = 1500
            elif rov_cmd == "left":
                rc_state['ch3'] = 1500
                rc_state['ch4'] = 1500
                rc_state['ch5'] = 1500
                rc_state['ch6'] = 1400
            elif rov_cmd == "right":
                rc_state['ch3'] = 1500
                rc_state['ch4'] = 1500
                rc_state['ch5'] = 1500
                rc_state['ch6'] = 1600
            elif rov_cmd == "up":
                rc_state['ch3'] = 1700
                rc_state['ch4'] = 1500
                rc_state['ch5'] = 1500
                rc_state['ch6'] = 1500
            elif rov_cmd == "down":
                rc_state['ch3'] = 1400
                rc_state['ch4'] = 1500
                rc_state['ch5'] = 1500
                rc_state['ch6'] = 1500
            elif rov_cmd == "stop":
                rc_state['ch3'] = 1500
                rc_state['ch4'] = 1500
                rc_state['ch5'] = 1500
                rc_state['ch6'] = 1500
            
            elif rov_cmd == "docking start":
                scheduler.start_task("docking")

            elif rov_cmd == "docking stop":
                scheduler.stop_current_task()
                scheduler.stop()
                print(f"[MAIN]              ⚠️ TINGZHI DUIJIE")
            
            elif msg == "reset":
                scheduler.reset_error_state()
                rc_state['ch3'] = 1500
                rc_state['ch4'] = 1500
                rc_state['ch5'] = 1500
                rc_state['ch6'] = 1500
            else:
                print(f"[MAIN] ⚠️ 未知 ROV 指令: {rov_cmd}")
           
            if status_update:
                surface.send_status(status_update)

        # ========== FISH 控制 ==========
        elif "fish" in data:
            fish_cmd = data["fish"]
            print(f"[MAIN] 📥 FISH 指令: {fish_cmd}")
            if fish_cmd == "forward":
                fish.send_command("F1")
            elif fish_cmd == "up":
                fish.send_command("U1")
            elif fish_cmd == "down":
                fish.send_command("D1")
            elif fish_cmd == "stop":
                fish.send_command("S0")
            elif fish_cmd == "record_start":
                openmv_receiver.send_command("0001")
            elif fish_cmd == "record_stop":
                openmv_receiver.send_command("0002")
            else:
                print(f"[MAIN] ⚠️ 未知 FISH 指令: {fish_cmd}")
        



    surface.register_command_handler(handle_surface_command)
    print("[MAIN] ✅ 系统初始化完成，进入主循环")
    try:
        while True:
             status = scheduler.get_system_status()
             system_state = status['system_state']
             current_task = status['current_task']['name'] if status['current_task'] else None
             
            #  pixhawk.send_velocity_command({"vx": 0, "vy": 0.5, "vz": 0, "yaw_rate": 0})
             pixhawk.send_rc_override(rc_state)
             if system_state == "system_idle":
                print("[MAIN] 💡 系统空闲，等待上位机任务指令")
             elif system_state == "system_error":
                print("[MAIN] ❗ 系统错误，等待上位机发送 reset")
             else:
                print(f"[MAIN] 🔄 当前任务: {current_task}")
             servo_out = pixhawk.get_servo_outputs()
             if servo_out:
                print(f"[PWM] {servo_out}")
             current_pose = camera.get_pose()
                #打印数据（实际使用时可替换为你的业务逻辑）
             if current_pose:
                print("\n最新姿态数据：")
                print(f"标签ID: {current_pose['id']}")
                print(f"坐标(X,Y,Z): ({current_pose['x']:.2f}, {current_pose['y']:.2f}, {current_pose['z']:.2f})")
                print(f"角度(Roll,Pitch,Yaw): ({current_pose['roll']:.1f}, {current_pose['pitch']:.1f}, {current_pose['yaw']:.1f})")
             time.sleep(0.1)  # 每秒检查一次状态    
    except KeyboardInterrupt:
        print("[MAIN] 🛑 收到中断，正在退出...")
        # === 停止所有模块 ===
        scheduler.stop()
        reporter.stop()
        openmv_receiver.stop()
        camera.stop()
        pixhawk.stop()
        fish.stop()
        charger.stop()
        surface.stop()
        print("[MAIN] ✅ 系统安全退出")

if __name__ == "__main__":
    main()
