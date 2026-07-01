import threading
import time
from modules.comms.surface_comm import SurfaceComm

class StatusReporter:
    def __init__(self, surface: SurfaceComm, pixhawk=None, fish=None, charger=None, scheduler=None):
        # 初始化状态数据来源与通信接口
        self.surface = surface
        self.pixhawk = pixhawk
        self.fish = fish
        self.charger = charger
        self.scheduler = scheduler
        self.running = False
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        # 启动数据采集线程
        self.running = True
        self.thread.start()
        print("[StatusReporter] 🟢 Started")

    def stop(self):
        # 停止线程
        self.running = False

    def _loop(self):
        # 每秒采集一次状态并发送给上位机
        while self.running:
            try:
                self.surface.send_status(self._collect_status())
            except Exception as e:
                print(f"[StatusReporter] ❌ Collect error: {e}")
            time.sleep(0.1)

    def _collect_status(self):
        status = {}
        if self.pixhawk:
            attitude = _safe_call(self.pixhawk, "get_attitude", {})
            status['rov_attitude'] = attitude
            status['rov_telemetry'] = {
                "flight_mode": _safe_call(self.pixhawk, "get_flight_mode", getattr(self.pixhawk, "current_mode", None)),
                "armed": _safe_call(self.pixhawk, "is_armed", False),
                "attitude": attitude,
                "local_velocity": _safe_call(self.pixhawk, "get_velocity", {}),
                "servo_outputs": _safe_call(self.pixhawk, "get_servo_outputs", {}),
            }
        if self.fish:
            fish_attitude = self.fish.get_status("fish_attitude", {})
            status['fish_status'] = fish_attitude
        if self.charger:
            status['charger_status'] = self.charger.get_status()
        if self.scheduler:
            status['task_status'] = self.scheduler.get_system_status()
        return status


def _safe_call(obj, method_name, default):
    method = getattr(obj, method_name, None)
    if method is None:
        return default
    try:
        return method()
    except Exception:
        return default
