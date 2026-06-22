import threading
import time
from modules.comms.surface_comm import SurfaceComm

class StatusReporter:
    def __init__(self, surface: SurfaceComm, pixhawk=None, fish=None, charger=None):
        # 初始化状态数据来源与通信接口
        self.surface = surface
        self.pixhawk = pixhawk
        self.fish = fish
        self.charger = charger
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
            status = {}
            try:
                if self.pixhawk:
                    status['rov_attitude'] = self.pixhawk.get_attitude()
                if self.fish:
                    fish_attitude = self.fish.get_status("fish_attitude", {})
                    status['fish_status'] = fish_attitude
                if self.charger:
                    status['charger_status'] = self.charger.get_status()

                self.surface.send_status(status)
            except Exception as e:
                print(f"[StatusReporter] ❌ Collect error: {e}")
            time.sleep(0.1)
