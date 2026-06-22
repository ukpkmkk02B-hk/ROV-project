import serial
import threading
import time
import yaml
import json

class ChargingComm:
    def __init__(self, cfg):
        self.port = cfg.get("port", "/dev/ttyUSB0")
        self.baudrate = cfg.get("baudrate", 115200)
        self.running = False
        self.status_handler = None
        self._status_dict = {}  # 私有字典保存状态参数[4,5](@ref)

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"[ChargingComm] ✅ 串口连接成功: {self.port}")
        except Exception as e:
            print(f"[ChargingComm] ❌ 串口连接失败: {e}")
            self.ser = None

        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.daemon = True
        self.thread.start()

    def start_charging(self):
        self.send_command("03")

    def stop_charging(self):
        self.send_command("04")
        
    def send_release_command(self):
        self.send_command("01")
        
    def send_connect_command(self):
        self.send_command("02")

    def send_command(self, cmd: str):
        if self.ser and self.ser.is_open:
            message = cmd.strip().upper()
            self.ser.write(message.encode("utf-8"))
            print(f"[ChargingComm] 📤 Sent: {message.strip()}")
        else:
            print("[ChargingComm] ⚠️ 串口未连接，无法发送指令")

    def register_status_handler(self, handler):
        self.status_handler = handler
        if not self.thread.is_alive():
            self.running = True
            self.thread.start()

    def _listen_loop(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8").strip()
                    if line:
                        status = json.loads(line)  # 假设接收JSON数据
                        self._status_dict.update(status)  # 更新状态字典[2](@ref)
                        print(f"[ChargingComm] 📥 Received: {line}")
                        if self.status_handler:
                            self.status_handler(line)
            except Exception as e:
                print(f"[ChargingComm] ❌ 读取错误: {e}")
            time.sleep(1)

    def close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[ChargingComm] ✅ 串口已关闭")

    def get_status(self) -> dict:
        # 返回最新的状态字典，如果没有则返回空字典
        return dict(self._status_dict)
# 测试
if __name__ == "__main__":
    with open("config/settings.yaml") as f:
        cfg = yaml.safe_load(f)
        charging = ChargingComm(cfg["charging"])
        charging.send_command("CRUISE")