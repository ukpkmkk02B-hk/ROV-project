import serial
import threading
import time
import json
import yaml
import re  

class FishComm:
    def __init__(self, config):
        self.port = config["port"]
        self.baudrate = config.get("baudrate", 115200)
        self.timeout = config.get("timeout", 1)
        self.running = True
        self.status_handler = None
        self._status_dict = {}  # 用于保存所有最新状态参数

        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"[FishComm] ✅ Connected to fish on {self.port}")
        except serial.SerialException as e:
            print(f"[FishComm] ❌ Failed to open serial port: {e}")
            self.serial = None

        # 启动接收线程
        self.thread = threading.Thread(target=self._listen)
        self.thread.daemon = True
        self.thread.start()

    def send_command(self, cmd):
        """发送控制指令给小鱼，如 CRUISE, DOCK, STOP"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.write((cmd.strip().upper() + "\n").encode("utf-8"))
                print(f"[FishComm] 📤 Sent command: {cmd}")
            except Exception as e:
                print(f"[FishComm] ❌ Send error: {e}")

    def register_status_handler(self, handler):
        """注册状态信息处理器"""
        self.status_handler = handler
    import re


    def _listen(self):
        """监听小鱼返回的状态信息，只解析 fish_attitude"""
        buffer = ""
        while self.running and self.serial and self.serial.is_open:
            try:
                # decode 时忽略无法解析的字节
                chunk = self.serial.readline().decode("utf-8", errors='ignore')
                if not chunk:
                    continue
                buffer += chunk
                # 尝试提取 fish_attitude
                attitude = self._parse_fish_attitude(buffer)
                if attitude:
                    # 更新状态字典
                    self._status_dict["fish_attitude"] = attitude
                    
                    # 打印解析后的数据
                    print(f"[FishComm] 🐟 Parsed fish_attitude: {attitude}")
                    
                    # 调用回调
                    if self.status_handler:
                        self.status_handler({"fish_attitude": attitude})
                    
                    # 解析成功后清空缓冲
                    buffer = ""  
            except Exception as e:
                print(f"[FishComm] ❌ Serial read error: {e}")
            time.sleep(0.05)




    def _parse_fish_attitude(self, text: str):
        """
        从文本中提取 fish_attitude:{} 里的键值对
        """
        # 只匹配 fish_attitude 里的 {...}
        match = re.search(r'fish_attitude:\s*\{([^}]*)\}', text)
        if not match:
            return None

        content = match.group(1)  # {} 内的内容
        result = {}

        for item in content.split(','):
            key_value = item.split(':')
            if len(key_value) != 2:
                continue
            key = key_value[0].strip()
            value_str = key_value[1].strip()
            try:
                result[key] = float(value_str)
            except ValueError:
                continue  # 丢弃不是数字的值

        return result



   
    def get_battery_level(self) -> float:
        # 获取仿生鱼当前电量（0~1.0）
        return self.get_status("battery", 0.0)
    
    def get_status(self, key=None, default=None):
        """
        返回当前仿生鱼状态的某个字段，如果 key 为 None，则返回全部状态字典。
        """
        if key is None:
            return dict(self._status_dict)
        else:
            return self._status_dict.get(key, default)

    def close(self):
        self.running = False
        if self.serial:
            self.serial.close()
        print("[FishComm] ✅ Closed connection")
#  return {
#             "pitch": -2.3,
#             "roll": 1.1,
#             "yaw": 5.0,
#             "water_level": 62,     # 吸排水电机位置
#             "center_pos": 128,     # 滑块位置
#             "mode": "cruise"
#         }
# 测试
if __name__ == "__main__":
    with open("config/settings.yaml") as f:
        cfg = yaml.safe_load(f)
        fish = FishComm(cfg["fish"])
        fish.send_command("CRUISE")