import serial
import threading
import struct
from modules.comms.surface_comm import SurfaceComm
import time 
class OpenMVReceiver:
    def __init__(self, config: dict, surface: SurfaceComm = None):
        # 初始化串口和通信接口
        port = config.get("port", "/dev/rfcomm0")
        baudrate = config.get("baudrate", 921600)
        self.surface = surface
        self.running = False
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.5)
            print(f"[OpenMVReceiver] ✅ 串口连接成功: {self.serial_port}")
        except Exception as e:
            print(f"[OpenMVReceiver] ❌ 串口连接失败: {e}")

    def start(self):
        # 启动读取线程
        self.running = True
        self.thread.start()
        print("[OpenMVReceiver] 🟢 Started")

    def stop(self):
        # 停止接收并关闭串口
        self.running = False
        if self.serial_port.is_open:
            self.serial_port.close()
    
    def send_command(self, cmd: str):
        """
        发送简单字符串指令给 OpenMV
        :param cmd: 指令字符串，例如 "START" 或 "STOP"
        """
        if self.serial_port.is_open:
            try:
                # 在指令末尾加换行符，便于 OpenMV 接收
                self.serial_port.write((cmd + "\n").encode("utf-8"))
                print(f"[OpenMVReceiver] 📤 已发送指令: {cmd}")
            except Exception as e:
                print(f"[OpenMVReceiver] ❌ 发送指令失败: {e}")
        else:
            print("[OpenMVReceiver] ❗ 串口未打开，无法发送指令")

    # def _read_loop(self):
    #     buffer = bytearray()

    #     while self.running:
    #         try:
    #             data = self.serial_port.read_all()
    #             if data:
    #                 print(f"[OpenMVReceiver] 📤 已发送指令: ")
    #                 buffer.extend(data)
                    
    #                 while len(buffer) >= 6:
    #                     main_type = buffer[0]
    #                     sub_type = buffer[1]
    #                     length = struct.unpack(">I", buffer[2:6])[0]

    #                     if len(buffer) < 6 + length:
    #                         break  # 等待完整帧

    #                     frame_data = buffer[6:6+length]
    #                     buffer = buffer[6+length:]  # 剩余数据

    #                     print(f"[OpenMVReceiver] ✅ 接收到视频帧 类型:{main_type} 子类型:{sub_type} 长度:{length} 字节")
    #                     if self.surface:
    #                         self.surface.send_video_packet(frame_dat/a, main_type, sub_type)
    #                     else:
    #                         print("[OpenMVReceiver] ❗ 未注册 surface，丢弃帧")
    #             else:
    #                 time.sleep(0.01)
    #         except Exception as e:
    #             print(f"[OpenMVReceiver] ❌ Error: {e}")
    #             time.sleep(0.5)

    def _read_loop(self):
        buffer = bytearray()
        MAX_FRAME_SIZE = 50000  # 最大帧大小，防止解析错误

        while self.running:
            try:
                data = self.serial_port.read(1024)
                if data:
                    buffer.extend(data)

                    while len(buffer) >= 8:
                        # 查找帧同步标识 0xAA55
                        if buffer[0] != 0xAA or buffer[1] != 0x55:
                            buffer.pop(0)
                            continue

                        main_type = buffer[2]
                        sub_type = buffer[3]
                        length = struct.unpack(">I", buffer[4:8])[0]

                        if length > MAX_FRAME_SIZE:
                            print(f"[WARN] 帧长度异常 {length}, 丢弃 buffer")
                            buffer = bytearray()
                            break

                        if len(buffer) < 8 + length:
                            print(f"[DEBUG] 等待完整帧, 已有 {len(buffer)} 字节, 需要 {8+length}")
                            break

                        file_data = buffer[8:8+length]
                        buffer = buffer[8+length:]

                        if main_type == 0x01 and sub_type == 0x01:
                            if self.surface:
                                self.surface.send_video_packet(file_data, main_type, sub_type)
                                print(f"[DEBUG] 📤 发送视频帧完成, 大小: {length}")
                            else:
                                print("[DEBUG] ❗ 未注册 surface，丢弃帧")
                        else:
                            print(f"[DEBUG] ⚠️ 未知帧类型 main_type={main_type}, sub_type={sub_type}")

                else:
                    time.sleep(0.01)

            except Exception as e:
                print(f"[OpenMVReceiver] ❌ Error: {e}")
                time.sleep(0.5)


