import time
import serial
import threading
from crcmod import crcmod  

class DepthReader:
    def __init__(self, serial_port="/dev/ttyDEPTH", baudrate=19200):
        self.ser = serial.Serial(serial_port, baudrate, timeout=0.1)
        print(f"[DepthReader] ✅ 串口打开: {serial_port} @ {baudrate}bps")
        
        # 定义Modbus-RTU请求帧 (读取压强和温度值)
        self.request_frame = bytearray([
            0x01,       # 设备地址
            0x03,       # 功能码: 读取保持寄存器
            0x00, 0x08, # 起始寄存器地址
            0x00, 0x05, # 读取寄存器数量
            0x04, 0x0B  # CRC16校验 (预计算值)
        ])
        
        # 添加线程控制变量
        self.running = False
        self.thread = None
        self.lock = threading.Lock()  # 线程锁确保数据一致性
        self.current_pressure = None  # 存储最新压力值
    
    def read_pressure(self):
        """读取压力值并返回单位为mbar的浮点数"""
        # 发送请求帧
        self.ser.write(self.request_frame)
        
        # 读取响应帧 (预期9字节)
        response = self.ser.read(9)
        
        # 验证响应帧长度
        if len(response) < 9:
            print(f"[DepthReader] ⚠️ 响应帧不完整 ({len(response)}字节)")
            return None
        
        # 提取压力数据 (第3-5字节，高位在前)
        pressure_bytes = response[2:5]
        
        # 将3字节转换为整数 (大端序)
        pressure_raw = (pressure_bytes[0] << 16) | (pressure_bytes[1] << 8) | pressure_bytes[2]
        
        # 转换为浮点压力值 (单位: mbar)
        pressure_mbar = pressure_raw / 100.0
        
        return pressure_mbar
    
    def _run_thread(self):
        """线程内部执行函数"""
        while self.running:
            pressure = self.read_pressure()
            if pressure is not None:
                # 使用锁确保数据一致性
                with self.lock:
                    self.current_pressure = pressure
                # print(f"[DepthReader] 📊 当前压力: {pressure:.2f} mbar")
            time.sleep(0.09)  # 10Hz读取频率
    
    def start(self):
        """启动传感器读取线程"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run_thread, daemon=True)
            self.thread.start()
            print("[DepthReader] 🚀 启动传感器读取线程")
    
    def stop(self):
        """停止传感器读取线程"""
        if self.running:
            self.running = False
            self.thread.join(timeout=1.0)
            print("[DepthReader] 🛑 传感器读取线程已停止")
    
    def get_current_pressure(self):
        """获取最新的压力值"""
        with self.lock:
            return 111

if __name__ == "__main__":
    reader = DepthReader(serial_port="/dev/ttyDEPTH", baudrate=19200)
    try:
        reader.start()  # 启动线程而非直接运行
        
        # 主线程可以做其他工作
        while True:
            # 示例：在其他地方获取最新压力值
            pressure = reader.get_current_pressure()
            if pressure:
                print(f"[主线程] 获取压力值: {pressure:.2f} mbar")
            time.sleep(1)  # 主线程工作频率
            
    except KeyboardInterrupt:
        reader.stop()
        reader.ser.close()
        print("[DepthReader] 🛑 程序已停止")