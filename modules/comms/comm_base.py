# modules/comm/comm_base.py

import threading
import queue
import time
import logging

class CommunicationBase:
    """通信基类，定义统一接口"""
    
    def __init__(self, name):
        self.name = name
        self.logger = logging.getLogger(f"Comm_{name}")
        self.input_queue = queue.Queue()  # 接收数据队列
        self.output_queue = queue.Queue()  # 发送数据队列
        self.running = False
        self.thread = None
        self.last_heartbeat = time.time()
        self.timeout = 3.0  # 心跳超时时间 (秒)
        
    def start(self):
        """启动通信线程"""
        if self.running:
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._run_loop)
        self.thread.daemon = True
        self.thread.start()
        self.logger.info(f"{self.name}通信启动1")
        
    def stop(self):
        """停止通信线程"""
        if not self.running:
            return
            
        self.running = False
        self.thread.join(timeout=2.0)
        if self.thread.is_alive():
            self.logger.warning(f"{self.name}通信线程未正常退出")
        else:
            self.logger.info(f"{self.name}通信停止")
        self.thread = None
        
    def put_command(self, command):
        """向设备发送命令"""
        if not self.running:
            self.logger.warning(f"尝试发送命令但{self.name}未启动")
            return False
        self.output_queue.put(command)
        return True
        
    def get_data(self):
        """获取设备数据"""
        if not self.running:
            return None
        try:
            return self.input_queue.get_nowait()
        except queue.Empty:
            return None
            
    def is_connected(self):
        """检查是否保持连接"""
        if not self.running:
            return False
        return time.time() - self.last_heartbeat < self.timeout
        
    def send_heartbeat(self):
        """发送心跳信号，证明连接正常"""
        self.last_heartbeat = time.time()
        
    def _run_loop(self):
        """通信线程主循环，子类必须实现此方法"""
        raise NotImplementedError("子类必须实现_run_loop方法")
        
    def __del__(self):
        self.stop()