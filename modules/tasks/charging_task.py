# tasks/charging_task.py
import time
import logging

class ChargingTask:
    def __init__(self, charging_comm, fish_comm, state_machine=None):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.charging_comm = charging_comm
        self.fish_comm = fish_comm
        self.state_machine = state_machine
        
        self.status = "idle"
        self.start_time = None
        self.max_duration = 60  # 最长充电时长（秒）
        self.battery_threshold = 0.95  # 设定充满电的电量阈值
        
    def start(self):
        self.logger.info("启动充电任务")
        self.status = "running"
        self.start_time = time.time()

        # 开始充电
        self.charging_comm.start_charging()
        
    def run(self):
        if self.status != "running":
            return
        self.charging_comm.start_charging()
        current_time = time.time()
        elapsed = current_time - self.start_time

        battery_level = self.fish_comm.get_battery_level()  # float: 0.0~1.0
        self.logger.info(f"当前电量: {battery_level:.2f}, 已充电: {elapsed:.1f}s")

        if battery_level >= self.battery_threshold or elapsed >= self.max_duration:
            self.logger.info("充电完成或超时，停止充电")
            self.charging_comm.stop_charging()
            self._release_mechanism()
            self.status = "completed"
            if self.state_machine:
                self.state_machine.notify_task_completed("charging")
                
    def stop(self):
        self.logger.info("停止充电任务")
        self.charging_comm.send_stop_command()
        self.status = "stopped"
        if self.state_machine:
            self.state_machine.notify_task_stop("charging")
    
    def get_status(self):
        return {"status": self.status}
    
    def _release_mechanism(self):
        """控制释放机构"""
        try:
            self.logger.info("发送释放机构指令")
            self.charging_comm.send_release_command()
        except Exception as e:
            self.logger.error(f"释放对接机构失败: {e}")
