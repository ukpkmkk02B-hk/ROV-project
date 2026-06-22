import time
import logging

class FishControlTask:
    """仿生鱼控制任务模块
    
    支持任务类型：
    - cruise: 巡航（带速度）
    - water_control: 控制吸排水电机位置
    - adjust_pitch: 调整俯仰（滑块控制）
    """

    def __init__(self, fish_comm, task_type="cruise", **kwargs):
        self.fish_comm = fish_comm             # 通信接口（必须具备 send_command() 和 get_status()）
        self.task_type = task_type             # 任务类型（cruise / water_control / adjust_pitch）
        print(self.task_type)
        self.status = "idle"                   # 当前任务状态
        self.logger = logging.getLogger(self.__class__.__name__)

        # 公共参数
        self.start_time = None                 # 任务启动时间
        self.timeout = kwargs.get("timeout", 20)  # 默认超时时间（秒）

        # 巡航任务参数
        self.speed = kwargs.get("speed", 0.3)          # 巡航速度
        self.duration = kwargs.get("duration", 10)     # 巡航时间                                                                                                                 

        # 吸排水参数（目标水量）                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             从ce
        self.target_water_level = kwargs.get("water_level", 50)

        # 俯仰调整（滑块目标位置）
        self.target_pitch_pos = kwargs.get("pitch_target", 85)

        # 内部完成标志
        self.done = False

    def start(self):
        """任务启动"""
        self.start_time = time.time()
        self.status = "running"
        self.logger.info(f"🐟 任务开始 [{self.task_type}]")

        # 根据任务类型发送启动指令
        if self.task_type == "cruise":
            self.fish_comm.send_command("D1\n")
            print("D1")
        elif self.task_type == "stop_control":
            self.fish_comm.send_command("D0")
        elif self.task_type == "water_control":
            self.fish_comm.send_command("D0") 
        elif self.task_type == "adjust_pitch":
            self.fish_comm.send_command(f"ADJUST {self.target_pitch_pos}")
        else:
            self.logger.error(f"❌ 不支持的任务类型: {self.task_type}")
            self.status = "failed"

    def run(self):
        """主循环：检查任务完成条件"""
        if self.status != "running":
            return
        # 检查是否超时
        elapsed = time.time() - self.start_time
        if elapsed > self.timeout:
            self.logger.warning(f"⏱️ 任务超时 [{self.task_type}]")
            self.status = "failed"
            return

        # 获取仿生鱼反馈状态
        state = self.fish_comm.get_status()

        # 判定是否完成
        if self.task_type == "cruise":
            # 持续运行，时间到即完成
            if elapsed >= self.duration:
                self.status = "completed"
                self.logger.info("✅ 巡航完成")

        elif self.task_type == "water_control":
            current = state.get("water_level", None)
            if current is not None and abs(current - self.target_water_level) < 3:
                self.status = "completed"
                self.logger.info(f"✅ 吸排水完成 当前: {current}")

        elif self.task_type == "adjust_pitch":
            current = state.get("center_pos", None)
            if current is not None and abs(current - self.target_pitch_pos) < 3:
                self.status = "completed"
                self.logger.info(f"✅ 俯仰调整完成 当前: {current}")

    def stop(self):
        """手动停止任务"""
        self.fish_comm.send_command("STOP")
        self.status = "stopped"
        self.logger.info("🛑 任务已手动停止")

    def get_status(self):
        """获取任务状态信息"""
        return {
            "status": self.status,
            "task_type": self.task_type,
            "elapsed_time": time.time() - self.start_time if self.start_time else 0
        }
