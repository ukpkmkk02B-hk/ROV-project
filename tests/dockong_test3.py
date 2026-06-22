import time
import random
from modules.tasks.docking_task import DockingTask
from modules.perception.camera import AprilTagCameraInterface

# ==== Mock PixhawkComm (实验不用真飞控) ====
class MockPixhawk:
    def __init__(self):
        self._armed = False

    def is_armed(self):
        return self._armed

    def arm_vehicle(self):
        print("[MockPixhawk] ✅ 解锁")
        self._armed = True

    def set_mode(self, mode):
        print(f"[MockPixhawk] ✅ 模式切换到 {mode}")
        return True

    def send_velocity_command(self, cmd):
        print(f"[MockPixhawk] ⬆️ 发送速度指令: {cmd}")

    def stop(self):
        print("[MockPixhawk] ⏹️ 停止输出")

# ==== Mock StateMachine ====
class MockStateMachine:
    def notify_task_start(self, name):
        print(f"[MockSM] 🚀 任务 {name} 已开始")

    def notify_task_stop(self, name):
        print(f"[MockSM] ⏹️ 任务 {name} 已停止")

    def notify_task_completed(self, name):
        print(f"[MockSM] ✅ 任务 {name} 已完成")

    def notify_task_failed(self, name):
        print(f"[MockSM] ❌ 任务 {name} 失败")

    def start_task(self, name):
        print(f"[MockSM] 🔄 启动任务 {name}")

# ==== Mock Camera (实验不用真摄像头) ====
class MockCamera:
    def __init__(self):
        # 初始位置：远处，逐渐靠近
        self.step = 0

    def get_pose(self):
        # 模拟目标靠近的过程（含噪声 & 丢帧）
        if random.random() < 0.1:  # 10% 概率丢帧
            return None

        # 模拟逐渐接近 (x,y,z,yaw)
        x = max(0.5 - 0.01*self.step, 0.0) + random.uniform(-0.01, 0.01)
        y = max(0.5 - 0.01*self.step, 0.0) + random.uniform(-0.01, 0.01)
        z = max(0.5 - 0.005*self.step, 0.0) + random.uniform(-0.005, 0.005)
        yaw = max(0.3 - 0.005*self.step, 0.0) + random.uniform(-0.01, 0.01)

        self.step += 1
        return {
            "id": 1,
            "x": x,
            "y": y,
            "z": z,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": yaw,
        }

    def target_detected(self):
        # 模拟目标检测到（大部分时间能看到）
        return random.random() > 0.1

# ==== 实验脚本 ====
def run_experiment():
    pixhawk = MockPixhawk()
    sm = MockStateMachine()
    camera = MockCamera()

    task = DockingTask(camera=camera, pixhawk=pixhawk, state_machine=sm)
    task.start()

    for i in range(100):  # 最多跑 100 步
        task.run()
        status = task.get_status()
        print(f"[Step {i}] 阶段={status['stage']} 状态={status['status']} pose={status['last_pose']}")
        if status["stage"] in [DockingTask.STATE_DOCKED, DockingTask.STATE_FAILED]:
            break
        time.sleep(0.1)

    task.stop()

if __name__ == "__main__":
    run_experiment()
