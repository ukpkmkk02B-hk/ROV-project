# modules/controller/precision_docking.py
import math

class PrecisionDockingController:
    """
    精准对接控制器（近距离 10cm 内）
    目标：同时对齐位置 + 姿态
    """
    def __init__(self):
        # 增益
        self.kp_xy = 0.6
        self.kp_z = 0.5
        self.kp_yaw = 0.8   # 控制偏航对准

        # 最大速度限制
        self.max_v = 0.2
        self.max_yaw_rate = 0.3  # rad/s

    def compute_control(self, pose):
        """
        输入: pose = {"x","y","z","yaw"} (单位: m, rad)
        输出: {"vx","vy","vz","yaw_rate"}
        """
        x, y, z, yaw = pose["x"], pose["y"], pose["z"], pose["yaw"]

        # 比例控制
        vx = -self.kp_xy * x
        vy = -self.kp_xy * y
        vz = -self.kp_z * z
        yaw_rate = -self.kp_yaw * yaw

        # 饱和限制
        vx = max(-self.max_v, min(self.max_v, vx))
        vy = max(-self.max_v, min(self.max_v, vy))
        vz = max(-self.max_v, min(self.max_v, vz))
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))

        return {"vx": vx, "vy": vy, "vz": vz, "yaw_rate": yaw_rate}
