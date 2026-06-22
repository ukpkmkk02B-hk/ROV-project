# modules/controller/path_follower.py
import math

class PathFollower:
    """
    粗对接控制器（远距离接近阶段）
    基于PI速度环，确保 ROV 向目标点逐渐收敛
    """
    def __init__(self):
        # 位置 → 速度映射增益
        self.kp_xy = 0.4   # 水平比例增益
        self.kp_z = 0.3    # 垂直比例增益
        self.ki = 0.05     # 积分增益，抵消水流干扰

        # 积分项
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0

        # 最大速度限制（m/s）
        self.max_v = 0.4

    def reset(self):
        """重置积分项"""
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0

    def compute_cmd(self, pose):
        """
        输入: pose = {"x","y","z","yaw",...} (相机坐标系下, 单位: m, rad)
        输出: 速度控制指令 {"vx","vy","vz","yaw_rate"}
        """
        x, y, z = pose["x"], pose["y"], pose["z"]

        # 积分项更新
        self.integral_x += x
        self.integral_y += y
        self.integral_z += z

        # PI 控制
        vx = -(self.kp_xy * x + self.ki * self.integral_x)
        vy = -(self.kp_xy * y + self.ki * self.integral_y)
        vz = -(self.kp_z * z + self.ki * self.integral_z)

        # 饱和限制
        vx = max(-self.max_v, min(self.max_v, vx))
        vy = max(-self.max_v, min(self.max_v, vy))
        vz = max(-self.max_v, min(self.max_v, vz))

        # 不控制 yaw，只保持旋转缓慢搜索
        return {"vx": vx, "vy": vy, "vz": vz, "yaw_rate": 0.0}


    def _is_goal_reached(self, dx, dy, dz, dyaw):
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist < self.pos_tolerance and abs(dyaw) < self.yaw_tolerance:
            return True
        return False

    def _clamp(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def _normalize_angle(self, angle):
        """将角度限制在 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
