# Phase 1: P-controller convergence test (no camera), with a simple vehicle model
# - Vehicle receives commanded body-frame velocities (vx, vy, vz, yaw_rate)
# - Achieved velocities follow first-order lag: dv/dt = (u_cmd - v)/tau, with saturation
# - World <-> body rotation by yaw
# - Target located at origin in world; vehicle starts at different initial conditions
# - Controller: your PathFollower (as provided), using target pose in the vehicle/body frame
# - Outputs: trajectory plot (x-y), error-norm plot over time, final status

import math
import numpy as np
import matplotlib.pyplot as plt

# ---- Your PathFollower controller (as given) ----
class PathFollower:
    def __init__(self):
        # 速度控制参数
        self.kp_pos = 0.5       # 位置比例增益
        self.kp_yaw = 1.0       # 偏航比例增益

        # 限幅参数
        self.max_v = 0.5        # 最大速度 m/s
        self.max_yaw_rate = 0.8 # 最大偏航角速度 rad/s

        # 靶点容差
        self.pos_tolerance = 0.05  # 5cm
        self.yaw_tolerance = 0.05  # 约3度

    def compute_cmd(self, pose):
        """
        输入相对目标 pose，输出控制指令 cmd
        pose: dict with keys: x, y, z, yaw, pitch, roll
        return: dict with keys: vx, vy, vz, yaw_rate
        """

        # 提取相对位姿
        dx = pose.get("x", 0.0)
        dy = pose.get("y", 0.0)
        dz = pose.get("z", 0.0)
        dyaw = pose.get("yaw", 0.0)

        # 判断是否接近目标
        if self._is_goal_reached(dx, dy, dz, dyaw):
            return {"vx": 0.0, "vy": 0.0, "vz": 0.0, "roll_rate": 0.0,"pitch_rate": 0.0,"yaw_rate": 0.0}

        # 简单 P 控制
        vx = self.kp_pos * dx
        vy = self.kp_pos * dy
        vz = self.kp_pos * dz
        yaw_rate = self.kp_yaw * self._normalize_angle(dyaw)

        # 限幅
        vx = self._clamp(vx, -self.max_v, self.max_v)
        vy = self._clamp(vy, -self.max_v, self.max_v)
        vz = self._clamp(vz, -self.max_v, self.max_v)
        yaw_rate = self._clamp(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)

        return {
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "roll_rate": 0.0,
            "pitch_rate": 0.0,
            "yaw_rate": yaw_rate
        }

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

# ---- Simple vehicle + environment model ----
class SimpleAUV:
    """
    Body-frame velocity dynamics with first-order lag.
    State in world frame: position (x,y,z), yaw (psi).
    """
    def __init__(self, pos, yaw, tau_v=0.4, tau_yaw=0.3, max_v=0.6, max_yaw_rate=1.0):
        self.pos = np.array(pos, dtype=float)  # [x, y, z] in world
        self.yaw = float(yaw)                  # radians

        # actual body-frame velocities (what the vehicle achieves), start at zero
        self.v_body = np.zeros(3)              # [vx, vy, vz]
        self.r = 0.0                           # yaw rate achieved

        # first-order lag constants
        self.tau_v = tau_v
        self.tau_yaw = tau_yaw

        # actuation limits (vehicle capability)
        self.max_v = max_v
        self.max_yaw_rate = max_yaw_rate

        # small process noise to mimic reality
        self.process_noise_std_v = 0.01  # m/s
        self.process_noise_std_r = 0.01  # rad/s

    def step(self, u_cmd, dt):
        """
        u_cmd: dict with 'vx','vy','vz','yaw_rate' (body frame)
        Update internal velocities with first-order lag and then propagate pose in world frame.
        """
        u_v = np.array([u_cmd["vx"], u_cmd["vy"], u_cmd["vz"]], dtype=float)
        u_r = float(u_cmd["yaw_rate"])

        # saturate commands to vehicle capability
        u_v = np.clip(u_v, -self.max_v, self.max_v)
        u_r = np.clip(u_r, -self.max_yaw_rate, self.max_yaw_rate)

        # first-order lag dynamics for achieved velocities
        dv = (u_v - self.v_body) / self.tau_v
        dr = (u_r - self.r) / self.tau_yaw

        # integrate with simple Euler
        self.v_body += dv * dt
        self.r += dr * dt

        # add small process noise
        self.v_body += np.random.normal(0.0, self.process_noise_std_v, size=3) * math.sqrt(dt)
        self.r += np.random.normal(0.0, self.process_noise_std_r) * math.sqrt(dt)

        # world-frame velocity by rotating body velocities
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        R = np.array([[c, -s, 0.0],
                      [s,  c, 0.0],
                      [0.0, 0.0, 1.0]])
        v_world = R @ self.v_body

        # propagate state
        self.pos += v_world * dt
        self.yaw += self.r * dt
        self.yaw = (self.yaw + math.pi) % (2*math.pi) - math.pi  # normalize

    def target_pose_in_body(self, target_world):
        """
        Return target pose relative to vehicle in body frame:
        (x_b, y_b, z_b) and yaw error (target assumed yaw=0).
        """
        # relative position in world
        rel_w = np.array(target_world) - self.pos

        # rotate into body frame
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        R_T = np.array([[ c, s, 0.0],
                        [-s, c, 0.0],
                        [0.0, 0.0, 1.0]])
        rel_b = R_T @ rel_w

        # desired yaw is to face the target: yaw error is angle of rel_w in world - current yaw
        desired_yaw = math.atan2(rel_w[1], rel_w[0])
        dyaw = (desired_yaw - self.yaw + math.pi) % (2*math.pi) - math.pi

        return {
            "x": float(rel_b[0]),
            "y": float(rel_b[1]),
            "z": float(rel_b[2]),
            "yaw": float(dyaw),
            "roll": 0.0,
            "pitch": 0.0
        }

def simulate(controller, init_pos=(2.0, 1.0, 0.3), init_yaw_deg=25.0,
             target=(0.0, 0.0, 0.0), dt=0.05, T=60.0,
             stop_on_reach=True):
    auv = SimpleAUV(pos=init_pos, yaw=math.radians(init_yaw_deg))
    target_w = np.array(target, dtype=float)

    times = []
    traj = []
    errs = []
    reached = False

    num_steps = int(T / dt)
    for k in range(num_steps):
        t = k * dt
        pose_b = auv.target_pose_in_body(target_w)
        cmd = controller.compute_cmd(pose_b)

        auv.step(cmd, dt)

        # record
        times.append(t)
        traj.append([*auv.pos, auv.yaw])
        err_pos = math.sqrt(pose_b["x"]**2 + pose_b["y"]**2 + pose_b["z"]**2)
        errs.append([err_pos, abs(pose_b["yaw"])])

        # goal check (use controller's own tolerance)
        if stop_on_reach and controller._is_goal_reached(pose_b["x"], pose_b["y"], pose_b["z"], pose_b["yaw"]):
            reached = True
            break

    return {
        "times": np.array(times),
        "traj": np.array(traj),          # [x,y,z,yaw]
        "errs": np.array(errs),          # [pos_err, yaw_err]
        "reached": reached,
        "final_pose_body": auv.target_pose_in_body(target_w)
    }

# ---- run a few initial conditions
controller = PathFollower()

cases = [
    {"name": "Case A: (2,1,0.3), yaw=25°", "pos": (2.0, 1.0, 0.3), "yaw_deg": 25.0},
    {"name": "Case B: (3,-1,0.0), yaw=-30°", "pos": (3.0, -1.0, 0.0), "yaw_deg": -30.0},
    {"name": "Case C: (1.2,0.5,0.5), yaw=0°", "pos": (1.2, 0.5, 0.5), "yaw_deg": 0.0},
]

results = []
for case in cases:
    res = simulate(controller, init_pos=case["pos"], init_yaw_deg=case["yaw_deg"])
    res["name"] = case["name"]
    results.append(res)

# ---- Plot 1: X-Y trajectories
plt.figure()
for res in results:
    traj = res["traj"]
    if len(traj) == 0:
        continue
    x = traj[:,0]
    y = traj[:,1]
    plt.plot(x, y, label=res["name"])
plt.scatter([0.0],[0.0], marker="x", s=80)  # target at origin
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("P-Controller Approach: XY Trajectories (Target at 0,0)")
plt.legend()
plt.grid(True)
plt.show()

# ---- Plot 2: Position error over time for each case
plt.figure()
for res in results:
    times = res["times"]
    errs = res["errs"]
    if len(times) == 0:
        continue
    plt.plot(times, errs[:,0], label=res["name"])
plt.xlabel("Time (s)")
plt.ylabel("Position Error (m)")
plt.title("Position Error vs Time")
plt.legend()
plt.grid(True)
plt.show()

# ---- Print summary
for res in results:
    print(res["name"])
    print("  Reached:", res["reached"])
    print("  Final body-frame pose:", res["final_pose_body"])
    print()
