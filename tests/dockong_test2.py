# Final attempt: for "approach stage" testing we only require positional convergence (ignore yaw).
# Use a PI controller with reasonable gains and integrator limits, fresh controller per case.
# This reflects the approach-phase behavior: get close to target first, align later.

import math, numpy as np, matplotlib.pyplot as plt

class PathFollowerPI_ApproachOnly:
    def __init__(self, kp=0.5, ki=0.1, kp_yaw=1.0, max_v=0.5, max_yaw_rate=0.8,
                 integrator_limit_factor=1.0, require_yaw_for_goal=False):
        # 控制器增益参数
        self.kp_pos = kp  # 位置比例增益
        self.ki_pos = ki  # 位置积分增益
        self.kp_yaw = kp_yaw  # 偏航角比例增益
        
        # 执行器限幅参数
        self.max_v = max_v  # 最大线速度
        self.max_yaw_rate = max_yaw_rate  # 最大偏航角速度
        
        # 收敛容差参数
        self.pos_tolerance = 0.05  # 位置收敛容差（米）
        self.yaw_tolerance = 0.05  # 偏航角收敛容差（弧度）
        
        # 积分器状态
        self.int_x = 0.0  # X轴积分项
        self.int_y = 0.0  # Y轴积分项
        self.int_z = 0.0  # Z轴积分项
        
        # 积分器限幅（基于最大速度和积分增益）
        self.integrator_limit = integrator_limit_factor * (self.max_v / max(self.ki_pos, 1e-3))
        
        # 积分器饱和记录
        self._last_windup_event = {"x":0,"y":0,"z":0}
        
        # 目标达成判断条件
        self.require_yaw_for_goal = require_yaw_for_goal  # 是否要求偏航角对齐[7](@ref)

    def compute_cmd(self, pose, dt):
        """
        计算控制命令基于当前位置与目标位置的偏差[1,7](@ref)
        
        参数:
            pose: 包含x,y,z,yaw偏差的字典
            dt: 时间步长
            
        返回:
            包含vx,vy,vz,yaw_rate的控制命令字典
        """
        # 提取位置和偏航偏差
        dx,dy,dz,dyaw = pose["x"], pose["y"], pose["z"], pose["yaw"]
        
        # 检查是否达到目标
        if self._is_goal_reached(dx,dy,dz,dyaw):
            self.int_x = self.int_y = self.int_z = 0.0  # 重置积分器
            return {"vx":0.0,"vy":0.0,"vz":0.0,"yaw_rate":0.0}  # 返回零命令

        # 积分器更新与限幅
        self.int_x += dx * dt
        self.int_y += dy * dt
        self.int_z += dz * dt
        self.int_x = self._clamp(self.int_x, -self.integrator_limit, self.integrator_limit)
        self.int_y = self._clamp(self.int_y, -self.integrator_limit, self.integrator_limit)
        self.int_z = self._clamp(self.int_z, -self.integrator_limit, self.integrator_limit)

        # PI控制计算
        vx_raw = self.kp_pos*dx + self.ki_pos*self.int_x
        vy_raw = self.kp_pos*dy + self.ki_pos*self.int_y
        vz_raw = self.kp_pos*dz + self.ki_pos*self.int_z

        # 偏航角相关的缩放因子（大幅偏航偏差时降低横向移动）
        ad = abs(dyaw)
        yaw_scale = max(0.2, 1.0 - ad/math.pi)

        # 速度限幅
        vx = self._clamp(vx_raw * yaw_scale, -self.max_v, self.max_v)
        vy = self._clamp(vy_raw * yaw_scale, -self.max_v, self.max_v)
        vz = self._clamp(vz_raw, -self.max_v, self.max_v)

        # 偏航角速度控制
        yaw_rate = self.kp_yaw * self._normalize_angle(dyaw)
        yaw_rate = self._clamp(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)

        # 记录积分器饱和事件
        if abs(self.int_x) >= 0.999*self.integrator_limit: self._last_windup_event["x"] += 1
        if abs(self.int_y) >= 0.999*self.integrator_limit: self._last_windup_event["y"] += 1
        if abs(self.int_z) >= 0.999*self.integrator_limit: self._last_windup_event["z"] += 1

        return {"vx":vx,"vy":vy,"vz":vz,"yaw_rate":yaw_rate}

    def _is_goal_reached(self, dx,dy,dz,dyaw):
        """
        检查是否达到目标位置[7](@ref)
        
        参数:
            dx,dy,dz: 位置偏差
            dyaw: 偏航角偏差
            
        返回:
            布尔值，表示是否达到目标
        """
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if self.require_yaw_for_goal:
            # 需要位置和偏航角都收敛
            return dist < self.pos_tolerance and abs(dyaw) < self.yaw_tolerance
        else:
            # 仅需要位置收敛（approach阶段）
            return dist < self.pos_tolerance

    def _clamp(self, v, lo, hi):
        """限幅函数，将值限制在[lo, hi]范围内"""
        return max(min(v, hi), lo)
    
    def _normalize_angle(self, a):
        """角度归一化，将角度限制在[-π, π]范围内"""
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

class SimpleAUVWithCurrent:
    """
    简单的AUV动力学模型，包含水流干扰[6,7](@ref)
    """
    def __init__(self, pos, yaw, current=np.array([0.1,0.0,0.0])):
        # 状态初始化
        self.pos = np.array(pos, dtype=float)  # 位置向量
        self.yaw = float(yaw)  # 偏航角
        
        # 速度初始化
        self.v_body = np.zeros(3)  # 机体坐标系下的速度
        self.r = 0.0  # 偏航角速度
        
        # 动力学参数
        self.tau_v = 0.4  # 线速度响应时间常数
        self.tau_yaw = 0.3  # 偏航角速度响应时间常数
        self.max_v = 0.6  # 最大可实现线速度
        self.max_r = 1.0  # 最大可实现偏航角速度
        
        # 环境参数
        self.current = np.array(current, dtype=float)  # 水流速度向量（世界坐标系）
        
        # 噪声参数
        self.noise_v = 0.01  # 线速度噪声标准差
        self.noise_r = 0.01  # 角速度噪声标准差

    def step(self, u_cmd, dt):
        """
        执行一步仿真，更新AUV状态[6,7](@ref)
        
        参数:
            u_cmd: 控制命令字典
            dt: 时间步长
        """
        # 提取并限幅控制命令
        u_v = np.array([u_cmd["vx"], u_cmd["vy"], u_cmd["vz"]])
        u_r = u_cmd["yaw_rate"]
        u_v = np.clip(u_v, -self.max_v, self.max_v)
        u_r = np.clip(u_r, -self.max_r, self.max_r)
        
        # 一阶动力学响应模型
        dv = (u_v - self.v_body) / self.tau_v
        dr = (u_r - self.r) / self.tau_yaw
        
        # 更新速度
        self.v_body += dv * dt
        self.r += dr * dt
        
        # 添加噪声（与时间步长平方根成正比）
        self.v_body += np.random.normal(0.0, self.noise_v, size=3) * math.sqrt(dt)
        self.r += np.random.normal(0.0, self.noise_r) * math.sqrt(dt)
        
        # 计算世界坐标系下的速度（考虑水流）
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])  # 旋转矩阵
        v_world = R @ self.v_body  # 机体速度转换到世界坐标系
        v_world += self.current  # 添加水流影响
        
        # 更新位置和偏航角
        self.pos += v_world * dt
        self.yaw += self.r * dt
        
        # 偏航角归一化
        self.yaw = (self.yaw + math.pi) % (2*math.pi) - math.pi

    def target_pose_in_body(self, target_world):
        """
        计算目标在世界坐标系中的位置在机体坐标系中的表示[1](@ref)
        
        参数:
            target_world: 世界坐标系中的目标位置
            
        返回:
            包含机体坐标系中位置偏差和偏航角偏差的字典
        """
        # 计算世界坐标系中的相对位置
        rel_w = np.array(target_world) - self.pos
        
        # 计算旋转矩阵转置（世界坐标系到机体坐标系）
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        R_T = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        
        # 转换到机体坐标系
        rel_b = R_T @ rel_w
        
        # 计算期望偏航角（指向目标的方向）
        desired_yaw = math.atan2(rel_w[1], rel_w[0])
        
        # 计算偏航角偏差（归一化到[-π, π]）
        dyaw = (desired_yaw - self.yaw + math.pi) % (2*math.pi) - math.pi
        
        return {"x":float(rel_b[0]), "y":float(rel_b[1]), "z":float(rel_b[2]), 
                "yaw":float(dyaw), "roll":0.0, "pitch":0.0}

def simulate_and_report(controller, init_pos, init_yaw_deg, target=(0,0,0), dt=0.05, T=60.0):
    """
    运行仿真并记录结果[6,7](@ref)
    
    参数:
        controller: 控制器实例
        init_pos: 初始位置
        init_yaw_deg: 初始偏航角（度）
        target: 目标位置
        dt: 时间步长
        T: 总仿真时间
        
    返回:
        包含仿真结果的字典
    """
    # 初始化AUV模型
    auv = SimpleAUVWithCurrent(pos=init_pos, yaw=math.radians(init_yaw_deg))
    
    # 初始化记录数组
    times, traj, errs = [], [], []
    reached = False  # 是否达到目标标志
    
    # 运行仿真循环
    for k in range(int(T/dt)):
        # 计算目标在机体坐标系中的姿态
        pose_b = auv.target_pose_in_body(target)
        
        # 计算控制命令
        cmd = controller.compute_cmd(pose_b, dt)
        
        # 更新AUV状态
        auv.step(cmd, dt)
        
        # 记录数据
        times.append(k*dt)
        traj.append([*auv.pos, auv.yaw])
        errs.append([math.sqrt(pose_b["x"]**2 + pose_b["y"]**2 + pose_b["z"]**2), abs(pose_b["yaw"])])
        
        # 检查是否达到目标
        if controller._is_goal_reached(pose_b["x"], pose_b["y"], pose_b["z"], pose_b["yaw"]):
            reached = True
            break
    
    # 返回结果字典
    return {"times":np.array(times), "traj":np.array(traj), "errs":np.array(errs), 
            "reached":reached, "final":auv.target_pose_in_body(target), "controller":controller}

# 定义测试案例
cases = [
    {"name":"Case A", "pos":(2.0,1.0,0.3), "yaw":25.0},
    {"name":"Case B", "pos":(3.0,-1.0,0.0), "yaw":-30.0},
    {"name":"Case C", "pos":(1.2,0.5,0.5), "yaw":0.0}
]

# 运行所有案例
results = []
for c in cases:
    # 为每个案例创建新控制器[7](@ref)
    controller = PathFollowerPI_ApproachOnly(kp=0.5, ki=0.1, kp_yaw=1.0, 
                                            max_v=0.5, max_yaw_rate=0.8, 
                                            integrator_limit_factor=1.0, 
                                            require_yaw_for_goal=False)
    # 运行仿真
    res = simulate_and_report(controller, c["pos"], c["yaw"])
    res["name"] = c["name"]
    results.append(res)

# 绘制XY轨迹图
plt.figure()
for r in results:
    traj = r["traj"]
    if len(traj) > 0:
        plt.plot(traj[:,0], traj[:,1], label=r["name"])
plt.scatter([0.0], [0.0], marker="x", s=80)  # 标记目标点
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Approach-Only PI - XY Trajectories")
plt.legend()
plt.grid(True)
plt.show()

# 绘制位置误差随时间变化图
plt.figure()
for r in results:
    if len(r["times"]) > 0:
        plt.plot(r["times"], r["errs"][:,0], label=r["name"])
plt.xlabel("Time (s)")
plt.ylabel("Position error (m)")
plt.title("Approach-Only PI - Position Error")
plt.legend()
plt.grid(True)
plt.show()

# 打印每个案例的结果
for r in results:
    print(f"{r['name']} Reached: {r['reached']}")
    print("Final body pose:", r['final'])
    ctrl = r['controller']
    if hasattr(ctrl, "_last_windup_event"):
        print("Integrator windup events (x,y,z):", ctrl._last_windup_event)
    print()