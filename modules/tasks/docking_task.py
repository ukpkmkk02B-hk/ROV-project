# tasks/docking_task.py
# 添加目录到sys.path使得导入可以正常工作
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, project_root)

import time
import math
from modules.controller import path_follower, precision_docking
from modules.controller.motion_command import camera_state_to_body_error, motion_command_from_mapping
from modules.controller.rc_override_mapper import RcOverrideMapper
from modules.controller.visual_tracking_controller import VisualTrackingController
from modules.comms.pixhawk_comm import PixhawkComm
from modules.comms.comm_base import CommunicationBase
from modules.perception.camera import AprilTagCameraInterface
from modules.perception.marker_tracker import validate_pose_quality
from modules.state.state_estimator import ConstantVelocityEKF
from modules.states_machine.state_machine import TaskScheduler


class DockingTask:
    STATE_LOCKED = "locked"
    STATE_SEARCH = "search"
    STATE_TRACK = "track"
    STATE_PRE_ALIGN = "pre_align"
    STATE_DOCKED = "docked"
    STATE_FAILED = "failed"
    STATE_APPROACH = STATE_TRACK
    STATE_ALIGN = STATE_PRE_ALIGN
    
    MAX_SEARCH_TIME = 3.0  # 最大搜索时间（秒）
    SEARCH_SPEED = 0.2  # 搜索旋转速度 (rad/s)
    VALID_OUTPUT_BACKENDS = {"mavlink_velocity", "rc_override"}

    def __init__(self, camera, pixhawk, state_machine, tracking_config=None):
        """
        初始化对接任务
        :param camera: camera.py 实例
        :param pixhawk: pixhawk_comm 实例
        :param state_machine: 状态机引用，用于反馈状态
        """
        self.camera = camera
        self.pixhawk = pixhawk
        self.state_machine = state_machine  # 状态机引用
        tracking_config = tracking_config or {}
        self.tracking_config = tracking_config
        self.stage = self.STATE_SEARCH  # 初始状态为搜索
        
        # 控制器
        self.approach_ctrl = path_follower.PathFollower()
        self.precise_ctrl = precision_docking.PrecisionDockingController()
        self.tracking_ctrl = VisualTrackingController(
            desired_z_m=tracking_config.get("desired_z_m", 0.8),
            max_v_m_s=tracking_config.get("max_v_m_s", 0.4),
            max_yaw_rate_deg_s=tracking_config.get("max_yaw_rate_deg_s", 25.0),
            kp_lateral=tracking_config.get("kp_lateral", 0.4),
            kp_vertical=tracking_config.get("kp_vertical", 0.3),
            kp_distance=tracking_config.get("kp_distance", 0.4),
            kp_yaw=tracking_config.get("kp_yaw", 1.0),
            pre_dock_position_tolerance_m=tracking_config.get("pre_dock_position_tolerance_m", 0.05),
            pre_dock_distance_tolerance_m=tracking_config.get("pre_dock_distance_tolerance_m", 0.05),
            pre_dock_yaw_tolerance_deg=tracking_config.get("pre_dock_yaw_tolerance_deg", 5.0),
            camera_to_body=tracking_config.get("camera_to_body", {}),
            min_pre_dock_valid_frames=tracking_config.get("min_pre_dock_valid_frames", 3),
            pre_dock_recent_observation_max_age_s=tracking_config.get("pre_dock_recent_observation_max_age_s", 0.5),
            control_mode=tracking_config.get("control_mode", "p"),
            pid_config=tracking_config.get("pid", {}),
        )
        self.enable_motion = bool(tracking_config.get("enable_motion", False))
        self.output_backend = tracking_config.get("output_backend", "mavlink_velocity")
        self.required_mode = tracking_config.get("required_mode", "GUIDED")
        self.rc_mapper = RcOverrideMapper(tracking_config.get("rc_override", {}))
        self.estimator = ConstantVelocityEKF(max_lost_frames=tracking_config.get("max_lost_frames", 10))
        
        # 任务变量
        self.start_time = time.time()
        self.search_start_time = None
        self.last_pose = None
        self.attempts = 0
        self.max_attempts = 3
        self.lost_counter = 0#容错
        self.max_lost_frames = tracking_config.get("max_lost_frames", 10)
        self.filtered_state = None
        self.last_command = self.tracking_ctrl.neutral_command()
        self.last_mavlink_command = motion_command_from_mapping(self.last_command).as_mavlink_body_ned()
        self.last_rc_override = {}
        self.pre_dock_ready = False
        self.valid_observation_count = 0
        self.last_valid_observation_time = None
        self.pre_dock_recent_observation_max_age_s = float(
            tracking_config.get("pre_dock_recent_observation_max_age_s", 0.5)
        )
        # 状态机所需属性
        self.name = "docking"
        self.status = "idle"

    def start(self):
        """开始对接任务"""
        if self.enable_motion:
            try:
                self._validate_motion_output_config()

                # 检查并解锁
                if not self.pixhawk.is_armed():
                    self.pixhawk.arm_vehicle()
                    time.sleep(2)  # 等待解锁生效

                if not self.pixhawk.is_armed():
                    raise RuntimeError("飞控解锁失败")

                # 设置飞控模式，默认 GUIDED。仅 enable_motion=true 时执行。
                if not self.pixhawk.set_mode(self.required_mode):
                    raise RuntimeError("飞控模式切换失败")

            except Exception as e:
                print(f"[DockingTask] ❌ 启动失败: {e}")
                self.status = "failed"
                self.state_machine.notify_task_failed(self.name)  # 通知状态机失败
                return  # 退出start，不继续执行

        # 解锁和模式切换成功后继续启动任务
        print(f"[DockingTask] 🚀 启动对接任务，dry_run={not self.enable_motion}")
        self.stage = self.STATE_SEARCH
        self.start_time = time.time()
        self.search_start_time = None
        self.attempts = 0
        self.status = "running"
        
        # 通知状态机任务已开始
        self.state_machine.notify_task_start(self.name)


    def stop(self):
        """停止对接任务"""
        print("[DockingTask] ⏹️ 停止对接任务")
        self.pixhawk.stop()
        self.status = "stopped"
        
        # 通知状态机任务已停止
        self.state_machine.notify_task_stop(self.name)######

    def run(self):
        """
        主任务循环，每次读取相对位姿并决定控制方式
        由状态机定期调用
        """
        if self.status != "running":
            return

        # 检查任务超时（60秒未完成）
        if time.time() - self.start_time > 60.0:
            print("[DockingTask] ⏰ 任务超时，切换到失败状态")
            self.stage = self.STATE_FAILED
            self._handle_failure()
            return

        try:
            now = time.time()
            pose = self.camera.get_pose()
            if pose is not None and not validate_pose_quality(pose, self.tracking_config):
                print(f"[DockingTask] invalid pose rejected: {pose.get('reject_reason', '')}")
                pose = None
            if pose is not None:
                self.last_pose = pose
                self.lost_counter = 0
                self.valid_observation_count += 1
                self.last_valid_observation_time = now
                self.filtered_state = self.estimator.update(pose, pose.get("timestamp", now))
                self.filtered_state = self._annotate_state(self.filtered_state, has_valid_observation=True, now=now)
            else:
                self.lost_counter += 1
                print(f"[DockingTask] ⚠️ 第 {self.lost_counter} 次丢失目标")
                self.filtered_state = self.estimator.predict(now)
                self.filtered_state = self._annotate_state(self.filtered_state, has_valid_observation=False, now=now)
                if self.filtered_state.get("status") == "lost":
                    return self._handle_no_target()

            print(f"[DockingTask] 📍 当前阶段: {self.stage}")
            print(f"[DockingTask] 📸 目标位姿: {pose or self.filtered_state}")

            if self.stage == self.STATE_SEARCH:
                if pose is not None and self.camera.target_detected():
                    print("[DockingTask] ✅ 目标发现! 切换到跟踪状态")
                    self.stage = self.STATE_TRACK
                    self.search_start_time = None
                else:
                    return self._handle_no_target()

            if self.stage in (self.STATE_TRACK, self.STATE_PRE_ALIGN):
                self._track(self.filtered_state)
            elif self.stage == self.STATE_DOCKED:
                self._docked()
            elif self.stage == self.STATE_FAILED:
                self._handle_failure()

        except Exception as e:
            print(f"[DockingTask] ❌ 运行异常: {str(e)}")
            self.stage = self.STATE_FAILED
            self._handle_failure()

    def get_status(self):
        """
        获取当前任务状态
        由状态机定期调用以获取任务状态
        """
        return {
            "name": self.name,
            "stage": self.stage,
            "status": self.status,
            "timestamp": time.time(),
            "last_pose": self.last_pose,
            "filtered_state": self.filtered_state,
            "control_cmd": self.last_command,
            "mavlink_cmd": self.last_mavlink_command,
            "rc_override": self.last_rc_override,
            "pre_dock_ready": self.pre_dock_ready,
            "enable_motion": self.enable_motion,
            "output_backend": self.output_backend,
            "attempts": self.attempts
        }

    def reset(self):
        """重置任务状态"""
        print("[DockingTask] 🔄 重置对接任务")
        self.attempts += 1
        self.pre_dock_ready = False
        self.last_command = self.tracking_ctrl.neutral_command()
        self.last_mavlink_command = motion_command_from_mapping(self.last_command).as_mavlink_body_ned()
        self.last_rc_override = {}
        self.valid_observation_count = 0
        self.last_valid_observation_time = None
        self.estimator.reset()
        if self.attempts >= self.max_attempts:
            self.stage = self.STATE_FAILED
            print(self.attempts)
            print(self.stage)
            self._handle_failure()
        else:
            self.stage = self.STATE_SEARCH
            self.search_start_time = None
            self.status = "running"

    def _handle_no_target(self):
        """处理目标丢失情况"""
        self.pre_dock_ready = False
        self.last_command = self.tracking_ctrl.neutral_command()
        self.last_mavlink_command = motion_command_from_mapping(self.last_command).as_mavlink_body_ned()
        self.last_rc_override = self.rc_mapper.map_motion_command(self.last_command)
        if self.enable_motion:
            self._send_motion_command(self.last_command)

        if self.stage == self.STATE_SEARCH:
            # 检查搜索超时
            current_time = time.time()
            if not self.search_start_time:
                self.search_start_time = current_time
                print("[DockingTask] 🔍 开始搜索目标...")
            elif current_time - self.search_start_time > self.MAX_SEARCH_TIME:
                print("[DockingTask] ⏰ 搜索超时，尝试重置")
                self.reset()
            print(current_time)
        else:
            # 对接过程中目标丢失，退回搜索状态
            print("[DockingTask] ⚠️ 目标丢失! 返回搜索")
            self.stage = self.STATE_SEARCH
            self.search_start_time = time.time()

    def _track(self, state):
        """跟踪阶段：计算控制量，默认dry-run不下发推进控制。"""
        if not state or state.get("status") == "lost":
            return self._handle_no_target()

        self.last_command = self.tracking_ctrl.compute_command(state)
        state.update(self.tracking_ctrl.pre_dock_diagnostics(state))
        self.filtered_state = state
        self.pre_dock_ready = self.tracking_ctrl.is_pre_dock_ready(state)
        self.last_mavlink_command = motion_command_from_mapping(self.last_command).as_mavlink_body_ned()
        self.last_rc_override = self.rc_mapper.map_motion_command(self.last_command)

        if self.enable_motion:
            self._send_motion_command(self.last_command)

        print(f"[tracking] 📸 控制输出: {self.last_command}, dry_run={not self.enable_motion}")
        if self.pre_dock_ready:
            print("[DockingTask] ✅ 视觉预对准完成")
            self.stage = self.STATE_PRE_ALIGN
        else:
            self.stage = self.STATE_TRACK

    def _annotate_state(self, state, has_valid_observation, now=None):
        state = dict(state or {})
        now = time.time() if now is None else now
        latest_pose_age = ""
        has_recent_valid_observation = False
        if self.last_valid_observation_time is not None:
            latest_pose_age = max(0.0, now - self.last_valid_observation_time)
            has_recent_valid_observation = latest_pose_age <= self.pre_dock_recent_observation_max_age_s
        if not has_recent_valid_observation:
            self.valid_observation_count = 0
        state.update(camera_state_to_body_error(state, self.tracking_config))
        state["has_valid_observation"] = bool(has_valid_observation)
        state["valid_observation_count"] = self.valid_observation_count
        state["has_recent_valid_observation"] = has_recent_valid_observation
        state["latest_pose_age_s"] = latest_pose_age
        state["pre_dock_valid_frame_count"] = self.valid_observation_count
        state["pre_dock_recent_observation_max_age_s"] = self.pre_dock_recent_observation_max_age_s
        return state

    def _send_motion_command(self, command):
        if self.output_backend == "rc_override":
            self.pixhawk.send_rc_override(self.rc_mapper.map_motion_command(command))
        elif self.output_backend == "mavlink_velocity":
            self.pixhawk.send_velocity_command(command)
        else:
            raise RuntimeError(f"unsupported output_backend: {self.output_backend}")

    def _validate_motion_output_config(self):
        if self.output_backend not in self.VALID_OUTPUT_BACKENDS:
            raise RuntimeError(f"unsupported output_backend: {self.output_backend}")
        if self.output_backend == "rc_override":
            self.rc_mapper.validate_for_motion(require_enabled=True)

    def _search_target(self):
        """搜索目标阶段 - 当检测到位姿时自动切换到接近状态"""
        # 主循环中会处理无目标情况
        # 如果有目标，自动进入approach状态
        if self.camera.target_detected():#最少检测到5次
            print("[DockingTask] ✅ 目标发现! 切换到接近状态")
            self.stage = self.STATE_APPROACH
            self.search_start_time = None

    def _approach(self, pose):
        """远距离接近阶段"""
        vel_cmd = self.approach_ctrl.compute_cmd(pose)
        # self.pixhawk.send_velocity_command(vel_cmd)
        print(f"[approach] 📸 目标输出: {vel_cmd}")

        if self._is_near_target(pose):
            print("[DockingTask] ✅ 接近完成! 切换到精准对接")
            self.stage = self.STATE_ALIGN
        elif time.time() - self.start_time > 45.0:
            print("[DockingTask] ⏰ 接近超时，尝试重置")
            self.reset()

    def _align(self, pose):
        """精准对接阶段"""
        vel_cmd = self.precise_ctrl.compute_control(pose)
        # self.pixhawk.send_velocity_command(vel_cmd)
        print(f"[align] 📸 目标输出: {vel_cmd}")

        if self._is_docked(pose):
            print("[DockingTask] ✅ 对接成功!")
            self.stage = self.STATE_DOCKED
            # self._docked()
        elif time.time() - self.start_time > 55.0:
            print("[DockingTask] ⏰ 对接超时，尝试重置")
            self.reset()

    def _docked(self):
        """对接完成状态"""
        self.pixhawk.stop()
        self.status = "completed"
        # 通知状态机任务完成
        self.state_machine.notify_task_completed(self.name)
        # 可以启动充电任务
        self.state_machine.start_task("charging")

    def _handle_failure(self):
        """处理任务失败"""
        self.status = "failed"
        # 通知状态机任务失败
        self.pixhawk.stop()
        self.state_machine.notify_task_failed(self.name)
        
        # 可以选择返航或其他恢复动作
      #  self.state_machine.start_task("return_to_home")

    def _is_near_target(self, pose):
        """判断是否接近目标（切换精准对接的条件）"""
        return abs(pose["x"]) < 0.3 and abs(pose["y"]) < 0.3 and abs(pose["z"]) < 0.3

    def _is_docked(self, pose):
        """判断是否完成对接（姿态 + 距离都满足）"""
        angle_tolerance_deg = 5.0
        dist_tolerance = 0.05  # 10cm
        
        dist_ok = (abs(pose["x"]) < dist_tolerance and 
                   abs(pose["y"]) < dist_tolerance and 
                   abs(pose["z"]) < dist_tolerance)
        
        ang_ok = (abs(pose["roll"]) < angle_tolerance_deg and
                  abs(pose["pitch"]) < angle_tolerance_deg and
                  abs(pose["yaw"]) < angle_tolerance_deg)
        
        return dist_ok and ang_ok
    
# 在 docking_task.py 文件末尾添加以下代码
if __name__ == '__main__':
    import time
    import logging
    import numpy as np
    from unittest.mock import MagicMock
    from modules.controller.path_follower import PathFollower
    from modules.controller.precision_docking import PrecisionDockingController

    print("===== 对接任务调试程序 =====")
    
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    )
    
    print("创建模拟相机...")
    
    # 创建模拟相机类
    class MockCamera:
        def __init__(self):
            self._target_detected = False
            self.continuous_detections = 10
            self.continuous_lost = 0
            self.min_detections = 5
            self.max_lost = 3
            self.x, self.y, self.z = -0.02, 0.66, -12.22
            self.roll, self.pitch, self.yaw = 163.09, 353.69, 355.84
            # 模拟目标移动路径
            self.path = []
            for i in range(30):
                self.path.append({
                    'x': -0.02 + (0 if i < 5 else (i-5)*0.1 if i < 20 else 0.8),
                    'y': 0.66 + (0 if i < 10 else (i-10)*-0.05 if i < 25 else -0.5),
                    'z': -12.22 + (i*0.4 if i < 20 else (30-i)*0.2 if i < 25 else -0.1),
                    'roll': 163.09 + (i*0.1),
                    'pitch': 353.69 + (i*-0.05),
                    'yaw': 355.84 + (i*-0.5)
                })
            self.step = 0
        
        def get_pose(self):
            """获取模拟位姿数据"""
            if self.step >= len(self.path):
                self.step = len(self.path) - 1
                
            # 模拟目标检测状态
            if self.step < 3:  # 开始阶段模拟无目标情况
                self._target_detected = False
                self.step += 1  # 关键：确保step正常递增
                return None
                
            if self.step > 25:  # 对接完成后模拟无目标
                self._target_detected = False
                self.step += 1
                return None
                
            pose = self.path[self.step]
            self.step += 1
            self._target_detected = True  # 标记目标已检测到
            return pose
        
        # 添加目标检测方法（必须的接口方法）
        def target_detected(self):
            return self._target_detected
        
        def is_target_stable(self):
            return self.continuous_detections >= self.min_detections
        
        def is_target_lost(self):
            return self.continuous_lost >= self.max_lost
        
    # 创建模拟Pixhawk类
    class MockPixhawk:
        def __init__(self):
            self.logger = logging.getLogger("MockPixhawk")
            self.last_command = None
            self.mode = "GUIDED"
            
        def connect(self):
            self.logger.info("连接模拟Pixhawk")
            return True
            
        def arm(self):
            self.logger.info("解锁模拟Pixhawk")
            return True
            
        def set_mode(self, mode):
            self.logger.info(f"设置飞行模式为: {mode}")
            self.mode = mode
            return True
            
        def send_velocity_command(self, cmd):
            self.last_command = cmd
            self.logger.info(f"发送速度指令: {cmd}")
            
        def stop(self):
            self.logger.info("停止所有运动")
            self.last_command = {"vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0}
            
        def get_status(self):
            return {
                "connected": True,
                "armed": True,
                "mode": self.mode
            }
    
    # 创建模拟状态机
    class MockStateMachine:
        def __init__(self):
            self.tasks = {}
            self.events = []
            
        def register_task(self, name, task):
            self.tasks[name] = task
            
        def notify_task_start(self, task_name):
            self.events.append(f"START:{task_name}")
            
        def notify_task_stop(self, task_name):
            self.events.append(f"STOP:{task_name}")
            
        def notify_task_completed(self, task_name):
            self.events.append(f"COMPLETED:{task_name}")
            
        def notify_task_failed(self, task_name):
            self.events.append(f"FAILED:{task_name}")
            
        def start_task(self, task_name):
            self.events.append(f"START_OTHER:{task_name}")
            
        def get_events(self):
            return self.events
    
    # 创建模拟控制器（简化版）
    class MockPathFollower:
        def compute_cmd(self, pose):
            # 简化版的路径跟随逻辑
            vx = max(-0.5, min(0.5, pose['z'] * -0.1))  # 根据距离调整前进速度
            return {"vx": vx, "vy": 0, "vz": 0, "yaw_rate": 0}
            
    class MockPrecisionDockingController:
        def compute_command(self, pose):
            # 简化版的精准对接逻辑
            vx = max(-0.2, min(0.2, pose['z'] * -0.5))
            vy = max(-0.1, min(0.1, pose['y'] * -0.2))
            return {"vx": vx, "vy": vy, "vz": 0, "yaw_rate": pose['yaw'] * -0.01}
    
    print("设置模拟环境...")
    
    # 创建模拟组件
    camera = MockCamera()
    pixhawk = MockPixhawk()
    state_machine = MockStateMachine()
    
    # 创建对接任务实例
    docking_task = DockingTask(camera, pixhawk, state_machine)
    docking_task.approach_ctrl = PathFollower()
    docking_task.precise_ctrl = PrecisionDockingController()
    
    # 注册任务到状态机
    state_machine.register_task("docking", docking_task)
    
    print("启动对接任务...")
    docking_task.start()
    
    print("运行对接任务循环...")
    start_time = time.time()
    try:
        while time.time() - start_time < 90:  # 最长运行90秒
            docking_task.run()
            
            # 显示当前任务状态
            status = docking_task.get_status()
            print(f"\n[DockingTask] 阶段: {status['stage']}")
            if status['last_pose']:
                pose = status['last_pose']
                print(f"  位姿: X={pose.get('x', 0):.2f}, Y={pose.get('y', 0):.2f}, Z={pose.get('z', 0):.2f}")
            print(f"  尝试次数: {status['attempts']}, 状态: {status['status']}")
            
            # 显示Pixhawk状态
            if pixhawk.last_command:
                cmd = pixhawk.last_command
                print(f"  Pixhawk指令: vx={cmd.get('vx',0):.2f}, vy={cmd.get('vy',0):.2f}, vz={cmd.get('vz',0):.2f}, yaw={cmd.get('yaw_rate',0):.3f}")
            
            # 显示状态机事件
            events = state_machine.get_events()
            if events:
                print(f"  状态机事件: {events[-1] if events else '无'}")
            
            # 检查任务是否完成
            if status["status"] in ["completed", "failed", "stopped"]:
                print(f"任务结束状态: {status['status']}")
                break
                
            time.sleep(0.5)  # 模拟主循环的调用间隔
            
    except KeyboardInterrupt:
        print("用户中断调试")
    
    print("\n===== 调试结果总结 =====")
    print(f"任务最终状态: {docking_task.get_status()['status']}")
    print("状态机事件历史:")
    for event in state_machine.get_events():
        print(f"  - {event}")
    
    print("===== 调试程序结束 =====")
