#modules/comm/pixhawk_comm.py
import yaml
import math
import time
import queue
import logging
from pymavlink import mavutil
from modules.comms.mavlink_velocity import send_body_velocity_command

try:
    from .comm_base import CommunicationBase
except ImportError:
    from comm_base import CommunicationBase

class PixhawkComm(CommunicationBase):
    """ArduSub专用的Pixhawk通信模块"""
    
    def __init__(self, config, depthReader=None):
        config_data = self._load_config(config)
        self.master = None
        super().__init__(config_data.get("name", "Pixhawk_ArduSub"))
        self.device = config_data["device"]
        self.baud = config_data["baud"]
        self.master = None
        self.target_system = 1  # 默认目标系统ID
        self.target_component = 1  # 默认目标组件ID
        self.attitude_cache = {}  # 缓存姿态数据
        self.last_heartbeat_time = time.time()
        self.connected = False
        self.depthReader = depthReader
        self.last_pressure_send_time = 0  # 深度计最后发送时间
        self.PRESSURE_SEND_INTERVAL = 0.1 # 20Hz频率（ArduSub推荐）
        self.time_offset = 0  # 飞控时间与NUC时间的偏移量（毫秒）
        self.last_system_time = 0  # 缓存的飞控启动时间
        self.last_local_receive_time = 0  # 最近一次收到SYSTEM_TIME时的NUC时间
        self.current_mode = None  # 初始化
        self.armed_state = False  # 初始化
        self.armed_ack = False  # 初始化
        self.servo_output = {}

        self.last_heartbeat_time = time.time()
        self.PRESSURE_SEND_INTERVAL = 0.1
        self.time_offset = 0
        self.last_system_time = 0
        self.last_local_receive_time = 0
        self.current_mode = None
        self.armed_state = False
        self.armed_ack = False
        self.servo_output = {}
        self.velocity_cache = {}

    def _load_config(self, config):
        """加载配置文件"""
        if isinstance(config, str):
            with open(config) as f:
                return yaml.safe_load(f)
        return config
    
    def start(self):
        """启动通信线程"""
        try:
            self.logger.info(f"尝试连接到Pixhawk: {self.device}@{self.baud} baud")
            self.master = mavutil.mavlink_connection(self.device, baud=self.baud)
            self.master.wait_heartbeat(timeout=3)
            self.connected = True
            self.logger.info("已连接到Pixhawk!")
            # self.master.mav.param_set_send(
            #     self.master.target_system,
            #     self.master.target_component,
            #     b'GND_EXT_BUS',
            #     1,
            #     mavutil.mavlink.MAV_PARAM_TYPE_INT8
            #     )
            # # 设定使用哪个数据源（0 表示使用 SCALED_PRESSURE）
            # self.master.mav.param_set_send(
            #     self.master.target_system,
            #     self.master.target_component,
            #     b'BARO_EXT_BUS',
            #     0,
            #     mavutil.mavlink.MAV_PARAM_TYPE_INT8
            # )
            # self.send_pressure_data()
            super().start()
            
        except Exception as e:
            self.logger.error(f"连接Pixhawk失败: {e}")
            self.connected = False
    
    def is_connected(self):
        """检查是否已连接"""
        return self.connected
    
    def _run_loop(self):
        """通信线程主循环"""
        try:
            self.logger.info("等待Pixhawk心跳...")
            # 初始化等待心跳
            if not self.master.wait_heartbeat(timeout=3) and self.running:
                self.logger.error("未收到Pixhawk心跳，连接失败")
                self.connected = False
                return
                
            self.logger.info("已连接到Pixhawk_ardusub!")
            self.connected = True
            
            # 设置姿态消息流速率 (10Hz)
            self.master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # 姿态数据
                10,  # 10 Hz
                1
            )
            self.master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10,  # 10 Hz
                1
            )
            
            # 主消息处理循环
            while self.running:
                self._process_incoming_messages()
                self._send_queued_commands()

                 # 发送深度计数据（20Hz频率）
                # current_time = time.time()
                # if current_time - self.last_pressure_send_time >= self.PRESSURE_SEND_INTERVAL:
                #     self.send_pressure_data()
                #     self.last_pressure_send_time = current_time
                # time.sleep(0.02 )  # 保持约50Hz循环
                
                # 检查心跳超时 (3秒无心跳视为断开)
                if time.time() - self.last_heartbeat_time > 3:
                    self.logger.warning("Pixhawk心跳丢失!")
                    self.connected = False

        except Exception as e:
            self.logger.exception("通信线程异常")
        finally:
            if self.master:
                try:
                    self.master.close()
                    self.logger.info("已关闭Pixhawk连接")
                except:
                    pass
                self.master = None
            self.connected = False
    
    def stop(self):
        """停止通信线程并清理资源"""
        self.logger.info("正在停止通信线程...")
        self.running = False  # 设置停止标志
        self.disarm_vehicle()
        # 关闭串口连接
        if self.master is not None:
            try:
                self.master.close()
                self.logger.info("已关闭Pixhawk连接")
            except Exception as e:
                self.logger.error(f"关闭连接时出错: {e}")
            finally:
                self.master = None
        super().stop()
        self.logger.info("通信线程已停止")
    
    def _process_incoming_messages(self):
        """处理接收到的MAVLink消息"""
        msg = self.master.recv_match(timeout=0.01)  # 超时10ms
        if not msg:
            return

         # 捕获SYSTEM_TIME消息并记录时间点
        if msg.get_type() == "SYSTEM_TIME":
            self.last_system_time = msg.time_boot_ms  # 飞控启动时间
            self.last_local_receive_time = time.time() * 1000  # NUC当前时间（毫秒）
            # 计算时间偏移：飞控时间 + 传输延迟 ≈ NUC时间
            self.time_offset = self.last_local_receive_time - self.last_system_time

        # 更新最后心跳时间
        if msg.get_type() == "HEARTBEAT":
            self.last_heartbeat_time = time.time()
            rev_mode_mapping = {v: k for k, v in self.master.mode_mapping().items()}  # 反转字典
            self.current_mode = rev_mode_mapping.get(msg.custom_mode)
            self.armed_state = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.send_heartbeat()

        # ✅ 监听 VFR_HUD (包含depth)
        # if msg.get_type() == "VFR_HUD":
            # self.logger.info(f"[Pixhawk] 当前深度: {msg.alt:.2f} m")

        # 缓存姿态信息
        if msg.get_type() == "ATTITUDE":
            self.attitude_cache = {
                'roll': math.degrees(msg.roll),
                'pitch': math.degrees(msg.pitch),
                'yaw': math.degrees(msg.yaw),
                'roll_speed': math.degrees(msg.rollspeed),
                'pitch_speed': math.degrees(msg.pitchspeed),
                'yaw_speed': math.degrees(msg.yawspeed),
            }
        elif msg.get_type() == "LOCAL_POSITION_NED":
            self.velocity_cache  = {
                'vx': msg.vx,
                'vy': msg.vy, 
                'vz': msg.vz,
            }
        elif msg.get_type() == "SERVO_OUTPUT_RAW":   # ✅ 捕获PWM输出
            self.servo_output = {
                'time_usec': msg.time_usec,
                'ch1': msg.servo1_raw,
                'ch2': msg.servo2_raw,
                'ch3': msg.servo3_raw,
                'ch4': msg.servo4_raw,
                'ch5': msg.servo5_raw,
                'ch6': msg.servo6_raw,
                'ch7': msg.servo7_raw,
                'ch8': msg.servo8_raw,
        }   
        # 添加到输入队列
        self.input_queue.put({
            'type': msg.get_type(),
            'data': msg.to_dict()
        })

    def get_servo_outputs(self):
        """获取最新PWM输出"""
        return self.servo_output.copy()

    def get_velocity(self):
        """Return latest LOCAL_POSITION_NED velocity cache if available."""
        return self.velocity_cache.copy()

    def get_flight_mode(self):
        """Return latest flight mode reported by HEARTBEAT."""
        return self.current_mode

    def _send_queued_commands(self):
        """发送队列中的命令"""
        try:
            while not self.output_queue.empty() and self.running:
                cmd = self.output_queue.get_nowait()
                self._send_mavlink_command(cmd)
        except queue.Empty:
            pass
            
    def _send_mavlink_command(self, cmd):
        """发送MAVLink命令"""
        try:
            if cmd['type'] == 'arm':
                self.arm_vehicle()
            elif cmd['type'] == 'disarm':
                self.disarm_vehicle()
            elif cmd['type'] == 'set_mode':
                self.set_mode(cmd['mode'])
            elif cmd['type'] == 'velocity':
                self.send_velocity_command(cmd['velocities'])
        except KeyError as e:
            self.logger.warning(f"命令格式错误: 缺少字段 {e}")
        except Exception as e:
            self.logger.error(f"发送命令时出错: {e}")
            
    # 下面是外部调用的接口方法
    def arm_vehicle(self):
        """解锁飞控"""
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # 确认
            1,  # ARM命令
            0, 0, 0, 0, 0, 0  # 参数保留
        )
        self.logger.info("发送解锁指令")
        
    def disarm_vehicle(self):
        """上锁飞控"""
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # 确认
            0,  # DISARM命令
            0, 0, 0, 0, 0, 0  # 参数保留
        )
        self.logger.info("发送上锁指令")
        
    def set_mode(self, mode_name, timeout=5):
        """设置飞行模式，并等待通信线程反馈"""
        modes = self.master.mode_mapping()
        mode_id = modes.get(mode_name.upper())
        
        if mode_id is None:
            for name, id_val in modes.items():
                if name.lower() == mode_name.lower():
                    mode_id = id_val
                    break
        if mode_id is None:
            self.logger.warning(f"未知模式: {mode_name}. 支持的模式: {list(modes.keys())}")
            return False

        self.master.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self.logger.info(f"请求设置为 {mode_name.upper()} 模式")

        # ✅ 等待通信线程报告模式变化
        start = time.time()
        while time.time() - start < timeout:
            if self.current_mode and self.current_mode.lower() == mode_name.lower():
                self.logger.info(f"模式切换成功: {self.current_mode}")
                return True
            time.sleep(0.2)

        self.logger.warning(f"模式切换超时，未切换到 {mode_name}")
        return False

        
    def send_velocity_command(self, vel_cmd):
        """发送速度控制命令 - ArduSub专用"""
        mav_velocity = send_body_velocity_command(
            self.master,
            self.target_system,
            self.target_component,
            vel_cmd,
            mavutil_module=mavutil,
        )
        self.logger.debug(
            f"发送速度命令: Vx={mav_velocity['vx']}, Vy={mav_velocity['vy']}, Vz={mav_velocity['vz']}"
        )
        
    def get_attitude(self):
        """获取当前姿态（转换为度数）"""
        return self.attitude_cache.copy()
        
    def is_armed(self):
        """检查飞控是否解锁"""
        return getattr(self, 'armed_state', False)
    
    def get_supported_modes(self):
        """获取支持的模式列表"""
        if self.is_connected():
            return self.master.mode_mapping()
        return {}
    
    def send_rc_override(self, channels: dict):
        """
        channels: dict, 可传 {'ch3':1500, 'ch4':1500, 'ch5':1600, 'ch6':1600}
        未指定的通道使用中位值 1500
        """
        ch1 = channels.get('ch1', 1500)
        ch2 = channels.get('ch2', 1500)
        ch3 = channels.get('ch3', 1500)
        ch4 = channels.get('ch4', 1500)
        ch5 = channels.get('ch5', 1500)
        ch6 = channels.get('ch6', 1500)
        ch7 = channels.get('ch7', 1500)
        ch8 = channels.get('ch8', 1500)
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8
        )
        
    def send_pressure_data(self):
        """
        直接将原始压强值（放大100倍的mbar）发送给Pixhawk
        参数:
            pressure_raw: 从传感器读取的原始压强值（放大100倍的mbar）
        """
        """以ArduSub要求的频率发送深度计数据（20Hz）"""
        if not self.depthReader:
            self.logger.warning("深度传感器未初始化")
            return False
        
        # 读取传感器数据（注意：read_pressure是方法，需加括号调用）
        pressure_raw = self.depthReader.get_current_pressure()
        print(f"[pixhawk] 📊 当前压力: {pressure_raw:.2f} mbar")

        if pressure_raw is None:
            self.logger.error("读取深度传感器失败")
            return False  
        # 打印当前压力值（调试用）
        
        if not self.connected:
            self.logger.warning("未连接到Pixhawk，无法发送深度数据")
            return False
        if self.last_system_time > 0:
            # 基于飞控启动时间 + 本地流逝时间计算
            elapsed_since_last = time.time() * 1000 - self.last_local_receive_time
            time_boot_ms = int(self.last_system_time + elapsed_since_last)
        else:
            # 降级方案：使用NUC本地时间（需标注不同步）
            time_boot_ms = self.last_system_time
            self.logger.warning("使用未同步的本地时间戳")

        try:    
            # 直接发送原始压强值（单位为0.01 mbar）
            self.master.mav.scaled_pressure2_send(
                time_boot_ms, 
                int (pressure_raw * 100),  # 直接使用传感器提供的原始值（已经是0.01 mbar单位）
                0,
                2000  # 使用固定温度值（20℃）
            )
            print(f"📤 发送深度计数据: {pressure_raw*100} (0.01 mbar)")
            
        except Exception as e:
            self.logger.error(f"发送深度数据失败: {str(e)}")
            return False
# modules/comm/pixhawk_comm.py
# import yaml
# import math
# import time
# import queue
# import logging
# from pymavlink import mavutil

# try:
#     from .comm_base import CommunicationBase
# except ImportError:
#     from comm_base import CommunicationBase


# class PixhawkComm(CommunicationBase):
#     """PX4/ArduSub通用的Pixhawk通信模块（适配 BlueROV2 Heavy）"""

#     def __init__(self, config, depthReader=None):
#         config_data = self._load_config(config)
#         self.master = None
#         super().__init__(config_data.get("name", "Pixhawk_Comm"))
#         self.device = config_data["device"]
#         self.baud = config_data["baud"]
#         self.master = None
#         self.target_system = 1
#         self.target_component = 1
#         self.attitude_cache = {}
#         self.last_heartbeat_time = time.time()
#         self.connected = False
#         self.depthReader = depthReader
#         self.last_pressure_send_time = 0
#         self.PRESSURE_SEND_INTERVAL = 0.1  # 20Hz
#         self.time_offset = 0
#         self.last_system_time = 0
#         self.last_local_receive_time = 0
#         self.current_mode = None
#         self.armed_state = False
#         self.armed_ack = False
#         self.servo_output = {}
#         self.velocity_cache = {}

#     def _load_config(self, config):
#         if isinstance(config, str):
#             with open(config) as f:
#                 return yaml.safe_load(f)
#         return config

#     def start(self):
#         try:
#             self.logger.info(f"尝试连接到 Pixhawk: {self.device}@{self.baud} baud")
#             self.master = mavutil.mavlink_connection(self.device, baud=self.baud)
#             self.master.wait_heartbeat(timeout=5)
#             self.connected = True
#             self.logger.info("已连接到 Pixhawk!")
#             super().start()
#         except Exception as e:
#             self.logger.error(f"连接 Pixhawk 失败: {e}")
#             self.connected = False

#     def is_connected(self):
#         return self.connected

#     def _run_loop(self):
#         try:
#             self.logger.info("等待 Pixhawk 心跳...")
#             if not self.master or not self.master.wait_heartbeat(timeout=5):
#                 self.logger.error("未收到 Pixhawk 心跳，连接失败")
#                 self.connected = False
#                 return
#             self.logger.info("已连接到 Pixhawk!")
#             self.connected = True

#             # 请求姿态和位置数据
#             self.master.mav.request_data_stream_send(
#                 self.target_system,
#                 self.target_component,
#                 mavutil.mavlink.MAV_DATA_STREAM_ALL,
#                 10,
#                 1
#             )
#             # self.master.mav.request_data_stream_send(
#             #     self.target_system,
#             #     self.target_component,
#             #     mavutil.mavlink.MAV_DATA_STREAM_POSITION,
#             #     10,
#             #     1
#             # )

#             while self.running and self.master:
#                 self._process_incoming_messages()
#                 self._send_queued_commands()
#                 if time.time() - self.last_heartbeat_time > 3:
#                     self.logger.warning("Pixhawk 心跳丢失!")
#                     self.connected = False
#         except Exception as e:
#             self.logger.exception("通信线程异常")
#         finally:
#             if self.master:
#                 try:
#                     self.master.close()
#                     self.logger.info("已关闭 Pixhawk 连接")
#                 except:
#                     pass
#             self.master = None
#             self.connected = False

#     def stop(self):
#         self.logger.info("正在停止通信线程...")
#         self.running = False
#         time.sleep(0.2)  # 等线程处理完
#         if self.master is not None:
#             try:
#                 if self.is_armed():
#                     self.disarm_vehicle()
#                 self.master.close()
#                 self.logger.info("已关闭 Pixhawk 连接")
#             except Exception as e:
#                 self.logger.error(f"关闭连接时出错: {e}")
#             finally:
#                 self.master = None
#         super().stop()
#         self.logger.info("通信线程已停止")

#     def _process_incoming_messages(self):
#         if not self.master:
#             return
#         msg = self.master.recv_match(timeout=0.01)
#         if not msg:
#             return

#         if msg.get_type() == "SYSTEM_TIME":
#             self.last_system_time = msg.time_boot_ms
#             self.last_local_receive_time = time.time() * 1000
#             self.time_offset = self.last_local_receive_time - self.last_system_time

#         # 在 _process_incoming_messages 方法中，处理 HEARTBEAT 消息时：
#         if msg.get_type() == "HEARTBEAT":
#             self.last_heartbeat_time = time.time()
            
#             # PX4: custom_mode 是数字，需要转换为模式名
#             custom_mode = msg.custom_mode
#             # 方法1: 使用mavutil的mode_mapping（可能需要调整）
#             try:
#                 # 获取模式映射字典（数字->名称）
#                 mode_map = self.master.mode_mapping()
#                 # 反转字典查找（注意：可能有多个名称对应同一数字？通常不会）
#                 rev_map = {v: k for k, v in mode_map.items()}
#                 self.current_mode = rev_map.get(custom_mode, f"UNKNOWN({custom_mode})")
#             except:
#                 # 方法2: 手动定义PX4模式映射（备用）
#                 px4_mode_map = {
#                     0: 'MANUAL',
#                     1: 'ALTCTL',
#                     2: 'POSCTL',
#                     3: 'AUTO',
#                     4: 'ACRO',
#                     5: 'OFFBOARD',
#                     6: 'STABILIZED',
#                     7: 'RATTITUDE',
#                     8: 'AUTO.MISSION',
#                     9: 'AUTO.LOITER',
#                     10: 'AUTO.RTL',
#                     11: 'AUTO.LAND',
#                     12: 'AUTO.TAKEOFF',
#                     13: 'AUTO.FOLLOW_TARGET',
#                     14: 'AUTO.PRECLAND',
#                     15: 'ORBIT',
#                     16: 'AUTO.READY',
#                     17: 'AUTO.LOITER',
#                     18: 'AUTO.TAKEOFF'
#                 }
#                 self.current_mode = px4_mode_map.get(custom_mode, f"UNKNOWN({custom_mode})")
            
#             self.armed_state = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
#             self.send_heartbeat()

#         if msg.get_type() == "ATTITUDE":
#             self.attitude_cache = {
#                 'roll': math.degrees(msg.roll),
#                 'pitch': math.degrees(msg.pitch),
#                 'yaw': math.degrees(msg.yaw),
#                 'roll_speed': math.degrees(msg.rollspeed),
#                 'pitch_speed': math.degrees(msg.pitchspeed),
#                 'yaw_speed': math.degrees(msg.yawspeed),
#             }

#         elif msg.get_type() == "LOCAL_POSITION_NED":
#             self.velocity_cache = {
#                 'vx': msg.vx,
#                 'vy': msg.vy,
#                 'vz': msg.vz,
#             }

#         elif msg.get_type() == "SERVO_OUTPUT_RAW":
#             self.servo_output = {
#                 'time_usec': msg.time_usec,
#                 'ch1': msg.servo1_raw,
#                 'ch2': msg.servo2_raw,
#                 'ch3': msg.servo3_raw,
#                 'ch4': msg.servo4_raw,
#                 'ch5': msg.servo5_raw,
#                 'ch6': msg.servo6_raw,
#                 'ch7': msg.servo7_raw,
#                 'ch8': msg.servo8_raw,
#             }

#         self.input_queue.put({
#             'type': msg.get_type(),
#             'data': msg.to_dict()
#         })

#     def get_servo_outputs(self):
#         return self.servo_output.copy()

#     def _send_queued_commands(self):
#         try:
#             while not self.output_queue.empty() and self.running:
#                 cmd = self.output_queue.get_nowait()
#                 self._send_mavlink_command(cmd)
#         except queue.Empty:
#             pass

#     def _send_mavlink_command(self, cmd):
#         try:
#             if cmd['type'] == 'arm':
#                 self.arm_vehicle()
#             elif cmd['type'] == 'disarm':
#                 self.disarm_vehicle()
#             elif cmd['type'] == 'set_mode':
#                 self.set_mode(cmd['mode'])
#             elif cmd['type'] == 'velocity':
#                 self.send_velocity_command(cmd['velocities'])
#         except KeyError as e:
#             self.logger.warning(f"命令格式错误: 缺少字段 {e}")
#         except Exception as e:
#             self.logger.error(f"发送命令时出错: {e}")

#     def arm_vehicle(self):
#         if not self.master:
#             return
#         self.master.mav.command_long_send(
#             self.target_system,
#             self.target_component,
#             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#             0,
#             1, 0, 0, 0, 0, 0, 0
#         )
#         self.logger.info("发送解锁指令")

#     def disarm_vehicle(self):
#         if not self.master:
#             return
#         self.master.mav.command_long_send(
#             self.target_system,
#             self.target_component,
#             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#             0,
#             0, 0, 0, 0, 0, 0, 0
#         )
#         self.logger.info("发送上锁指令")

#     def set_mode(self, mode_name, timeout=5):
#         if not self.master:
#             return False
#         modes = self.master.mode_mapping()
#         mode_id = modes.get(mode_name.upper())

#         if isinstance(mode_id, tuple):
#             base_mode = mode_id[0]
#             custom_mode = mode_id[1]
#         else:
#             base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
#             custom_mode = mode_id

#         self.master.mav.set_mode_send(
#             self.target_system,
#             self.target_component,
#             base_mode,
#             custom_mode
#         )
#         self.logger.info(f"请求设置为 {mode_name.upper()} 模式")

#         start = time.time()
#         while time.time() - start < timeout:
#             if self.current_mode and self.current_mode.lower() == mode_name.lower():
#                 self.logger.info(f"模式切换成功: {self.current_mode}")
#                 return True
#             time.sleep(0.2)

#         self.logger.warning(f"模式切换超时，未切换到 {mode_name.upper()}")
#         return False

#     def send_velocity_command(self, vel_cmd):
#         """PX4/ArduSub兼容的速度控制"""
#         if not self.master:
#             return
#         frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
#         ignore_mask = (
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
#             mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
#         )

#         self.master.mav.set_position_target_local_ned_send(
#             0,
#             self.target_system,
#             self.target_component,
#             frame,
#             ignore_mask,
#             0, 0, 0,
#             vel_cmd.get('vx', 0),
#             vel_cmd.get('vy', 0),
#             vel_cmd.get('vz', 0),
#             0, 0, 0,
#             0,
#             vel_cmd.get("v_yaw", 0)
#         )
#         self.logger.debug(f"发送速度命令: Vx={vel_cmd.get('vx', 0)}, Vy={vel_cmd.get('vy', 0)}, Vz={vel_cmd.get('vz', 0)}")

#     def get_attitude(self):
#         return self.attitude_cache.copy()

#     def is_armed(self):
#         return getattr(self, 'armed_state', False)

#     def get_supported_modes(self):
#         if self.is_connected():
#             return self.master.mode_mapping()
#         return {}

  
if __name__ == "__main__":
    # ArduSub测试配置 - 请根据您的实际设置修改
    TEST_CONFIG = {
        "name": "ArduSub_Test",
        "device": "/dev/ttyACM0",  # 请替换为您的实际设备路径
        "baud": 115200             # ArduSub默认波特率
    }
    
    safe_test = False  # 设置为True只运行安全测试(跳过解锁和速度测试)
    
    # 测试逻辑
    import time
    import queue
    import logging
    from pymavlink import mavutil
    
    # 初始化日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 初始化通信模块
    comm = PixhawkComm(TEST_CONFIG)
    comm.logger.setLevel(logging.DEBUG)  # 启用详细日志
    comm.start()
    
    print("\n" + "="*50)
    print("ArduSub通信模块测试")
    print("="*50)
    print(f"使用设备: {TEST_CONFIG['device']}, 波特率: {TEST_CONFIG['baud']}")
    print("注意: 请确保水下无人机处于安全环境中!")
    
    try:
        # 等待连接建立
        connection_timeout = 10
        start_time = time.time()
        while not comm.is_connected() and time.time() - start_time < connection_timeout:
            print("等待Pixhawk连接...")
            time.sleep(1)
        
        if not comm.is_connected():
            print("连接超时，退出测试")
            exit(1)
            
        print("\n[测试1] 基本连接测试")
        print(f"设备支持的模式: {comm.get_supported_modes()}")
        print("等待接收消息...")
        
        # 接收消息测试
        received_messages = 0
        start_time = time.time()
        while time.time() - start_time < 5:  # 5秒超时
            if not comm.input_queue.empty():
                msg = comm.input_queue.get()
                print(f"  收到消息: {msg['type']}")
                received_messages += 1
                if received_messages >= 5:
                    break
            time.sleep(0.1)
        
        # 获取姿态信息
        attitude = comm.get_attitude()
        if attitude:
            print(f"  当前姿态: Roll={attitude.get('roll', 0):.1f}°, Pitch={attitude.get('pitch', 0):.1f}°, Yaw={attitude.get('yaw', 0):.1f}°")
        else:
            print("  警告: 未收到姿态数据!")
        
        # 测试2: 模式切换
        print("\n[测试2] 模式切换测试")
        # ArduSub常用模式
        test_modes = ["MANUAL", "STABILIZE", "ALT_HOLD", "AUTO"]
        supported_modes = comm.master.mode_mapping()
        print(f"支持的模式: {supported_modes}")
        for mode in test_modes:
            print(f"  尝试切换到 {mode} 模式...")
            if comm.set_mode(mode):
                time.sleep(2)  # 给飞控时间切换模式
        
        # 如果 safe_test 跳过后续
        if safe_test:
            print("\n安全模式: 跳过解锁和速度测试")
            comm.stop()
            exit(0)
        
        # 测试4: 解锁/上锁操作
        print("\n[测试4] 解锁/上锁测试")
        current_armed = comm.is_armed()
        print(f"  当前状态: {'已解锁' if current_armed else '已上锁'}")
        
        # 确定要执行的操作
        action = "disarm" if current_armed else "arm"
        
        print("\n警告: 解锁水下无人机需要满足安全条件!")
        print("请注意: 水上测试时推力器可能空转，避免接触!")
        confirm = input(f"  是否执行{action}操作? (危险! 务必谨慎!) [y/N]: ")
        
        if confirm.lower() == "y":
            if action == "arm":
                print("  发送解锁指令")
                comm.arm_vehicle()
                time.sleep(1)  # 等待解锁生效
            
           
                # 切换到自稳模式(STABILIZE)并确认
        print("\n切换到自稳模式(STABILIZE)以测试 RC override ...")
        if not comm.set_mode("STABILIZE"):
            print("  无法切换到 STABILIZE 模式，跳过 RC override 测试")
        else:
            time.sleep(1)
            print("✅ 已切换到 STABILIZE 模式，开始 RC override 测试")

            # RC override 测试：6 路（前进、后退、左移、右移、上升、下降）
            OVERRIDE_HZ = 5
            STEP_DURATION = 3 # 每个动作保持 3 秒
            interval = 1.0 / OVERRIDE_HZ

            # # 定义测试序列（模拟手柄输入）
            # test_sequence = [
            #     ("前进",  {'ch3': 1500, 'ch4': 1500, 'ch5': 1600, 'ch6': 1500}),
            #     ("后退",  {'ch3': 1500, 'ch4': 1500, 'ch5': 1400, 'ch6': 1500}),
            #     ("左移",  {'ch3': 1500, 'ch4': 1500, 'ch5': 1500, 'ch6': 1400}),
            #     ("右移",  {'ch3': 1500, 'ch4': 1500, 'ch5': 1500, 'ch6': 1600}),
            #     ("上升",  {'ch3': 1600, 'ch4': 1500, 'ch5': 1500, 'ch6': 1500}),
            #     ("下降",  {'ch3': 1400, 'ch4': 1500, 'ch5': 1500, 'ch6': 1500}),
            #     ("回中",  {'ch3': 1500, 'ch4': 1500, 'ch5': 1500, 'ch6': 1500}),
            # ]

            # for action, override in test_sequence:
            #     print(f"\n>>> 执行动作: {action}")
            #     start_time = time.time()
            #     next_send = start_time
            #     while time.time() - start_time < STEP_DURATION:
            #         now = time.time()
            #         if now >= next_send:
            #             comm.send_rc_override(override)
            #             next_send = now + interval

                    # 获取 PWM 输出
            servo_out = comm.get_servo_outputs()
            if servo_out:
                print(f"[PWM] {servo_out}")
                time.sleep(0.05)

            print("\n✅ RC override 测试完成，恢复中位值")
            comm.send_rc_override({'ch3': 1500, 'ch4': 1500, 'ch5': 1500, 'ch6': 1500})
            time.sleep(1)
            
            # 循环完成后可立即上锁
            if comm.is_armed():
                reconfirm = input("  无人机已解锁! 是否立即上锁? [Y/n]: ")
                if not reconfirm.lower() == "n":
                    comm.disarm_vehicle()
                    time.sleep(1)
                    print("  已发送上锁指令")
                else:
                    print("  跳过操作")
        
        print("\n测试完成")
        
    except Exception as e:
        print(f"测试期间发生错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        comm.stop()


# if __name__ == "__main__":
#     import time
#     import logging

#     # 配置日志
#     logging.basicConfig(level=logging.INFO)

#     # Pixhawk 配置
#     config = {
#         "device": "/dev/ttyACM0",  # 根据你的设备修改
#         "baud": 115200,
#         "name": "BlueROV2_Comm"
#     }

#     # 创建通信对象
#     px4 = PixhawkComm(config)
    
#     # 启动通信线程
#     px4.start()

#     # 等待心跳稳定
#     time.sleep(3)

#     if not px4.is_connected():
#         print("连接失败，退出测试")
#         px4.stop()
#         exit(1)

#     print("连接成功，开始测试...")

#     # 查询并打印支持模式
#     supported_modes = px4.get_supported_modes()
#     print("支持的模式:", supported_modes)

#     # 尝试切换到 MANUAL 模式
#     if px4.set_mode("MANUAL"):
#         print("模式切换到 MANUAL 成功")
#     else:
#         print("模式切换到 MANUAL 失败")

#     # 尝试解锁
#     px4.arm_vehicle()
#     time.sleep(2)  # 等待解锁
#     print("解锁状态:", px4.is_armed())

#     # 循环打印姿态和PWM输出，确保通信正常
#     try:
#         for i in range(20):  # 循环 10 秒
#             attitude = px4.get_attitude()
#             pwm = px4.get_servo_outputs()
#             print(f"姿态: {attitude}")
#             print(f"PWM输出: {pwm}")
#             time.sleep(0.5)
#     except KeyboardInterrupt:
#         print("用户中断测试")

#     # 切换到 OFFBOARD 模式，发送速度指令
#     if px4.set_mode("OFFBOARD"):
#         print("模式切换到 OFFBOARD 成功，发送速度指令")
#         vel_cmd = {"vx": 0.2, "vy": 0.0, "vz": 0.0, "v_yaw": 0.0}  # 向前 0.2 m/s
#         px4.send_velocity_command(vel_cmd)
#         time.sleep(2)
#     else:
#         print("模式切换到 OFFBOARD 失败，无法发送速度指令")

#     # 上锁并停止通信
#     px4.disarm_vehicle()
#     time.sleep(2)
#     print("解锁状态:", px4.is_armed())
    
#     px4.stop()
#     print("通信线程已停止，测试结束")
