# modules/comm/pixhawk_comm.py
import yaml
import math
import time
import queue
import logging
from pymavlink import mavutil

try:
    from .comm_base import CommunicationBase
except ImportError:
    from comm_base import CommunicationBase

class PixhawkComm(CommunicationBase):
    """PX4/ArduSub通用的Pixhawk通信模块"""

    def __init__(self, config, depthReader=None):
        config_data = self._load_config(config)
        self.master = None
        super().__init__(config_data.get("name", "Pixhawk_Comm"))
        self.device = config_data["device"]
        self.baud = config_data["baud"]
        self.master = None
        self.target_system = 1
        self.target_component = 1
        self.attitude_cache = {}
        self.last_heartbeat_time = time.time()
        self.connected = False
        self.depthReader = depthReader
        self.last_pressure_send_time = 0
        self.PRESSURE_SEND_INTERVAL = 0.1  # 20Hz
        self.time_offset = 0
        self.last_system_time = 0
        self.last_local_receive_time = 0
        self.current_mode = None
        self.armed_state = False
        self.armed_ack = False
        self.servo_output = {}
        self.velocity_cache = {}

    def _load_config(self, config):
        if isinstance(config, str):
            with open(config) as f:
                return yaml.safe_load(f)
        return config

    def start(self):
        try:
            self.logger.info(f"尝试连接到Pixhawk: {self.device}@{self.baud} baud")
            self.master = mavutil.mavlink_connection(self.device, baud=self.baud)
            self.master.wait_heartbeat(timeout=3)
            self.connected = True
            self.logger.info("已连接到Pixhawk!")
            super().start()
        except Exception as e:
            self.logger.error(f"连接Pixhawk失败: {e}")
            self.connected = False

    def is_connected(self):
        return self.connected

    def _run_loop(self):
        try:
            self.logger.info("等待Pixhawk心跳...")
            if not self.master.wait_heartbeat(timeout=3) and self.running:
                self.logger.error("未收到Pixhawk心跳，连接失败")
                self.connected = False
                return

            self.logger.info("已连接到Pixhawk!")
            self.connected = True

            # 请求姿态和位置数据
            self.master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                10,
                1
            )
            self.master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10,
                1
            )

            while self.running:
                self._process_incoming_messages()
                self._send_queued_commands()

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
        self.logger.info("正在停止通信线程...")
        self.running = False
        self.disarm_vehicle()
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
        msg = self.master.recv_match(timeout=0.01)
        if not msg:
            return

        if msg.get_type() == "SYSTEM_TIME":
            self.last_system_time = msg.time_boot_ms
            self.last_local_receive_time = time.time() * 1000
            self.time_offset = self.last_local_receive_time - self.last_system_time

        if msg.get_type() == "HEARTBEAT":
            self.last_heartbeat_time = time.time()
            rev_mode_mapping = {v: k for k, v in self.master.mode_mapping().items()}
            self.current_mode = rev_mode_mapping.get(msg.custom_mode)
            self.armed_state = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.send_heartbeat()

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
            self.velocity_cache = {
                'vx': msg.vx,
                'vy': msg.vy,
                'vz': msg.vz,
            }
        elif msg.get_type() == "SERVO_OUTPUT_RAW":
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

        self.input_queue.put({
            'type': msg.get_type(),
            'data': msg.to_dict()
        })

    def get_servo_outputs(self):
        return self.servo_output.copy()

    def _send_queued_commands(self):
        try:
            while not self.output_queue.empty() and self.running:
                cmd = self.output_queue.get_nowait()
                self._send_mavlink_command(cmd)
        except queue.Empty:
            pass

    def _send_mavlink_command(self, cmd):
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

    def arm_vehicle(self):
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0
        )
        self.logger.info("发送解锁指令")

    def disarm_vehicle(self):
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0, 0, 0, 0, 0, 0
        )
        self.logger.info("发送上锁指令")

    def set_mode(self, mode_name, timeout=5):
        """支持PX4 OFFBOARD / ArduSub 模式"""
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

        start = time.time()
        while time.time() - start < timeout:
            if self.current_mode and self.current_mode.lower() == mode_name.lower():
                self.logger.info(f"模式切换成功: {self.current_mode}")
                return True
            time.sleep(0.2)

        self.logger.warning(f"模式切换超时，未切换到 {mode_name}")
        return False

    def send_velocity_command(self, vel_cmd):
        """PX4/ArduSub兼容的速度控制"""
        frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        # PX4需要忽略位置，只用速度
        ignore_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            frame,
            ignore_mask,
            0, 0, 0,
            vel_cmd.get('vx', 0),
            vel_cmd.get('vy', 0),
            vel_cmd.get('vz', 0),
            0, 0, 0,
            0,
            vel_cmd.get("v_yaw", 0)
        )
        self.logger.debug(f"发送速度命令: Vx={vel_cmd.get('vx', 0)}, Vy={vel_cmd.get('vy', 0)}, Vz={vel_cmd.get('vz', 0)}")

    def get_attitude(self):
        return self.attitude_cache.copy()

    def is_armed(self):
        return getattr(self, 'armed_state', False)

    def get_supported_modes(self):
        if self.is_connected():
            return self.master.mode_mapping()
        return {}

    def send_pressure_data(self):
        if not self.depthReader:
            self.logger.warning("深度传感器未初始化")
            return False

        pressure_raw = self.depthReader.get_current_pressure()
        print(f"[pixhawk] 📊 当前压力: {pressure_raw:.2f} mbar")

        if pressure_raw is None:
            self.logger.error("读取深度传感器失败")
            return False

        if not self.connected:
            self.logger.warning("未连接到Pixhawk，无法发送深度数据")
            return False
        if self.last_system_time > 0:
            elapsed_since_last = time.time() * 1000 - self.last_local_receive_time
            time_boot_ms = int(self.last_system_time + elapsed_since_last)
        else:
            time_boot_ms = self.last_system_time
            self.logger.warning("使用未同步的本地时间戳")

        try:
            self.master.mav.scaled_pressure2_send(
                time_boot_ms,
                int(pressure_raw * 100),
                0,
                2000
            )
            print(f"📤 发送深度计数据: {pressure_raw*100} (0.01 mbar)")

        except Exception as e:
            self.logger.error(f"发送深度数据失败: {str(e)}")
            return False


import unittest
import time
import logging
from pixhawk_comm111 import PixhawkComm
# 测试配置 - 请根据您的实际设置修改
TEST_CONFIG = {
    "name": "Pixhawk_Test",
    "device": "/dev/ttyACM0",  # 请替换为您的实际设备路径
    "baud": 115200             # PX4默认波特率
}

class TestPixhawkComm(unittest.TestCase):
    """Pixhawk通信模块单元测试"""
    
    @classmethod
    def setUpClass(cls):
        """测试类设置"""
        cls.comm = PixhawkComm(TEST_CONFIG)
        cls.comm.logger.setLevel(logging.DEBUG)
        cls.comm.start()
        
        # 等待连接建立
        connection_timeout = 10
        start_time = time.time()
        while not cls.comm.is_connected() and time.time() - start_time < connection_timeout:
            print("等待Pixhawk连接...")
            time.sleep(1)
        
        if not cls.comm.is_connected():
            raise ConnectionError("连接Pixhawk超时")
    
    @classmethod
    def tearDownClass(cls):
        """测试类清理"""
        if cls.comm.is_connected():
            cls.comm.stop()
    
    def test_01_connection(self):
        """测试1: 基本连接测试"""
        self.assertTrue(self.comm.is_connected(), "Pixhawk连接失败")
        print("✓ Pixhawk连接成功")
    
    def test_02_supported_modes(self):
        """测试2: 获取支持的模式"""
        modes = self.comm.get_supported_modes()
        self.assertIsInstance(modes, dict, "获取支持模式失败")
        self.assertGreater(len(modes), 0, "未获取到任何支持的模式")
        print(f"✓ 支持的模式: {list(modes.keys())}")
    
    def test_03_attitude_data(self):
        """测试3: 姿态数据获取"""
        # 等待一段时间接收数据
        time.sleep(2)
        
        attitude = self.comm.get_attitude()
        self.assertIsInstance(attitude, dict, "姿态数据格式错误")
        
        # 检查必要的姿态字段
        expected_keys = ['roll', 'pitch', 'yaw', 'roll_speed', 'pitch_speed', 'yaw_speed']
        for key in expected_keys:
            self.assertIn(key, attitude, f"姿态数据缺少 {key} 字段")
        
        print(f"✓ 当前姿态: Roll={attitude.get('roll', 0):.1f}°, "
              f"Pitch={attitude.get('pitch', 0):.1f}°, "
              f"Yaw={attitude.get('yaw', 0):.1f}°")
    
    def test_04_servo_outputs(self):
        """测试4: 伺服输出获取"""
        servo_output = self.comm.get_servo_outputs()
        self.assertIsInstance(servo_output, dict, "伺服输出数据格式错误")
        
        if servo_output:  # 如果有数据才检查具体字段
            self.assertIn('ch1', servo_output, "伺服输出缺少 ch1 字段")
            print(f"✓ 伺服输出: {servo_output}")
    
    def test_05_mode_switching(self):
        """测试5: 模式切换测试"""
        # 获取支持的模式并尝试切换
        modes = self.comm.get_supported_modes()
        test_modes = ["MANUAL", "STABILIZE", "ALTCTL", "POSCTL", "AUTO"]
        
        for mode in test_modes:
            if mode in modes:
                success = self.comm.set_mode(mode)
                if success:
                    print(f"✓ 成功切换到 {mode} 模式")
                    time.sleep(1)  # 给飞控时间切换模式
                else:
                    print(f"⚠ 切换到 {mode} 模式失败")
            else:
                print(f"⚠ 模式 {mode} 不被支持")
    
    def test_06_arming_disarming(self):
        """测试6: 解锁/上锁测试（需要用户确认）"""
        current_armed = self.comm.is_armed()
        print(f"当前状态: {'已解锁' if current_armed else '已上锁'}")
        
        # 安全确认
        print("\n⚠️  警告: 解锁无人机需要满足安全条件!")
        print("请注意: 测试时推力器可能启动，确保安全!")
        confirm = input("是否执行解锁/上锁测试? (y/N): ")
        
        if confirm.lower() == 'y':
            if current_armed:
                # 如果已解锁，先上锁
                print("发送上锁指令...")
                self.comm.disarm_vehicle()
                time.sleep(2)
                self.assertFalse(self.comm.is_armed(), "上锁失败")
                print("✓ 成功上锁")
                
                # 再解锁
                print("发送解锁指令...")
                self.comm.arm_vehicle()
                time.sleep(2)
                self.assertTrue(self.comm.is_armed(), "解锁失败")
                print("✓ 成功解锁")
            else:
                # 如果已上锁，先解锁
                print("发送解锁指令...")
                self.comm.arm_vehicle()
                time.sleep(2)
                self.assertTrue(self.comm.is_armed(), "解锁失败")
                print("✓ 成功解锁")
                
                # 再上锁
                print("发送上锁指令...")
                self.comm.disarm_vehicle()
                time.sleep(2)
                self.assertFalse(self.comm.is_armed(), "上锁失败")
                print("✓ 成功上锁")
        else:
            print("跳过解锁/上锁测试")
            self.skipTest("用户跳过解锁/上锁测试")
    
    def test_07_velocity_control(self):
        """测试7: 速度控制测试（需要用户确认）"""
        # 安全确认
        print("\n⚠️  警告: 速度控制将移动无人机!")
        print("请确保在安全环境中测试!")
        confirm = input("是否执行速度控制测试? (y/N): ")
        
        if confirm.lower() == 'y':
            # 切换到适当模式
            if not self.comm.set_mode("ALTCTL"):
                print("无法切换到ALTCTL模式，尝试其他模式...")
                if not self.comm.set_mode("STABILIZE"):
                    print("无法切换到任何合适模式，跳过速度测试")
                    self.skipTest("无法切换到合适模式进行速度测试")
                    return
            
            time.sleep(1)  # 等待模式切换
            
            # 发送速度指令
            print("发送前进速度指令 (vx=0.5m/s)...")
            for _ in range(50):  # 持续5秒
                self.comm.send_velocity_command({'vx': 0.5, 'vy': 0.0, 'vz': 0.0})
                time.sleep(0.1)
            
            # 停止
            print("发送停止指令...")
            for _ in range(10):
                self.comm.send_velocity_command({'vx': 0.0, 'vy': 0.0, 'vz': 0.0})
                time.sleep(0.1)
            
            print("✓ 速度控制测试完成")
        else:
            print("跳过速度控制测试")
            self.skipTest("用户跳过速度控制测试")

def main():
    """主测试函数"""
    print("=" * 60)
    print("Pixhawk通信模块全面测试")
    print("=" * 60)
    print(f"使用设备: {TEST_CONFIG['device']}, 波特率: {TEST_CONFIG['baud']}")
    print("注意: 请确保无人机处于安全环境中!")
    print("=" * 60)
    
    # 运行单元测试
    unittest.main(verbosity=2)

if __name__ == "__main__":
    # 设置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    main()