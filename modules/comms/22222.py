#!/usr/bin/env python3
import time
from pymavlink import mavutil

connection_str = '/dev/ttyACM0'  # 改为你的 Pixhawk 串口
baud_rate = 115200
OVERRIDE_HZ = 10
TEST_SECONDS = 12

# 连接
print(f"连接 Pixhawk: {connection_str}@{baud_rate} ...")
master = mavutil.mavlink_connection(connection_str, baud=baud_rate, autoreconnect=True)
print("等待 HEARTBEAT ...")
hb = master.wait_heartbeat(timeout=10)
if not hb:
    print("❌ 未收到 HEARTBEAT")
    exit(1)
print(f"已连接 系统 {master.target_system} 组件 {master.target_component}")
print(f"初始 HEARTBEAT: base_mode={hb.base_mode} custom_mode={hb.custom_mode}")

# 检查是否有外部深度计
print("检测外部压力传感器 (SCALED_PRESSURE) ...")
have_pressure = False
t0 = time.time()
while time.time() - t0 < 3:
    msg = master.recv_match(type=['SCALED_PRESSURE', 'SCALED_PRESSURE2', 'SCALED_PRESSURE3'], blocking=False)
    if msg:
        have_pressure = True
        print("✅ 检测到压力传感器:", msg.get_type())
        break
    time.sleep(0.1)
if not have_pressure:
    print("⚠️ 未检测到压力传感器，Depth Hold 无法生效，建议在 Stabilize 下测试水平推力")

# 发送 ARM 指令（手动确认安全开关，否则飞控拒绝）
print("发送 ARM 指令 ...")
master.mav.command_long_send(master.target_system, master.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

# 等待 armed
def wait_armed(timeout=8):
    t_start = time.time()
    while time.time() - t_start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return True
    return False

if wait_armed(8):
    print("✅ 已解锁")
else:
    print("❌ 解锁失败（检查安全开关、failsafe 或外部深度计）")

# 如果无外部深度计，则用 Stabilize 模式测试 RC override
print("切换到 Stabilize 模式测试 RC override ...")
STABILIZE_MODE = 81
master.set_mode(STABILIZE_MODE)

# RC override 函数
def send_rc(ch3=1500, ch4=1500, ch5=1600, ch6=1600):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, ch3, ch4, ch5, ch6, 1500, 1500
    )

# 发送 RC override 并打印 PWM
prev = None
t_start = time.time()
interval = 1.0 / OVERRIDE_HZ
next_send = time.time()
print("开始测试水平推力 RC override ...")
while time.time() - t_start < TEST_SECONDS:
    now = time.time()
    if now >= next_send:
        send_rc()
        next_send = now + interval

    # 请求 SERVO_OUTPUT_RAW
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                 mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
                                 0,0,0,0,0,0)

    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=0.5)
    if msg:
        cur = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
               msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw]
        if prev is None:
            print("[PWM initial] " + " ".join(f"{i+1}:{v}" for i,v in enumerate(cur)))
        else:
            diffs = [f"{i}:{a}->{b}" for i,(a,b) in enumerate(zip(prev, cur),1) if a!=b]
            if diffs:
                print("[PWM change] " + " ".join(diffs))
        prev = cur
    time.sleep(0.01)

# 恢复中位
send_rc(1500,1500,1500,1500)
print("测试完成。")
