from pymavlink import mavutil
import time

# ------------------------
# 配置
# ------------------------
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
TARGET_SYSTEM = 1
TARGET_COMPONENT = 0
WAIT_AFTER_ARM = 5          # 等待秒数
CHECK_INTERVAL = 0.2        # 循环间隔

# ------------------------
# 连接 Pixhawk
# ------------------------
print("连接 Pixhawk...")
master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUDRATE)
print("等待初始 HEARTBEAT...")
master.wait_heartbeat()
print(f"已连接到系统 {master.target_system}, 组件 {master.target_component}")

# ------------------------
# 工具函数
# ------------------------
def arm_vehicle():
    master.mav.command_long_send(
        TARGET_SYSTEM,
        TARGET_COMPONENT,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("[安全解锁] 指令已发送")

def disarm_vehicle():
    master.mav.command_long_send(
        TARGET_SYSTEM,
        TARGET_COMPONENT,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("[上锁] 指令已发送")

def set_safe_mode():
    """切换到 MANUAL/STABILIZED 模式，确保可上锁"""
    # mode=0 为 MANUAL/STABILIZED 对应 PX4 CUSTOM_MODE，你可根据固件确认
    master.mav.set_mode_send(
        TARGET_SYSTEM,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0
    )
    print("[模式切换] 已切换到安全模式")

def get_armed_state():
    hb_msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if hb_msg:
        armed = bool(hb_msg.base_mode & 0x80)
        system_status = hb_msg.system_status
        return armed, system_status
    return None, None

def force_low_throttle():
    master.mav.rc_channels_override_send(
        TARGET_SYSTEM, TARGET_COMPONENT,
        1500, 1500, 1000, 1500, 0, 0, 0, 0
    )
    print("[RC_OVERRIDE] 油门已置低")

def print_statustext_messages():
    while True:
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if not msg:
            break
        print(f"[固件信息] {msg.text}")

# ------------------------
# 自动解锁
# ------------------------
print("开始自动解锁...")
while True:
    armed, system_status = get_armed_state()
    if armed is None:
        print("等待 HEARTBEAT...")
        time.sleep(CHECK_INTERVAL)
        continue
    print(f"[解锁状态] armed={armed}, system_status={system_status}")
    if armed:
        print("已解锁")
        break
    arm_vehicle()
    print_statustext_messages()
    time.sleep(1)

# ------------------------
# 等待任务执行
# ------------------------
print(f"等待 {WAIT_AFTER_ARM} 秒后自动上锁...")
time.sleep(WAIT_AFTER_ARM)

# ------------------------
# 自动上锁
# ------------------------
print("开始自动上锁...")
set_safe_mode()  # 切换到安全模式
while True:
    armed, system_status = get_armed_state()
    if armed is None:
        print("等待 HEARTBEAT...")
        time.sleep(CHECK_INTERVAL)
        continue

    print(f"[上锁状态] armed={armed}, system_status={system_status}")

    if not armed:
        print("已上锁成功")
        break

    # 强制油门最低
    force_low_throttle()

    # 发送 DISARM 指令
    disarm_vehicle()

    # 打印固件 STATUSTEXT 消息
    for _ in range(5):
        print_statustext_messages()
        time.sleep(0.05)

    time.sleep(CHECK_INTERVAL)
