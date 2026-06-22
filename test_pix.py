from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"✅ Connected to system {master.target_system}, component {master.target_component}")

# 获取飞控当前模式
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,  # 请求 SYS_STATUS
    0,0,0,0,0,0,0
)

# 等待 SYS_STATUS 消息
while True:
    msg = master.recv_match(type='SYS_STATUS', blocking=True)
    if msg:
        print(f"🔋 电池电压: {msg.voltage_battery / 1000.0} V")
        break

# 获取 Pixhawk 当前的模式（飞行模式）
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
    0,0,0,0,0,0,0
)
msg = master.recv_match(type='HEARTBEAT', blocking=True)
print(f"模式: {msg.base_mode}")
