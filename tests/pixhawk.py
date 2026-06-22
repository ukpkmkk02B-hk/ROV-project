import time
from pymavlink import mavutil

def main():
    connection_string = 'udpout:192.168.2.1:14550'
    master = mavutil.mavlink_connection(connection_string, source_system=1)

    # 等待飞控回应 Heartbeat，确保连接建立
    master.wait_heartbeat()
    print("[✅] Heartbeat received from Pixhawk")

    print("[INFO] 开始循环发送模拟压力数据（SCALED_PRESSURE2）...")

    last_heartbeat_time = 0

    while True:
        now = time.time()
        time_boot_ms = int((now * 1000) % 0xFFFFFFFF)
        pressure_pa = 100000  # Pa
        press_abs = pressure_pa / 100.0  # hPa
        temperature = 2000  # 20.00°C

        # 每秒发送一次 HEARTBEAT，保持连接活跃
        if now - last_heartbeat_time > 1.0:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            last_heartbeat_time = now
            print("[❤️ ] Sent heartbeat")

        # 发送压力数据
        master.mav.scaled_pressure2_send(
            time_boot_ms,
            press_abs,
            0.0,
            temperature
        )
        print(f"[📤] 发送 SCALED_PRESSURE2: {press_abs:.2f} hPa")
        time.sleep(0.05)  # 20Hz

if __name__ == "__main__":
    main()
