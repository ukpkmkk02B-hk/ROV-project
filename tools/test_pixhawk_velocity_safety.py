import argparse
import math
import sys
import time
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover - Linux runtime should provide PyYAML.
    yaml = None

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - Linux runtime should provide pymavlink.
    mavutil = None

from modules.comms.mavlink_velocity import send_body_velocity_command
from modules.controller.motion_command import MotionCommand


MAX_LINEAR_M_S = 0.05
MAX_YAW_DEG_S = 5.0
MAX_DURATION_S = 2.0
DEFAULT_SEND_HZ = 5.0
DEFAULT_NEUTRAL_DURATION_S = 0.5


def load_pixhawk_config(config_path):
    if yaml is None:
        raise RuntimeError("PyYAML is required to read config/settings.yaml")
    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    config = data.get("pixhawk_comm", data) or {}
    device = config.get("device")
    baud = config.get("baud", config.get("baudrate"))
    if not device:
        raise ValueError("pixhawk_comm.device is required")
    if not baud:
        raise ValueError("pixhawk_comm.baud is required")
    return {"device": device, "baud": int(baud)}


def validate_duration(duration_s):
    duration_s = float(duration_s)
    if duration_s <= 0.0:
        raise ValueError("duration must be > 0")
    if duration_s > MAX_DURATION_S:
        raise ValueError(f"duration must be <= {MAX_DURATION_S:.1f}s")
    return duration_s


def build_motion_command(axis, value):
    axis = (axis or "").lower()
    value = float(value)
    if axis in {"forward", "right", "up"}:
        if abs(value) > MAX_LINEAR_M_S:
            raise ValueError(f"{axis} value must be within +/-{MAX_LINEAR_M_S:.2f} m/s")
        fields = {"forward": "forward_m_s", "right": "right_m_s", "up": "up_m_s"}
        return MotionCommand(**{fields[axis]: value})
    if axis == "yaw":
        if abs(value) > MAX_YAW_DEG_S:
            raise ValueError(f"yaw value must be within +/-{MAX_YAW_DEG_S:.1f} deg/s")
        return MotionCommand(yaw_rate_rad_s=math.radians(value))
    raise ValueError("axis must be one of: forward, right, up, yaw")


def connect_pixhawk(config, mavutil_module=mavutil, timeout=5.0):
    if mavutil_module is None:
        raise RuntimeError("pymavlink is required to connect to Pixhawk")
    master = mavutil_module.mavlink_connection(config["device"], baud=config["baud"])
    heartbeat = master.wait_heartbeat(timeout=timeout)
    if heartbeat is None:
        raise RuntimeError("timed out waiting for Pixhawk heartbeat")
    return master, heartbeat


def _mode_from_heartbeat(master, heartbeat):
    mode = getattr(master, "flightmode", None)
    if mode:
        return mode
    try:
        reverse = {value: name for name, value in master.mode_mapping().items()}
        return reverse.get(getattr(heartbeat, "custom_mode", None), "UNKNOWN")
    except Exception:
        return "UNKNOWN"


def _armed_from_heartbeat(master, heartbeat, mavutil_module=mavutil):
    try:
        return bool(master.motors_armed())
    except Exception:
        if mavutil_module is None:
            return False
        flag = getattr(mavutil_module.mavlink, "MAV_MODE_FLAG_SAFETY_ARMED", 0)
        return bool(getattr(heartbeat, "base_mode", 0) & flag)


def print_status(master, heartbeat, mavutil_module=mavutil, stdout=sys.stdout):
    mode = _mode_from_heartbeat(master, heartbeat)
    armed = _armed_from_heartbeat(master, heartbeat, mavutil_module=mavutil_module)
    try:
        modes = sorted(master.mode_mapping().keys())
    except Exception:
        modes = []
    print(f"mode: {mode}", file=stdout)
    print(f"armed: {armed}", file=stdout)
    print(f"supported_modes: {', '.join(modes) if modes else 'unknown'}", file=stdout)


def set_mode(master, mode_name, mavutil_module=mavutil):
    modes = master.mode_mapping()
    mode_id = modes.get(mode_name.upper())
    if mode_id is None:
        for name, value in modes.items():
            if name.lower() == mode_name.lower():
                mode_id = value
                break
    if mode_id is None:
        raise ValueError(f"unsupported mode: {mode_name}")

    if isinstance(mode_id, tuple):
        base_mode, custom_mode = mode_id
    else:
        base_mode = mavutil_module.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = mode_id
    master.mav.set_mode_send(master.target_system, base_mode, custom_mode)


def send_arm_command(master, arm, mavutil_module=mavutil):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil_module.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1 if arm else 0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def send_motion_for_duration(
    master,
    command,
    duration_s,
    mavutil_module=mavutil,
    send_hz=DEFAULT_SEND_HZ,
    neutral_duration_s=DEFAULT_NEUTRAL_DURATION_S,
    sleeper=time.sleep,
    monotonic=time.monotonic,
):
    interval = 1.0 / float(send_hz)
    end_time = monotonic() + float(duration_s)
    sent_motion = False
    try:
        while True:
            now = monotonic()
            if sent_motion and now >= end_time:
                break
            send_body_velocity_command(
                master,
                master.target_system,
                master.target_component,
                command,
                mavutil_module=mavutil_module,
            )
            sent_motion = True
            sleeper(interval)
    finally:
        send_body_velocity_command(
            master,
            master.target_system,
            master.target_component,
            MotionCommand.neutral(),
            mavutil_module=mavutil_module,
        )
        neutral_end = monotonic() + max(0.0, float(neutral_duration_s))
        while monotonic() < neutral_end:
            sleeper(interval)
            send_body_velocity_command(
                master,
                master.target_system,
                master.target_component,
                MotionCommand.neutral(),
                mavutil_module=mavutil_module,
            )


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Safe Pixhawk body velocity test tool")
    parser.add_argument("--config", default="config/settings.yaml")
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--axis", choices=["forward", "right", "up", "yaw"])
    parser.add_argument("--value", type=float)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--set-mode")
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--send", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    return parser


def validate_request(args):
    if args.confirm_motion and not args.send:
        raise ValueError("--confirm-motion is only valid with --send")
    if args.set_mode and not args.send:
        raise ValueError("--set-mode is only allowed with --send")
    if args.arm and not args.send:
        raise ValueError("--arm is only allowed with --send")
    if not args.send:
        return
    if not args.confirm_motion:
        raise ValueError("--send requires --confirm-motion")
    if args.axis is None:
        raise ValueError("--send requires --axis")
    if args.value is None:
        raise ValueError("--send requires --value")
    validate_duration(args.duration)
    build_motion_command(args.axis, args.value)


def main(
    argv=None,
    mavutil_module=mavutil,
    sleeper=time.sleep,
    stdout=sys.stdout,
    stderr=sys.stderr,
    neutral_duration_s=DEFAULT_NEUTRAL_DURATION_S,
):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        validate_request(args)
        config = load_pixhawk_config(Path(args.config))
        master, heartbeat = connect_pixhawk(
            config,
            mavutil_module=mavutil_module,
            timeout=args.connect_timeout,
        )
        try:
            print_status(master, heartbeat, mavutil_module=mavutil_module, stdout=stdout)
            if not args.send:
                print("read_only: true", file=stdout)
                return 0

            command = build_motion_command(args.axis, args.value)
            if args.set_mode:
                set_mode(master, args.set_mode, mavutil_module=mavutil_module)
            if args.arm:
                send_arm_command(master, True, mavutil_module=mavutil_module)
            try:
                send_motion_for_duration(
                    master,
                    command,
                    validate_duration(args.duration),
                    mavutil_module=mavutil_module,
                    sleeper=sleeper,
                    neutral_duration_s=neutral_duration_s,
                )
            finally:
                if args.arm:
                    send_arm_command(master, False, mavutil_module=mavutil_module)
            return 0
        finally:
            if master is not None:
                master.close()
    except Exception as exc:
        print(f"error: {exc}", file=stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
