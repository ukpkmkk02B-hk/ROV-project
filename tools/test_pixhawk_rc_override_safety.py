import argparse
import sys
import time
from pathlib import Path

from modules.controller.manual_modes import normalize_supported_mode
from tools.test_pixhawk_velocity_safety import (
    connect_pixhawk,
    load_pixhawk_config,
    print_status,
    send_arm_command,
    set_mode,
    wait_for_mode,
    _armed_from_heartbeat,
    _mode_from_heartbeat,
    mavutil,
)


NEUTRAL_PWM = 1500
MAX_PWM_OFFSET = 50
MAX_DURATION_S = 2.0
DEFAULT_SEND_HZ = 5.0
DEFAULT_NEUTRAL_DURATION_S = 0.5
AXIS_CHANNELS = {
    "up": "ch3",
    "yaw": "ch4",
    "forward": "ch5",
    "right": "ch6",
}
AXIS_SIGNS = {
    "right": -1,
}


def validate_duration(duration_s):
    duration_s = float(duration_s)
    if duration_s <= 0.0:
        raise ValueError("duration must be > 0")
    if duration_s > MAX_DURATION_S:
        raise ValueError(f"duration must be <= {MAX_DURATION_S:.1f}s")
    return duration_s


def neutral_channels():
    return {f"ch{i}": NEUTRAL_PWM for i in range(1, 9)}


def build_override_channels(axis, pwm_offset):
    axis = (axis or "").lower()
    if axis not in AXIS_CHANNELS:
        raise ValueError("axis must be one of: forward, right, up, yaw")
    pwm_offset = int(pwm_offset)
    if abs(pwm_offset) > MAX_PWM_OFFSET:
        raise ValueError(f"pwm-offset must be within +/-{MAX_PWM_OFFSET}")
    channels = neutral_channels()
    channel = AXIS_CHANNELS[axis]
    channels[channel] = NEUTRAL_PWM + pwm_offset * AXIS_SIGNS.get(axis, 1)
    return channels


def send_rc_override(master, channels):
    ordered = [channels.get(f"ch{i}", NEUTRAL_PWM) for i in range(1, 9)]
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *ordered,
    )


def send_rc_for_duration(
    master,
    channels,
    duration_s,
    send_hz=DEFAULT_SEND_HZ,
    neutral_duration_s=DEFAULT_NEUTRAL_DURATION_S,
    sleeper=time.sleep,
    monotonic=time.monotonic,
):
    interval = 1.0 / float(send_hz)
    end_time = monotonic() + float(duration_s)
    counts = {"rc_frames_sent": 0, "neutral_frames_sent": 0}
    sent_motion = False
    try:
        while True:
            now = monotonic()
            if sent_motion and now >= end_time:
                break
            send_rc_override(master, channels)
            counts["rc_frames_sent"] += 1
            sent_motion = True
            sleeper(interval)
    finally:
        neutral = neutral_channels()
        send_rc_override(master, neutral)
        counts["neutral_frames_sent"] += 1
        neutral_end = monotonic() + max(0.0, float(neutral_duration_s))
        while monotonic() < neutral_end:
            sleeper(interval)
            send_rc_override(master, neutral)
            counts["neutral_frames_sent"] += 1
    return counts


def wait_for_armed(master, expected_armed, timeout_s=1.0, sleeper=time.sleep, monotonic=time.monotonic):
    deadline = monotonic() + max(0.0, float(timeout_s))
    expected = bool(expected_armed)
    heartbeat = None
    while True:
        try:
            armed = bool(master.motors_armed())
        except Exception:
            armed = False
        if armed == expected:
            return True, armed, heartbeat
        if monotonic() >= deadline:
            return False, armed, heartbeat
        try:
            heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.2)
        except AttributeError:
            heartbeat = master.wait_heartbeat(timeout=0.2)
        except Exception:
            heartbeat = None
        sleeper(0.05)


def format_channels(channels):
    return " ".join(f"{name}={channels[name]}" for name in sorted(channels))


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Safe Pixhawk RC override test tool")
    parser.add_argument("--config", default="config/settings.yaml")
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--axis", choices=["forward", "right", "up", "yaw"])
    parser.add_argument("--pwm-offset", type=int)
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
    if args.arm and not args.set_mode:
        raise ValueError("--arm requires --set-mode")
    if not args.send:
        return
    if not args.confirm_motion:
        raise ValueError("--send requires --confirm-motion")
    if args.axis is None:
        raise ValueError("--send requires --axis")
    if args.pwm_offset is None:
        raise ValueError("--send requires --pwm-offset")
    if args.set_mode:
        args.set_mode = normalize_supported_mode(args.set_mode)
    validate_duration(args.duration)
    build_override_channels(args.axis, args.pwm_offset)


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

            channels = build_override_channels(args.axis, args.pwm_offset)
            print(f"rc_override: axis={args.axis} pwm_offset={args.pwm_offset}", file=stdout)
            print(f"rc_channels: {format_channels(channels)}", file=stdout)
            if args.set_mode:
                print(f"set_mode_requested: {args.set_mode}", file=stdout)
                set_mode(master, args.set_mode, mavutil_module=mavutil_module)
                print("set_mode_sent: true", file=stdout)
                confirmed, mode_after_set, _ = wait_for_mode(master, args.set_mode, sleeper=sleeper)
                print(f"mode_after_set: {mode_after_set}", file=stdout)
                print(f"mode_change_confirmed: {str(confirmed).lower()}", file=stdout)
                if not confirmed:
                    print("error: mode_change_not_confirmed", file=stdout)
                    return 2
            if args.arm:
                print("arm_requested: true", file=stdout)
                send_arm_command(master, True, mavutil_module=mavutil_module)
                arm_confirmed, arm_after_request, _ = wait_for_armed(master, True, sleeper=sleeper)
                print(f"arm_after_request: {arm_after_request}", file=stdout)
                print(f"arm_confirmed: {str(arm_confirmed).lower()}", file=stdout)
                if not arm_confirmed:
                    send_arm_command(master, False, mavutil_module=mavutil_module)
                    disarm_confirmed, disarm_after_request, _ = wait_for_armed(master, False, sleeper=sleeper)
                    print("disarm_sent: true", file=stdout)
                    print(f"disarm_after_request: {disarm_after_request}", file=stdout)
                    print(f"disarm_confirmed: {str(disarm_confirmed).lower()}", file=stdout)
                    return 2
            exit_code = 0
            try:
                counts = send_rc_for_duration(
                    master,
                    channels,
                    validate_duration(args.duration),
                    sleeper=sleeper,
                    neutral_duration_s=neutral_duration_s,
                )
                print(f"rc_frames_sent: {counts['rc_frames_sent']}", file=stdout)
                print(f"neutral_frames_sent: {counts['neutral_frames_sent']}", file=stdout)
            finally:
                if args.arm:
                    send_arm_command(master, False, mavutil_module=mavutil_module)
                    print("disarm_sent: true", file=stdout)
                    disarm_confirmed, disarm_after_request, _ = wait_for_armed(master, False, sleeper=sleeper)
                    print(f"disarm_after_request: {disarm_after_request}", file=stdout)
                    print(f"disarm_confirmed: {str(disarm_confirmed).lower()}", file=stdout)
                    if not disarm_confirmed:
                        print("warning: disarm_not_confirmed", file=stdout)
                        exit_code = 2
            post_heartbeat = master.wait_heartbeat(timeout=1.0) or heartbeat
            print(f"post_mode: {_mode_from_heartbeat(master, post_heartbeat)}", file=stdout)
            print(
                f"post_armed: {_armed_from_heartbeat(master, post_heartbeat, mavutil_module=mavutil_module)}",
                file=stdout,
            )
            return exit_code
        finally:
            if master is not None:
                master.close()
    except Exception as exc:
        print(f"error: {exc}", file=stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
