import importlib
import io
import sys
import tempfile
import types
import unittest
from pathlib import Path


class FakeMavlinkConstants:
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_MODE_FLAG_SAFETY_ARMED = 128


class FakeHeartbeat:
    custom_mode = 19
    base_mode = 0


class FakeMav:
    def __init__(self):
        self.rc_overrides = []
        self.mode_commands = []
        self.long_commands = []

    def rc_channels_override_send(self, *args):
        self.rc_overrides.append(args)

    def set_mode_send(self, *args):
        self.mode_commands.append(args)

    def command_long_send(self, *args):
        self.long_commands.append(args)


class FakeMaster:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = FakeMav()
        self.closed = False
        self.flightmode = "MANUAL"
        self.armed = False

    def wait_heartbeat(self, timeout=None):
        return FakeHeartbeat()

    def recv_match(self, *args, **kwargs):
        return FakeHeartbeat()

    def mode_mapping(self):
        return {"MANUAL": 19, "STABILIZE": 0, "ALT_HOLD": 2}

    def motors_armed(self):
        return self.armed

    def close(self):
        self.closed = True


class FakeMavutil:
    def __init__(self, master):
        self.mavlink = FakeMavlinkConstants
        self.master = master
        self.connections = []

    def mavlink_connection(self, device, baud):
        self.connections.append((device, baud))
        return self.master


def import_tool_with_fake_mavutil(master):
    fake_mavutil = FakeMavutil(master)
    pymavlink = types.ModuleType("pymavlink")
    pymavlink.mavutil = fake_mavutil
    sys.modules["pymavlink"] = pymavlink
    sys.modules["pymavlink.mavutil"] = fake_mavutil
    sys.modules["yaml"] = types.SimpleNamespace(
        safe_load=lambda _: {"pixhawk_comm": {"device": "/dev/ttl_pixhawk", "baud": 115200}}
    )
    sys.modules.pop("tools.test_pixhawk_rc_override_safety", None)
    return importlib.import_module("tools.test_pixhawk_rc_override_safety"), fake_mavutil


def write_config(directory):
    path = Path(directory) / "settings.yaml"
    path.write_text(
        "pixhawk_comm:\n"
        "  device: /dev/ttl_pixhawk\n"
        "  baud: 115200\n",
        encoding="utf-8",
    )
    return path


class PixhawkRcOverrideSafetyToolTests(unittest.TestCase):
    def test_default_status_check_does_not_send_rc_mode_or_arm_commands(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpdir:
            exit_code = tool.main(
                ["--config", str(write_config(tmpdir))],
                mavutil_module=fake_mavutil,
                stdout=output,
            )

        self.assertEqual(exit_code, 0)
        self.assertEqual(fake_mavutil.connections, [("/dev/ttl_pixhawk", 115200)])
        self.assertEqual(master.mav.rc_overrides, [])
        self.assertEqual(master.mav.mode_commands, [])
        self.assertEqual(master.mav.long_commands, [])
        self.assertIn("read_only: true", output.getvalue())
        self.assertTrue(master.closed)

    def test_send_requires_explicit_motion_confirmation_before_connecting(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        with tempfile.TemporaryDirectory() as tmpdir:
            exit_code = tool.main(
                [
                    "--config",
                    str(write_config(tmpdir)),
                    "--axis",
                    "forward",
                    "--pwm-offset",
                    "30",
                    "--send",
                ],
                mavutil_module=fake_mavutil,
                stdout=io.StringIO(),
                stderr=io.StringIO(),
            )

        self.assertEqual(exit_code, 2)
        self.assertEqual(fake_mavutil.connections, [])

    def test_non_manual_set_mode_is_rejected_before_connecting(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        stderr = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpdir:
            exit_code = tool.main(
                [
                    "--config",
                    str(write_config(tmpdir)),
                    "--axis",
                    "forward",
                    "--pwm-offset",
                    "20",
                    "--duration",
                    "0.01",
                    "--set-mode",
                    "STABILIZE",
                    "--send",
                    "--confirm-motion",
                ],
                mavutil_module=fake_mavutil,
                stdout=io.StringIO(),
                stderr=stderr,
            )

        self.assertEqual(exit_code, 2)
        self.assertEqual(fake_mavutil.connections, [])
        self.assertIn("only MANUAL mode is supported", stderr.getvalue())

    def test_limits_reject_excessive_pwm_and_duration_values(self):
        master = FakeMaster()
        tool, _ = import_tool_with_fake_mavutil(master)

        with self.assertRaises(ValueError):
            tool.build_override_channels("forward", 51)
        with self.assertRaises(ValueError):
            tool.validate_duration(2.1)

    def test_right_axis_uses_current_vehicle_direction(self):
        master = FakeMaster()
        tool, _ = import_tool_with_fake_mavutil(master)

        channels = tool.build_override_channels("right", 30)

        self.assertEqual(channels["ch6"], 1470)
        self.assertEqual(channels["ch5"], 1500)

    def test_forward_axis_uses_current_vehicle_direction(self):
        master = FakeMaster()
        tool, _ = import_tool_with_fake_mavutil(master)

        channels = tool.build_override_channels("forward", 30)

        self.assertEqual(channels["ch5"], 1470)
        self.assertEqual(channels["ch6"], 1500)

    def test_forward_send_path_sends_axis_pwm_then_neutral(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpdir:
            exit_code = tool.main(
                [
                    "--config",
                    str(write_config(tmpdir)),
                    "--axis",
                    "forward",
                    "--pwm-offset",
                    "30",
                    "--duration",
                    "0.01",
                    "--set-mode",
                    "MANUAL",
                    "--send",
                    "--confirm-motion",
                ],
                mavutil_module=fake_mavutil,
                sleeper=lambda _: None,
                stdout=output,
                neutral_duration_s=0.0,
            )

        self.assertEqual(exit_code, 0)
        self.assertGreaterEqual(len(master.mav.rc_overrides), 2)
        first = master.mav.rc_overrides[0]
        last = master.mav.rc_overrides[-1]
        self.assertEqual(first[0], master.target_system)
        self.assertEqual(first[1], master.target_component)
        self.assertEqual(first[6], 1470)
        self.assertEqual(last[2:], (1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500))
        text = output.getvalue()
        self.assertIn("rc_override: axis=forward pwm_offset=30", text)
        self.assertRegex(text, r"rc_frames_sent: [1-9]\d*")
        self.assertRegex(text, r"neutral_frames_sent: [1-9]\d*")
        self.assertIn("post_armed: False", text)

    def test_arm_path_reports_confirmed_arm_and_disarm_states(self):
        master = FakeMaster()
        original_command_long_send = master.mav.command_long_send

        def command_long_and_update_armed(*args):
            original_command_long_send(*args)
            master.armed = bool(args[4])

        master.mav.command_long_send = command_long_and_update_armed
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpdir:
            exit_code = tool.main(
                [
                    "--config",
                    str(write_config(tmpdir)),
                    "--axis",
                    "forward",
                    "--pwm-offset",
                    "20",
                    "--duration",
                    "0.01",
                    "--set-mode",
                    "MANUAL",
                    "--send",
                    "--confirm-motion",
                    "--arm",
                ],
                mavutil_module=fake_mavutil,
                sleeper=lambda _: None,
                stdout=output,
                neutral_duration_s=0.0,
            )

        self.assertEqual(exit_code, 0)
        text = output.getvalue()
        self.assertIn("arm_requested: true", text)
        self.assertIn("arm_after_request: True", text)
        self.assertIn("arm_confirmed: true", text)
        self.assertIn("disarm_sent: true", text)
        self.assertIn("disarm_after_request: False", text)
        self.assertIn("disarm_confirmed: true", text)

    def test_arm_not_confirmed_skips_rc_motion_and_returns_error(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpdir:
            exit_code = tool.main(
                [
                    "--config",
                    str(write_config(tmpdir)),
                    "--axis",
                    "forward",
                    "--pwm-offset",
                    "20",
                    "--duration",
                    "0.01",
                    "--set-mode",
                    "MANUAL",
                    "--send",
                    "--confirm-motion",
                    "--arm",
                ],
                mavutil_module=fake_mavutil,
                sleeper=lambda _: None,
                stdout=output,
                neutral_duration_s=0.0,
            )

        self.assertEqual(exit_code, 2)
        self.assertEqual(master.mav.rc_overrides, [])
        text = output.getvalue()
        self.assertIn("arm_after_request: False", text)
        self.assertIn("arm_confirmed: false", text)

    def test_arm_requires_set_mode_before_connecting(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        stderr = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpdir:
            exit_code = tool.main(
                [
                    "--config",
                    str(write_config(tmpdir)),
                    "--axis",
                    "forward",
                    "--pwm-offset",
                    "20",
                    "--duration",
                    "0.01",
                    "--send",
                    "--confirm-motion",
                    "--arm",
                ],
                mavutil_module=fake_mavutil,
                stdout=io.StringIO(),
                stderr=stderr,
            )

        self.assertEqual(exit_code, 2)
        self.assertEqual(fake_mavutil.connections, [])
        self.assertIn("--arm requires --set-mode", stderr.getvalue())

    def test_arm_not_confirmed_after_manual_mode_skips_rc_motion(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpdir:
            exit_code = tool.main(
                [
                    "--config",
                    str(write_config(tmpdir)),
                    "--axis",
                    "forward",
                    "--pwm-offset",
                    "20",
                    "--duration",
                    "0.01",
                    "--set-mode",
                    "MANUAL",
                    "--send",
                    "--confirm-motion",
                    "--arm",
                ],
                mavutil_module=fake_mavutil,
                sleeper=lambda _: None,
                stdout=output,
                neutral_duration_s=0.0,
            )

        self.assertEqual(exit_code, 2)
        self.assertEqual(master.mav.rc_overrides, [])
        self.assertGreaterEqual(len(master.mav.long_commands), 1)
        text = output.getvalue()
        self.assertIn("mode_after_set: MANUAL", text)
        self.assertIn("arm_after_request: False", text)
        self.assertIn("arm_confirmed: false", text)

    def test_disarm_not_confirmed_returns_error_after_neutral_rc(self):
        master = FakeMaster()
        original_command_long_send = master.mav.command_long_send

        def command_long_only_arms(*args):
            original_command_long_send(*args)
            if args[4] == 1:
                master.armed = True

        master.mav.command_long_send = command_long_only_arms
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpdir:
            exit_code = tool.main(
                [
                    "--config",
                    str(write_config(tmpdir)),
                    "--axis",
                    "forward",
                    "--pwm-offset",
                    "20",
                    "--duration",
                    "0.01",
                    "--set-mode",
                    "MANUAL",
                    "--send",
                    "--confirm-motion",
                    "--arm",
                ],
                mavutil_module=fake_mavutil,
                sleeper=lambda _: None,
                stdout=output,
                neutral_duration_s=0.0,
            )

        self.assertEqual(exit_code, 2)
        self.assertGreaterEqual(len(master.mav.rc_overrides), 2)
        self.assertIn("disarm_confirmed: false", output.getvalue())


if __name__ == "__main__":
    unittest.main()
