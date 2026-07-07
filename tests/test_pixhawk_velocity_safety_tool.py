import importlib
import io
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path


class FakeMavlinkConstants:
    MAV_FRAME_LOCAL_NED = 1
    MAV_FRAME_BODY_NED = 8
    POSITION_TARGET_TYPEMASK_X_IGNORE = 1 << 0
    POSITION_TARGET_TYPEMASK_Y_IGNORE = 1 << 1
    POSITION_TARGET_TYPEMASK_Z_IGNORE = 1 << 2
    POSITION_TARGET_TYPEMASK_AX_IGNORE = 1 << 3
    POSITION_TARGET_TYPEMASK_AY_IGNORE = 1 << 4
    POSITION_TARGET_TYPEMASK_AZ_IGNORE = 1 << 5
    POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1 << 10
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 1 << 11
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1


class FakeHeartbeat:
    custom_mode = 4
    base_mode = 0


class FakeMav:
    def __init__(self):
        self.position_targets = []
        self.mode_commands = []
        self.long_commands = []

    def set_position_target_local_ned_send(self, *args):
        self.position_targets.append(args)

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
        self.wait_heartbeat_timeout = None
        self.flightmode = "MANUAL"

    def wait_heartbeat(self, timeout=None):
        self.wait_heartbeat_timeout = timeout
        return FakeHeartbeat()

    def mode_mapping(self):
        return {"MANUAL": 0, "GUIDED": 4}

    def motors_armed(self):
        return False

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
    for name in [
        "modules.comms.mavlink_velocity",
        "modules.comms.pixhawk_comm",
        "tools.test_pixhawk_velocity_safety",
    ]:
        sys.modules.pop(name, None)
    return importlib.import_module("tools.test_pixhawk_velocity_safety"), fake_mavutil


def write_config(directory, body=""):
    path = Path(directory) / "settings.yaml"
    path.write_text(
        "pixhawk_comm:\n"
        "  device: /dev/ttl_pixhawk\n"
        "  baud: 115200\n"
        f"{body}",
        encoding="utf-8",
    )
    return path


class PixhawkVelocitySafetyToolTests(unittest.TestCase):
    def test_default_status_check_does_not_send_motion_mode_or_arm_commands(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = write_config(tmpdir)

            exit_code = tool.main(
                ["--config", str(config_path)],
                mavutil_module=fake_mavutil,
                stdout=output,
            )

        self.assertEqual(exit_code, 0)
        self.assertEqual(fake_mavutil.connections, [("/dev/ttl_pixhawk", 115200)])
        self.assertEqual(master.mav.position_targets, [])
        self.assertEqual(master.mav.mode_commands, [])
        self.assertEqual(master.mav.long_commands, [])
        self.assertTrue(master.closed)
        self.assertIn("mode", output.getvalue())

    def test_send_requires_explicit_motion_confirmation(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = write_config(tmpdir)

            exit_code = tool.main(
                [
                    "--config",
                    str(config_path),
                    "--axis",
                    "forward",
                    "--value",
                    "0.03",
                    "--duration",
                    "1.0",
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
            config_path = write_config(tmpdir)

            exit_code = tool.main(
                [
                    "--config",
                    str(config_path),
                    "--axis",
                    "forward",
                    "--value",
                    "0.03",
                    "--duration",
                    "0.01",
                    "--set-mode",
                    "GUIDED",
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

    def test_limits_reject_excessive_linear_yaw_and_duration_values(self):
        master = FakeMaster()
        tool, _ = import_tool_with_fake_mavutil(master)

        with self.assertRaises(ValueError):
            tool.build_motion_command("forward", 0.051)
        with self.assertRaises(ValueError):
            tool.build_motion_command("yaw", 5.1)
        with self.assertRaises(ValueError):
            tool.validate_duration(2.1)

    def test_axis_values_map_to_motion_command_fields(self):
        master = FakeMaster()
        tool, _ = import_tool_with_fake_mavutil(master)

        self.assertEqual(tool.build_motion_command("forward", 0.03).forward_m_s, 0.03)
        self.assertEqual(tool.build_motion_command("right", -0.02).right_m_s, -0.02)
        self.assertEqual(tool.build_motion_command("up", 0.01).up_m_s, 0.01)
        self.assertAlmostEqual(tool.build_motion_command("yaw", 5.0).yaw_rate_rad_s, math.radians(5.0))

    def test_send_path_finishes_with_neutral_velocity(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = write_config(tmpdir)

            exit_code = tool.main(
                [
                    "--config",
                    str(config_path),
                    "--axis",
                    "forward",
                    "--value",
                    "0.03",
                    "--duration",
                    "0.01",
                    "--send",
                    "--confirm-motion",
                ],
                mavutil_module=fake_mavutil,
                sleeper=lambda _: None,
                stdout=io.StringIO(),
            )

        self.assertEqual(exit_code, 0)
        self.assertGreaterEqual(len(master.mav.position_targets), 2)
        self.assertEqual(master.mav.position_targets[0][8], 0.03)
        last_target = master.mav.position_targets[-1]
        self.assertEqual(last_target[8], 0.0)
        self.assertEqual(last_target[9], 0.0)
        self.assertEqual(last_target[10], -0.0)
        self.assertEqual(last_target[15], 0.0)

    def test_send_path_prints_motion_mode_and_post_status_diagnostics(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = write_config(tmpdir)

            exit_code = tool.main(
                [
                    "--config",
                    str(config_path),
                    "--axis",
                    "yaw",
                    "--value",
                    "3.0",
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
        text = output.getvalue()
        self.assertIn("set_mode_requested: MANUAL", text)
        self.assertIn("mode_after_set: MANUAL", text)
        self.assertIn("mode_change_confirmed: true", text)
        self.assertIn("motion_command: axis=yaw value=3.0", text)
        self.assertRegex(text, r"motion_frames_sent: [1-9]\d*")
        self.assertRegex(text, r"neutral_frames_sent: [1-9]\d*")
        self.assertIn("post_mode:", text)
        self.assertIn("post_armed:", text)
        self.assertEqual(master.mav.mode_commands[0][2], 0)

    def test_mode_change_diagnostics_report_confirmed_when_mode_updates(self):
        master = FakeMaster()
        tool, fake_mavutil = import_tool_with_fake_mavutil(master)
        output = io.StringIO()
        original_set_mode_send = master.mav.set_mode_send

        def set_mode_and_update_flightmode(*args):
            original_set_mode_send(*args)
            master.flightmode = "MANUAL"

        master.mav.set_mode_send = set_mode_and_update_flightmode
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = write_config(tmpdir)

            exit_code = tool.main(
                [
                    "--config",
                    str(config_path),
                    "--axis",
                    "forward",
                    "--value",
                    "0.03",
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
        text = output.getvalue()
        self.assertIn("mode_after_set: MANUAL", text)
        self.assertIn("mode_change_confirmed: true", text)


if __name__ == "__main__":
    unittest.main()
