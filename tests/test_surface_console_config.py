import tempfile
import unittest
from pathlib import Path

from tools.surface_console.config_store import read_console_config, update_console_config


SAMPLE_SETTINGS = """pixhawk_comm:
  device: "/dev/ttl_pixhawk"
  baud: 115200

vision_tracking:
  marker_type: "aruco"
  desired_z_m: 0.8
  max_v_m_s: 0.4
  max_yaw_rate_deg_s: 10.0
  control_deadband_m: 0.01
  yaw_deadband_deg: 1.0
  command_smoothing_alpha: 0.6
  tracking_vertical_mode: "visual_pid"
  pre_align_axis_mode: "small_correction"
  pre_align_correction_scale: 0.25
  pre_align_max_v_m_s: 0.05
  pre_align_max_yaw_rate_deg_s: 3.0
  enable_motion: false
  min_pre_dock_valid_frames: 3
  pre_dock_recent_observation_max_age_s: 0.5
  pre_dock_position_tolerance_m: 0.05
  pre_dock_distance_tolerance_m: 0.05
  pre_dock_yaw_tolerance_deg: 5.0

surface_comm:
  port: 9002
"""


class SurfaceConsoleConfigTests(unittest.TestCase):
    def test_read_console_config_returns_only_editable_vision_fields(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(SAMPLE_SETTINGS, encoding="utf-8")

            values = read_console_config(path)

        self.assertEqual(values["desired_z_m"], 0.8)
        self.assertEqual(values["enable_motion"], False)
        self.assertEqual(values["min_pre_dock_valid_frames"], 3)
        self.assertEqual(values["tracking_vertical_mode"], "visual_pid")
        self.assertEqual(values["pre_align_axis_mode"], "small_correction")
        self.assertNotIn("device", values)
        self.assertNotIn("marker_type", values)

    def test_update_console_config_writes_whitelisted_values_and_preserves_other_lines(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(SAMPLE_SETTINGS, encoding="utf-8")

            updated = update_console_config(
                path,
                {
                    "desired_z_m": 0.5,
                    "min_pre_dock_valid_frames": 4,
                    "tracking_vertical_mode": "hold_captured_ch3",
                    "pre_align_axis_mode": "lock_horizontal",
                    "enable_motion": True,
                },
                confirm_motion=True,
            )
            text = path.read_text(encoding="utf-8")

        self.assertEqual(updated["desired_z_m"], 0.5)
        self.assertEqual(updated["min_pre_dock_valid_frames"], 4)
        self.assertEqual(updated["tracking_vertical_mode"], "hold_captured_ch3")
        self.assertEqual(updated["pre_align_axis_mode"], "lock_horizontal")
        self.assertEqual(updated["enable_motion"], True)
        self.assertIn('device: "/dev/ttl_pixhawk"', text)
        self.assertIn("desired_z_m: 0.5", text)
        self.assertIn("min_pre_dock_valid_frames: 4", text)
        self.assertIn('tracking_vertical_mode: "hold_captured_ch3"', text)
        self.assertIn('pre_align_axis_mode: "lock_horizontal"', text)
        self.assertIn("enable_motion: true", text)

    def test_update_console_config_rejects_unknown_or_unsafe_enable_motion(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(SAMPLE_SETTINGS, encoding="utf-8")

            with self.assertRaises(ValueError):
                update_console_config(path, {"device": "/dev/video0"})
            with self.assertRaises(PermissionError):
                update_console_config(path, {"enable_motion": True}, confirm_motion=False)

    def test_update_console_config_rejects_unknown_axis_modes(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(SAMPLE_SETTINGS, encoding="utf-8")

            with self.assertRaises(ValueError):
                update_console_config(path, {"tracking_vertical_mode": "alt_hold"})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_axis_mode": "drift"})


if __name__ == "__main__":
    unittest.main()
