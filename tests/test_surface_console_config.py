import tempfile
import unittest
from pathlib import Path

from tools.surface_console.config_store import read_console_config, update_console_config


SAMPLE_SETTINGS = """pixhawk_comm:
  device: "/dev/ttl_pixhawk"
  baud: 115200

vision_tracking:
  marker_type: "aruco"
  min_marker_pixel_size_px: 25.0
  max_reprojection_error_px: 5.0
  camera_to_body:
    yaw_offset_deg: -90.0
  max_v_m_s: 0.4
  max_yaw_rate_deg_s: 10.0
  control_deadband_m: 0.01
  yaw_deadband_deg: 1.0
  command_smoothing_alpha: 0.6
  tracking_vertical_mode: "disabled"
  pre_align_axis_mode: "small_correction"
  pre_align_correction_scale: 0.25
  pre_align_max_v_m_s: 0.05
  pre_align_max_yaw_rate_deg_s: 3.0
  pre_align_buoyancy_hold_pwm: 1600
  pre_align_down_pwm_max: 1700
  pre_align_target_approach_speed_m_s: 0.03
  pre_align_approach_speed_kp: 2000
  pre_dock_approach_speed_tolerance_m_s: 0.01
  pre_align_close_loss_hold_max_distance_m: 0.15
  pre_align_docking_center_offset_camera_x_m: 0.03
  pre_align_docking_center_offset_camera_y_m: 0.06
  pre_align_docking_center_tolerance_m: 0.01
  pre_align_docking_center_release_hysteresis_m: 0.05
  docking_timeout_s: 180
  enable_motion: false
  min_pre_dock_valid_frames: 3
  pre_dock_recent_observation_max_age_s: 0.5
  pre_dock_position_tolerance_m: 0.05
  pre_dock_distance_tolerance_m: 0.05
  pre_dock_yaw_tolerance_deg: 5.0
  pid:
    forward:
      kp: 0.4
      output_limit: 0.4
    right:
      kp: 0.4
      output_limit: 0.4
    up:
      kp: 0.3
      output_limit: 0.4
    yaw:
      kp: 0.5
  rc_override:
    enabled: true
    neutral_pwm: 1500
    pwm_per_m_s: 250
    pwm_per_rad_s: 120
    min_active_pwm_offset: 30

surface_comm:
  port: 9002
"""


class SurfaceConsoleConfigTests(unittest.TestCase):
    def test_read_console_config_returns_only_editable_vision_fields(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(SAMPLE_SETTINGS, encoding="utf-8")

            values = read_console_config(path)

        self.assertEqual(values["enable_motion"], False)
        self.assertEqual(values["min_pre_dock_valid_frames"], 3)
        self.assertEqual(values["tracking_vertical_mode"], "disabled")
        self.assertEqual(values["pre_align_axis_mode"], "small_correction")
        self.assertEqual(values["pre_align_buoyancy_hold_pwm"], 1600)
        self.assertEqual(values["pre_align_down_pwm_max"], 1700)
        self.assertEqual(values["pre_align_target_approach_speed_m_s"], 0.03)
        self.assertEqual(values["pre_align_approach_speed_kp"], 2000)
        self.assertEqual(values["pre_dock_approach_speed_tolerance_m_s"], 0.01)
        self.assertEqual(values["pre_align_close_loss_hold_max_distance_m"], 0.15)
        self.assertEqual(values["pre_align_docking_center_offset_camera_x_m"], 0.03)
        self.assertEqual(values["pre_align_docking_center_offset_camera_y_m"], 0.06)
        self.assertEqual(values["pre_align_docking_center_tolerance_m"], 0.01)
        self.assertEqual(values["pre_align_docking_center_release_hysteresis_m"], 0.05)
        self.assertEqual(values["docking_timeout_s"], 180)
        self.assertEqual(values["min_marker_pixel_size_px"], 25.0)
        self.assertEqual(values["max_reprojection_error_px"], 5.0)
        self.assertEqual(values["camera_to_body.yaw_offset_deg"], -90.0)
        self.assertEqual(values["pid.forward.kp"], 0.4)
        self.assertEqual(values["pid.right.kp"], 0.4)
        self.assertEqual(values["pid.up.kp"], 0.3)
        self.assertEqual(values["pid.yaw.kp"], 0.5)
        self.assertEqual(values["pid.forward.output_limit"], 0.4)
        self.assertEqual(values["pid.right.output_limit"], 0.4)
        self.assertEqual(values["pid.up.output_limit"], 0.4)
        self.assertEqual(values["rc_override.pwm_per_m_s"], 250)
        self.assertEqual(values["rc_override.pwm_per_rad_s"], 120)
        self.assertEqual(values["rc_override.min_active_pwm_offset"], 30)
        self.assertNotIn("device", values)
        self.assertNotIn("marker_type", values)

    def test_update_console_config_writes_whitelisted_values_and_preserves_other_lines(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(SAMPLE_SETTINGS, encoding="utf-8")

            updated = update_console_config(
                path,
                {
                    "min_pre_dock_valid_frames": 4,
                    "tracking_vertical_mode": "hold_captured_ch3",
                    "pre_align_axis_mode": "full_control",
                    "pre_align_buoyancy_hold_pwm": 1580,
                    "pre_align_down_pwm_max": 1680,
                    "pre_align_target_approach_speed_m_s": 0.04,
                    "pre_align_approach_speed_kp": 1800,
                    "pre_dock_approach_speed_tolerance_m_s": 0.015,
                    "pre_align_close_loss_hold_max_distance_m": 0.12,
                    "pre_align_docking_center_offset_camera_x_m": 0.025,
                    "pre_align_docking_center_offset_camera_y_m": 0.055,
                    "pre_align_docking_center_tolerance_m": 0.008,
                    "pre_align_docking_center_release_hysteresis_m": 0.07,
                    "docking_timeout_s": 240,
                    "min_marker_pixel_size_px": 40.0,
                    "max_reprojection_error_px": 3.5,
                    "camera_to_body.yaw_offset_deg": -88.0,
                    "pid.forward.kp": 0.21,
                    "pid.right.kp": 0.22,
                    "pid.up.kp": 0.23,
                    "pid.yaw.kp": 0.24,
                    "pid.forward.output_limit": 0.61,
                    "pid.right.output_limit": 0.62,
                    "pid.up.output_limit": 0.63,
                    "rc_override.pwm_per_m_s": 180.0,
                    "rc_override.pwm_per_rad_s": 90.0,
                    "rc_override.min_active_pwm_offset": 40.0,
                    "enable_motion": True,
                },
                confirm_motion=True,
            )
            text = path.read_text(encoding="utf-8")

        self.assertEqual(updated["min_pre_dock_valid_frames"], 4)
        self.assertEqual(updated["tracking_vertical_mode"], "hold_captured_ch3")
        self.assertEqual(updated["pre_align_axis_mode"], "full_control")
        self.assertEqual(updated["pre_align_buoyancy_hold_pwm"], 1580)
        self.assertEqual(updated["pre_align_down_pwm_max"], 1680)
        self.assertEqual(updated["pre_align_target_approach_speed_m_s"], 0.04)
        self.assertEqual(updated["pre_align_approach_speed_kp"], 1800.0)
        self.assertEqual(updated["pre_dock_approach_speed_tolerance_m_s"], 0.015)
        self.assertEqual(updated["pre_align_close_loss_hold_max_distance_m"], 0.12)
        self.assertEqual(updated["pre_align_docking_center_offset_camera_x_m"], 0.025)
        self.assertEqual(updated["pre_align_docking_center_offset_camera_y_m"], 0.055)
        self.assertEqual(updated["pre_align_docking_center_tolerance_m"], 0.008)
        self.assertEqual(updated["pre_align_docking_center_release_hysteresis_m"], 0.07)
        self.assertEqual(updated["docking_timeout_s"], 240.0)
        self.assertEqual(updated["camera_to_body.yaw_offset_deg"], -88.0)
        self.assertEqual(updated["pid.forward.kp"], 0.21)
        self.assertEqual(updated["pid.forward.output_limit"], 0.61)
        self.assertEqual(updated["rc_override.pwm_per_m_s"], 180.0)
        self.assertEqual(updated["rc_override.min_active_pwm_offset"], 40.0)
        self.assertEqual(updated["enable_motion"], True)
        self.assertIn('device: "/dev/ttl_pixhawk"', text)
        self.assertIn("min_pre_dock_valid_frames: 4", text)
        self.assertIn('tracking_vertical_mode: "hold_captured_ch3"', text)
        self.assertIn('pre_align_axis_mode: "full_control"', text)
        self.assertIn("pre_align_buoyancy_hold_pwm: 1580", text)
        self.assertIn("pre_align_down_pwm_max: 1680", text)
        self.assertIn("pre_align_target_approach_speed_m_s: 0.04", text)
        self.assertIn("pre_align_approach_speed_kp: 1800.0", text)
        self.assertIn("pre_dock_approach_speed_tolerance_m_s: 0.015", text)
        self.assertIn("pre_align_close_loss_hold_max_distance_m: 0.12", text)
        self.assertIn("pre_align_docking_center_offset_camera_x_m: 0.025", text)
        self.assertIn("pre_align_docking_center_offset_camera_y_m: 0.055", text)
        self.assertIn("pre_align_docking_center_tolerance_m: 0.008", text)
        self.assertIn("pre_align_docking_center_release_hysteresis_m: 0.07", text)
        self.assertIn("docking_timeout_s: 240.0", text)
        self.assertIn("min_marker_pixel_size_px: 40.0", text)
        self.assertIn("max_reprojection_error_px: 3.5", text)
        self.assertIn("yaw_offset_deg: -88.0", text)
        self.assertIn("kp: 0.21", text)
        self.assertIn("output_limit: 0.61", text)
        self.assertIn("output_limit: 0.62", text)
        self.assertIn("output_limit: 0.63", text)
        self.assertIn("pwm_per_m_s: 180.0", text)
        self.assertIn("pwm_per_rad_s: 90.0", text)
        self.assertIn("min_active_pwm_offset: 40.0", text)
        self.assertIn("enable_motion: true", text)

    def test_update_console_config_rejects_unknown_or_unsafe_enable_motion(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(SAMPLE_SETTINGS, encoding="utf-8")

            with self.assertRaises(ValueError):
                update_console_config(path, {"device": "/dev/video0"})
            with self.assertRaises(ValueError):
                update_console_config(path, {"desired_z_m": 0.5})
            with self.assertRaises(ValueError):
                update_console_config(path, {"rc_override.channels.forward": "ch1"})
            with self.assertRaises(PermissionError):
                update_console_config(path, {"enable_motion": True}, confirm_motion=False)

    def test_update_console_config_rejects_unknown_axis_modes(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(SAMPLE_SETTINGS, encoding="utf-8")

            with self.assertRaises(ValueError):
                update_console_config(path, {"tracking_vertical_mode": "alt_hold"})
            with self.assertRaises(ValueError):
                update_console_config(path, {"tracking_vertical_mode": "visual_pid"})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_axis_mode": "drift"})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_axis_mode": "lock_horizontal"})

    def test_legacy_visual_pid_is_read_and_saved_as_disabled(self):
        legacy_settings = SAMPLE_SETTINGS.replace(
            'tracking_vertical_mode: "disabled"',
            'tracking_vertical_mode: "visual_pid"',
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(legacy_settings, encoding="utf-8")

            values = read_console_config(path)
            updated = update_console_config(path, {"max_v_m_s": 0.5})
            text = path.read_text(encoding="utf-8")

        self.assertEqual(values["tracking_vertical_mode"], "disabled")
        self.assertEqual(updated["tracking_vertical_mode"], "disabled")
        self.assertIn('tracking_vertical_mode: "disabled"', text)
        self.assertNotIn('tracking_vertical_mode: "visual_pid"', text)

    def test_legacy_lock_horizontal_is_read_and_saved_as_small_correction(self):
        legacy_settings = SAMPLE_SETTINGS.replace(
            'pre_align_axis_mode: "small_correction"',
            'pre_align_axis_mode: "lock_horizontal"',
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(legacy_settings, encoding="utf-8")

            values = read_console_config(path)
            updated = update_console_config(path, {"max_v_m_s": 0.5})
            text = path.read_text(encoding="utf-8")

        self.assertEqual(values["pre_align_axis_mode"], "small_correction")
        self.assertEqual(updated["pre_align_axis_mode"], "small_correction")
        self.assertIn('pre_align_axis_mode: "small_correction"', text)
        self.assertNotIn('pre_align_axis_mode: "lock_horizontal"', text)

    def test_docking_vertical_fields_enforce_ranges_and_hold_not_above_max(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(SAMPLE_SETTINGS, encoding="utf-8")

            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_buoyancy_hold_pwm": 1600.5})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_buoyancy_hold_pwm": 2001})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_down_pwm_max": 2001})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_target_approach_speed_m_s": 0.21})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_approach_speed_kp": 5001})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_dock_approach_speed_tolerance_m_s": 0.11})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_close_loss_hold_max_distance_m": 0.04})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_close_loss_hold_max_distance_m": 0.51})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_docking_center_offset_camera_x_m": -0.21})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_docking_center_offset_camera_y_m": 0.21})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_docking_center_tolerance_m": 0.001})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_docking_center_tolerance_m": 0.051})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_docking_center_release_hysteresis_m": 0.009})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_docking_center_release_hysteresis_m": 0.51})
            with self.assertRaises(ValueError):
                update_console_config(path, {"docking_timeout_s": -1})
            with self.assertRaises(ValueError):
                update_console_config(path, {"docking_timeout_s": 601})
            with self.assertRaises(ValueError):
                update_console_config(path, {"docking_timeout_s": float("nan")})
            with self.assertRaises(ValueError):
                update_console_config(path, {"docking_timeout_s": float("inf")})
            with self.assertRaises(ValueError):
                update_console_config(
                    path,
                    {
                        "pre_align_buoyancy_hold_pwm": 1710,
                        "pre_align_down_pwm_max": 1700,
                    },
                )

            updated = update_console_config(
                path,
                {
                    "pre_align_buoyancy_hold_pwm": 2000,
                    "pre_align_down_pwm_max": 2000,
                    "docking_timeout_s": 0,
                },
            )
            self.assertEqual(updated["pre_align_buoyancy_hold_pwm"], 2000)
            self.assertEqual(updated["pre_align_down_pwm_max"], 2000)
            self.assertEqual(updated["docking_timeout_s"], 0.0)

    def test_velocity_limit_fields_allow_one_meter_per_second_but_no_more(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "settings.yaml"
            path.write_text(SAMPLE_SETTINGS, encoding="utf-8")

            updated = update_console_config(
                path,
                {
                    "max_v_m_s": 1.0,
                    "pre_align_max_v_m_s": 1.0,
                    "pid.forward.output_limit": 1.0,
                    "pid.right.output_limit": 1.0,
                    "pid.up.output_limit": 1.0,
                },
            )

            self.assertEqual(updated["max_v_m_s"], 1.0)
            self.assertEqual(updated["pre_align_max_v_m_s"], 1.0)
            self.assertEqual(updated["pid.forward.output_limit"], 1.0)
            self.assertEqual(updated["pid.right.output_limit"], 1.0)
            self.assertEqual(updated["pid.up.output_limit"], 1.0)

            with self.assertRaises(ValueError):
                update_console_config(path, {"max_v_m_s": 1.01})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pre_align_max_v_m_s": 1.01})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pid.forward.output_limit": 1.01})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pid.right.output_limit": 1.01})
            with self.assertRaises(ValueError):
                update_console_config(path, {"pid.up.output_limit": 1.01})


if __name__ == "__main__":
    unittest.main()
