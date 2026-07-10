import unittest
import re
from pathlib import Path

from modules.controller.motion_command import MotionCommand
from modules.controller.rc_override_mapper import RcOverrideMapper


def read_settings_text():
    return Path("config/settings.yaml").read_text(encoding="utf-8")


def runtime_rc_override_config():
    text = read_settings_text()
    required_lines = [
        "    enabled: true",
        "    neutral_pwm: 1500",
        "    min_pwm: 1400",
        "    max_pwm: 1600",
        "    pwm_per_m_s: 250",
        "    pwm_per_rad_s: 120",
        '      forward: "ch5"',
        '      right: "ch6"',
        '      up: "ch3"',
        '      yaw: "ch4"',
        "      forward: -1.0",
        "      right: -1.0",
        "      up: -1.0",
    ]
    missing = [line for line in required_lines if line not in text]
    if missing:
        raise AssertionError(f"settings.yaml missing expected rc_override lines: {missing}")
    return {
        "enabled": True,
        "neutral_pwm": 1500,
        "min_pwm": 1400,
        "max_pwm": 1600,
        "pwm_per_m_s": 250,
        "pwm_per_rad_s": 120,
        "channels": {"forward": "ch5", "right": "ch6", "up": "ch3", "yaw": "ch4"},
        "axis_signs": {"forward": -1.0, "right": -1.0, "up": -1.0},
    }


class RuntimeSettingsTests(unittest.TestCase):
    def test_vision_tracking_uses_safe_rc_override_defaults(self):
        text = read_settings_text()

        self.assertIn("enable_motion: false", text)
        self.assertIn('output_backend: "rc_override"', text)
        self.assertIn('required_mode: "MANUAL"', text)
        self.assertIn('control_mode: "pid"', text)
        self.assertIn('target_motion_mode: "stationary_child"', text)
        self.assertIn('child_command_mode: "disabled"', text)
        self.assertIn('tracking_vertical_mode: "disabled"', text)
        self.assertIn("control_deadband_m: 0.01", text)
        self.assertIn("yaw_deadband_deg: 1.0", text)
        self.assertIn("command_smoothing_alpha: 0.6", text)
        self.assertIn("start_charging_after_dock: false", text)
        self.assertIn("pre_align_buoyancy_hold_pwm: 1600", text)
        self.assertIn("pre_align_down_pwm_max: 1700", text)
        self.assertIn("pre_align_target_approach_speed_m_s: 0.03", text)
        self.assertIn("pre_align_approach_speed_kp: 2000", text)
        self.assertIn("pre_dock_approach_speed_tolerance_m_s: 0.01", text)
        self.assertIn("pre_align_close_loss_hold_max_distance_m: 0.15", text)
        self.assertIn("max_yaw_rate_deg_s: 10.0", text)
        self.assertIn('forward_axis: "y"', text)
        self.assertIn("forward_sign: -1.0", text)
        self.assertIn('up_axis: "z"', text)
        self.assertIn("pid:", text)
        self.assertIn("forward:", text)
        self.assertIn("right:", text)
        self.assertIn("up:", text)
        self.assertIn("yaw:", text)
        self.assertNotIn("kd: 0.05", text)
        self.assertNotIn("kd: 0.03", text)
        self.assertGreaterEqual(text.count("kd: 0.0"), 4)
        self.assertIn("derivative_min_dt_s: 0.08", text)
        self.assertGreaterEqual(text.count("d_limit: 0.05"), 4)
        self.assertIn("pwm_per_rad_s: 120", text)
        yaw_block = re.search(r"    yaw:\n(?P<body>(?:      .+\n)+)", text).group("body")
        self.assertIn("kp: 0.5", yaw_block)
        self.assertIn("output_limit_deg_s: 10.0", yaw_block)
        self.assertIn("d_limit: 0.05", yaw_block)
        self.assertIn("rc_override:", text)
        self.assertIn("enabled: true", text)
        self.assertIn('forward: "ch5"', text)
        self.assertIn('right: "ch6"', text)
        self.assertIn('up: "ch3"', text)
        self.assertIn('yaw: "ch4"', text)
        self.assertIn("      forward: -1.0", text)
        self.assertIn("min_active_pwm_offset: 30", text)

    def test_visual_rc_override_reverses_vertical_axis_for_current_vehicle(self):
        rc_config = runtime_rc_override_config()

        self.assertEqual(rc_config["axis_signs"]["up"], -1.0)

        mapper = RcOverrideMapper(rc_config)
        up_channels = mapper.map_motion_command(MotionCommand(up_m_s=0.1))
        down_channels = mapper.map_motion_command(MotionCommand(up_m_s=-0.1))

        self.assertLess(up_channels["ch3"], rc_config["neutral_pwm"])
        self.assertGreater(down_channels["ch3"], rc_config["neutral_pwm"])

    def test_visual_rc_override_reverses_lateral_axis_for_current_vehicle(self):
        rc_config = runtime_rc_override_config()

        self.assertEqual(rc_config["axis_signs"]["right"], -1.0)

        mapper = RcOverrideMapper(rc_config)
        right_channels = mapper.map_motion_command(MotionCommand(right_m_s=0.1))
        left_channels = mapper.map_motion_command(MotionCommand(right_m_s=-0.1))

        self.assertLess(right_channels["ch6"], rc_config["neutral_pwm"])
        self.assertGreater(left_channels["ch6"], rc_config["neutral_pwm"])

    def test_visual_rc_override_reverses_forward_axis_for_current_vehicle(self):
        rc_config = runtime_rc_override_config()

        self.assertEqual(rc_config["axis_signs"]["forward"], -1.0)

        mapper = RcOverrideMapper(rc_config)
        forward_channels = mapper.map_motion_command(MotionCommand(forward_m_s=0.1))
        backward_channels = mapper.map_motion_command(MotionCommand(forward_m_s=-0.1))

        self.assertLess(forward_channels["ch5"], rc_config["neutral_pwm"])
        self.assertGreater(backward_channels["ch5"], rc_config["neutral_pwm"])

    def test_visual_rc_override_does_not_drive_roll_or_pitch_inputs(self):
        rc_config = runtime_rc_override_config()

        mapper = RcOverrideMapper(rc_config)
        channels = mapper.map_motion_command(
            MotionCommand(forward_m_s=0.1, right_m_s=0.1, up_m_s=0.1, yaw_rate_rad_s=0.1)
        )

        self.assertEqual(channels["ch1"], rc_config["neutral_pwm"])
        self.assertEqual(channels["ch2"], rc_config["neutral_pwm"])


if __name__ == "__main__":
    unittest.main()
