import unittest
import re
from pathlib import Path

import yaml

from modules.controller.motion_command import MotionCommand
from modules.controller.rc_override_mapper import RcOverrideMapper


class RuntimeSettingsTests(unittest.TestCase):
    def test_vision_tracking_uses_safe_rc_override_defaults(self):
        text = Path("config/settings.yaml").read_text(encoding="utf-8")

        self.assertIn("enable_motion: false", text)
        self.assertIn('output_backend: "rc_override"', text)
        self.assertIn('required_mode: "MANUAL"', text)
        self.assertIn('control_mode: "pid"', text)
        self.assertIn('target_motion_mode: "stationary_child"', text)
        self.assertIn('child_command_mode: "disabled"', text)
        self.assertIn("control_deadband_m: 0.01", text)
        self.assertIn("yaw_deadband_deg: 1.0", text)
        self.assertIn("command_smoothing_alpha: 0.6", text)
        self.assertIn("start_charging_after_dock: false", text)
        self.assertIn("max_yaw_rate_deg_s: 10.0", text)
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

    def test_visual_rc_override_reverses_vertical_axis_for_current_vehicle(self):
        config = yaml.safe_load(Path("config/settings.yaml").read_text(encoding="utf-8"))
        rc_config = config["vision_tracking"]["rc_override"]

        self.assertEqual(rc_config["axis_signs"]["up"], -1.0)

        mapper = RcOverrideMapper(rc_config)
        up_channels = mapper.map_motion_command(MotionCommand(up_m_s=0.1))
        down_channels = mapper.map_motion_command(MotionCommand(up_m_s=-0.1))

        self.assertLess(up_channels["ch3"], rc_config["neutral_pwm"])
        self.assertGreater(down_channels["ch3"], rc_config["neutral_pwm"])

    def test_visual_rc_override_does_not_drive_roll_or_pitch_inputs(self):
        config = yaml.safe_load(Path("config/settings.yaml").read_text(encoding="utf-8"))
        rc_config = config["vision_tracking"]["rc_override"]

        mapper = RcOverrideMapper(rc_config)
        channels = mapper.map_motion_command(
            MotionCommand(forward_m_s=0.1, right_m_s=0.1, up_m_s=0.1, yaw_rate_rad_s=0.1)
        )

        self.assertEqual(channels["ch1"], rc_config["neutral_pwm"])
        self.assertEqual(channels["ch2"], rc_config["neutral_pwm"])


if __name__ == "__main__":
    unittest.main()
