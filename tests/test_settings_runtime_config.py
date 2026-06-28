import unittest
import re
from pathlib import Path


class RuntimeSettingsTests(unittest.TestCase):
    def test_vision_tracking_uses_safe_rc_override_defaults(self):
        text = Path("config/settings.yaml").read_text(encoding="utf-8")

        self.assertIn("enable_motion: false", text)
        self.assertIn('output_backend: "rc_override"', text)
        self.assertIn('required_mode: "STABILIZE"', text)
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


if __name__ == "__main__":
    unittest.main()
