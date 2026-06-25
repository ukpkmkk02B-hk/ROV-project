import unittest
from pathlib import Path


class RuntimeSettingsTests(unittest.TestCase):
    def test_vision_tracking_uses_safe_rc_override_defaults(self):
        text = Path("config/settings.yaml").read_text(encoding="utf-8")

        self.assertIn("enable_motion: false", text)
        self.assertIn('output_backend: "rc_override"', text)
        self.assertIn('required_mode: "STABILIZE"', text)
        self.assertIn('control_mode: "pid"', text)
        self.assertIn("pid:", text)
        self.assertIn("forward:", text)
        self.assertIn("right:", text)
        self.assertIn("up:", text)
        self.assertIn("yaw:", text)
        self.assertIn("rc_override:", text)
        self.assertIn("enabled: true", text)
        self.assertIn('forward: "ch5"', text)
        self.assertIn('right: "ch6"', text)
        self.assertIn('up: "ch3"', text)
        self.assertIn('yaw: "ch4"', text)


if __name__ == "__main__":
    unittest.main()
