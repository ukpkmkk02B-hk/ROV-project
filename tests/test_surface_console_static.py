import re
import unittest
from pathlib import Path


class SurfaceConsoleStaticTests(unittest.TestCase):
    def test_surface_console_keeps_existing_rov_command_values(self):
        html = Path("tools/surface_console/static/index.html").read_text(encoding="utf-8")

        commands = set(re.findall(r'data-rov="([^"]+)"', html))

        self.assertEqual(
            commands,
            {
                "STABILIZE",
                "ALT_HOLD",
                "MANUAL",
                "arm",
                "disarm",
                "stop",
                "tracking mode visual_pid",
                "tracking capture ch3",
                "tracking mode hold_captured_ch3",
                "docking start",
                "docking confirm",
                "docking stop",
                "prealign mode full_control",
                "prealign mode small_correction",
                "prealign mode lock_horizontal",
                "up",
                "forward",
                "down",
                "left",
                "right",
                "backward",
            },
        )


if __name__ == "__main__":
    unittest.main()
