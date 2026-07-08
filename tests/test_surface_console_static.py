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
                "MANUAL",
                "arm",
                "disarm",
                "stop",
                "tracking start",
                "tracking stop",
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

    def test_surface_console_uses_mjpeg_video_stream(self):
        html = Path("tools/surface_console/static/index.html").read_text(encoding="utf-8")
        js = Path("tools/surface_console/static/app.js").read_text(encoding="utf-8")

        self.assertIn("/api/video.mjpg?fps=25", html)
        self.assertNotIn("setInterval(refreshVideoFrame, 500)", js)

    def test_surface_console_clarifies_motion_and_restart_semantics(self):
        html = Path("tools/surface_console/static/index.html").read_text(encoding="utf-8")
        js = Path("tools/surface_console/static/app.js").read_text(encoding="utf-8")

        self.assertIn("Visual closed-loop motion output", html)
        self.assertIn("not a global manual RC lock", html)
        self.assertIn("Start Tracking does not auto Arm", html)
        self.assertIn("not a substitute for STOP or Disarm", html)
        self.assertIn("当前仅支持 MANUAL", html)
        self.assertIn("Only MANUAL mode is supported", html)
        self.assertIn("rc_override.min_active_pwm_offset", html)
        self.assertNotIn('data-rov="STABILIZE"', html)
        self.assertNotIn('data-rov="ALT_HOLD"', html)
        self.assertIn("Restart main.py to apply", js)

    def test_surface_console_preserves_unsaved_config_edits_during_refresh(self):
        js = Path("tools/surface_console/static/app.js").read_text(encoding="utf-8")

        self.assertIn("configDirty: false", js)
        self.assertIn('configForm.addEventListener("input", markConfigDirty)', js)
        self.assertIn('configForm.addEventListener("change", markConfigDirty)', js)
        self.assertIn("if (!state.configDirty) {\n    fillConfigForm(config);\n  }", js)
        self.assertIn("state.configDirty = false;\n  fillConfigForm(payload.config);", js)
        self.assertIn("async function reloadConfig()", js)
        self.assertIn(
            '$("reloadConfigBtn").addEventListener("click", () => reloadConfig().catch(handleUiError));',
            js,
        )
        self.assertNotIn('$("reloadConfigBtn").addEventListener("click", refreshStatus);', js)


if __name__ == "__main__":
    unittest.main()
