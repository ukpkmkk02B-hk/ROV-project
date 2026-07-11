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
                "tracking mode disabled",
                "tracking capture ch3",
                "tracking mode hold_captured_ch3",
                "docking start",
                "docking confirm",
                "docking stop",
                "prealign mode full_control",
                "prealign mode small_correction",
                "up",
                "forward",
                "down",
                "left",
                "turn_left",
                "turn_right",
                "right",
                "backward",
            },
        )

    def test_surface_console_has_manual_turn_left_and_right_buttons(self):
        html = Path("tools/surface_console/static/index.html").read_text(encoding="utf-8")

        self.assertIn('data-rov="turn_left"', html)
        self.assertIn('data-rov="turn_right"', html)
        self.assertIn("左转", html)
        self.assertIn("Turn Left", html)
        self.assertIn("右转", html)
        self.assertIn("Turn Right", html)
        self.assertIn('data-rov="left"', html)
        self.assertIn('data-rov="right"', html)
        self.assertIn('data-rov="stop"', html)

    def test_surface_console_exposes_only_supported_tracking_vertical_modes(self):
        html = Path("tools/surface_console/static/index.html").read_text(encoding="utf-8")
        js = Path("tools/surface_console/static/app.js").read_text(encoding="utf-8")

        self.assertIn('data-rov="tracking mode disabled"', html)
        self.assertIn("Disable Vertical", html)
        self.assertIn('<option value="disabled">disabled</option>', html)
        self.assertIn('<option value="hold_captured_ch3">hold_captured_ch3</option>', html)
        self.assertIn('LABELS.verticalMode.disabled = "关闭升沉 / disabled";', js)
        self.assertNotIn("visual_pid", html)
        self.assertNotIn("visual_pid", js)
        self.assertNotIn("desired_z_m", html)

    def test_surface_console_exposes_only_supported_pre_align_modes(self):
        html = Path("tools/surface_console/static/index.html").read_text(encoding="utf-8")
        js = Path("tools/surface_console/static/app.js").read_text(encoding="utf-8")

        self.assertIn('data-rov="prealign mode full_control"', html)
        self.assertIn('data-rov="prealign mode small_correction"', html)
        self.assertNotIn("lock_horizontal", html)
        self.assertNotIn("lock_horizontal", js)

    def test_surface_console_manual_motion_is_three_by_three_with_turns_on_backward_row(self):
        html = Path("tools/surface_console/static/index.html").read_text(encoding="utf-8")
        css = Path("tools/surface_console/static/styles.css").read_text(encoding="utf-8")
        motion_pad = re.search(r'<div class="motion-pad">(.*?)</div>', html, re.S).group(1)
        commands = re.findall(r'data-rov="([^"]+)"', motion_pad)

        self.assertEqual(len(commands), 9)
        self.assertIn('grid-template-columns: repeat(3, minmax(94px, 1fr));', css)
        self.assertIn(".motion-pad > span", css)
        self.assertIn("display: none;", css)
        self.assertIn('.motion-pad [data-rov="turn_left"]', css)
        self.assertIn("order: 7;", css)
        self.assertIn('.motion-pad [data-rov="backward"]', css)
        self.assertIn("order: 8;", css)
        self.assertIn('.motion-pad [data-rov="turn_right"]', css)
        self.assertIn("order: 9;", css)

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

    def test_surface_console_exposes_docking_vertical_tuning_and_diagnostics(self):
        html = Path("tools/surface_console/static/index.html").read_text(encoding="utf-8")
        js = Path("tools/surface_console/static/app.js").read_text(encoding="utf-8")

        for field in (
            "pre_align_buoyancy_hold_pwm",
            "pre_align_down_pwm_max",
            "pre_align_target_approach_speed_m_s",
            "pre_align_approach_speed_kp",
            "pre_dock_approach_speed_tolerance_m_s",
            "pre_align_close_loss_hold_max_distance_m",
            "pre_align_docking_center_offset_camera_x_m",
            "pre_align_docking_center_offset_camera_y_m",
            "pre_align_docking_center_tolerance_m",
            "pre_align_docking_center_release_hysteresis_m",
            "docking_timeout_s",
        ):
            self.assertIn(f'name="{field}"', html)

        self.assertIn('name="pre_align_buoyancy_hold_pwm" type="number" min="1500" max="2000"', html)
        self.assertIn('name="pre_align_down_pwm_max" type="number" min="1500" max="2000"', html)

        for element_id in (
            "dockingApproachRaw",
            "dockingApproachFiltered",
            "dockingApproachTarget",
            "dockingApproachError",
            "dockingCh3",
            "dockingHoldActive",
            "dockingPwmLimit",
            "dockedHoldActive",
            "dockingLostHoldActive",
            "dockingLostHoldLastZ",
            "dockingLostHoldCh3",
            "dockingLostHoldReason",
            "dockingLostHoldWaiting",
            "dockingCenterOffsetActive",
            "dockingCenterTargetX",
            "dockingCenterTargetY",
            "dockingCenterTargetForward",
            "dockingCenterTargetRight",
            "dockingCenterPositionOk",
            "dockingCenterErrorX",
            "dockingCenterErrorY",
            "dockingCenterTolerance",
            "dockingCenterAlignmentOk",
            "dockingCenterReleaseDistance",
            "dockingCenterReleaseReason",
        ):
            self.assertIn(f'id="{element_id}"', html)
            self.assertIn(f'"{element_id}"', js)

        self.assertIn('docked_hold: "对接浮力保持 / docked_hold"', js)
        self.assertIn('docking_lost_hold: "近距离丢失保持 / docking_lost_hold"', js)
        self.assertIn("Restart main.py", html)


if __name__ == "__main__":
    unittest.main()
