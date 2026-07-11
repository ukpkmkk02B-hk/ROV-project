import math
import unittest

from modules.controller.visual_axis_policy import VisualAxisPolicy


def base_command():
    return {
        "forward_m_s": 0.4,
        "right_m_s": -0.2,
        "up_m_s": 0.08,
        "yaw_rate_rad_s": 0.30,
        "vx": 0.4,
        "vy": -0.2,
        "vz": 0.08,
        "yaw_rate": 0.30,
        "v_yaw": 0.30,
    }


class VisualAxisPolicyTests(unittest.TestCase):
    def test_default_tracking_vertical_mode_disables_vertical_motion_in_tracking(self):
        policy = VisualAxisPolicy({})

        adjusted = policy.apply(base_command(), stage="track")

        self.assertAlmostEqual(adjusted["forward_m_s"], 0.4)
        self.assertAlmostEqual(adjusted["right_m_s"], -0.2)
        self.assertAlmostEqual(adjusted["up_m_s"], 0.0)
        self.assertAlmostEqual(adjusted["yaw_rate_rad_s"], 0.30)
        self.assertEqual(adjusted["tracking_vertical_mode"], "disabled")
        self.assertTrue(adjusted["vertical_disabled_active"])

    def test_removed_visual_pid_mode_is_rejected_and_vertical_stays_disabled(self):
        policy = VisualAxisPolicy(
            {
                "tracking_vertical_mode": "visual_pid",
                "pre_align_axis_mode": "full_control",
            }
        )

        adjusted = policy.apply(base_command(), stage="track")

        self.assertAlmostEqual(adjusted["forward_m_s"], 0.4)
        self.assertAlmostEqual(adjusted["right_m_s"], -0.2)
        self.assertAlmostEqual(adjusted["up_m_s"], 0.0)
        self.assertAlmostEqual(adjusted["yaw_rate_rad_s"], 0.30)
        self.assertEqual(adjusted["tracking_vertical_mode"], "disabled")
        self.assertEqual(adjusted["tracking_vertical_rejected_reason"], "invalid_tracking_vertical_mode")
        self.assertEqual(adjusted["pre_align_axis_mode"], "full_control")

    def test_hold_captured_ch3_zeroes_vertical_command_and_overrides_rc_ch3(self):
        policy = VisualAxisPolicy({}, up_channel="ch3")
        policy.capture_ch3(1534, timestamp=10.0, source="unit_test")
        result = policy.set_tracking_vertical_mode("hold_captured_ch3")

        adjusted = policy.apply(base_command(), stage="track")
        rc = policy.apply_rc_override({"ch3": 1520, "ch5": 1600}, stage="track")

        self.assertTrue(result["accepted"])
        self.assertAlmostEqual(adjusted["up_m_s"], 0.0)
        self.assertAlmostEqual(adjusted["vz"], 0.0)
        self.assertEqual(rc["ch3"], 1534)
        self.assertTrue(adjusted["vertical_hold_active"])
        self.assertEqual(policy.status()["captured_hold_ch3_pwm"], 1534)

    def test_rejects_hold_captured_ch3_mode_without_captured_value(self):
        policy = VisualAxisPolicy({})

        result = policy.set_tracking_vertical_mode("hold_captured_ch3")

        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"], "ch3_not_captured")
        self.assertEqual(policy.status()["tracking_vertical_mode"], "disabled")

    def test_pre_align_small_correction_scales_and_limits_horizontal_axes_and_vertical_descent(self):
        policy = VisualAxisPolicy(
            {
                "pre_align_axis_mode": "small_correction",
                "pre_align_correction_scale": 0.25,
                "pre_align_max_v_m_s": 0.05,
                "pre_align_max_yaw_rate_deg_s": 3.0,
            }
        )

        adjusted = policy.apply(base_command(), stage="pre_align")

        self.assertAlmostEqual(adjusted["forward_m_s"], 0.05)
        self.assertAlmostEqual(adjusted["right_m_s"], -0.05)
        self.assertAlmostEqual(adjusted["up_m_s"], 0.05)
        self.assertAlmostEqual(adjusted["yaw_rate_rad_s"], math.radians(3.0))
        self.assertTrue(adjusted["pre_align_horizontal_scaled"])

    def test_legacy_lock_horizontal_falls_back_to_small_correction(self):
        policy = VisualAxisPolicy({"pre_align_axis_mode": "lock_horizontal"})

        adjusted = policy.apply(base_command(), stage="pre_align")

        self.assertEqual(policy.pre_align_axis_mode, "small_correction")
        self.assertEqual(policy.pre_align_mode_rejected_reason, "invalid_pre_align_axis_mode")
        self.assertAlmostEqual(adjusted["forward_m_s"], 0.05)
        self.assertAlmostEqual(adjusted["right_m_s"], -0.05)
        self.assertAlmostEqual(adjusted["up_m_s"], 0.05)
        self.assertAlmostEqual(adjusted["yaw_rate_rad_s"], math.radians(3.0))
        self.assertFalse(adjusted["pre_align_horizontal_locked"])

        result = policy.set_pre_align_axis_mode("lock_horizontal")
        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"], "invalid_pre_align_axis_mode")


if __name__ == "__main__":
    unittest.main()
