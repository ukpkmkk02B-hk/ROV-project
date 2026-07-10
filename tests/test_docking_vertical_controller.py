import unittest

from modules.controller.docking_vertical_controller import DockingVerticalController


class DockingVerticalControllerTests(unittest.TestCase):
    def make_controller(self, **overrides):
        config = {
            "pre_align_buoyancy_hold_pwm": 1600,
            "pre_align_down_pwm_max": 1700,
            "pre_align_target_approach_speed_m_s": 0.03,
            "pre_align_approach_speed_kp": 2000.0,
            "pre_dock_approach_speed_tolerance_m_s": 0.01,
        }
        config.update(overrides)
        return DockingVerticalController(config)

    def test_negative_camera_vz_is_positive_approach_speed(self):
        controller = self.make_controller()
        controller.reset(timestamp=0.0, initial_pwm=1600)

        result = controller.update(
            camera_vz_m_s=-0.02,
            desired_up_m_s=-0.03,
            vertical_allowed=True,
            timestamp=1.0,
        )

        self.assertAlmostEqual(result["pre_align_raw_approach_speed_m_s"], 0.02)
        self.assertAlmostEqual(result["pre_align_filtered_approach_speed_m_s"], 0.02)
        self.assertAlmostEqual(result["pre_align_target_approach_speed_m_s"], 0.03)
        self.assertEqual(result["pre_align_vertical_pwm"], 1620)

    def test_pwm_returns_to_hold_at_zero_target_and_zero_speed(self):
        controller = self.make_controller()
        controller.reset(timestamp=0.0, initial_pwm=1500)

        result = controller.update(
            camera_vz_m_s=0.0,
            desired_up_m_s=0.0,
            vertical_allowed=True,
            timestamp=1.0,
        )

        self.assertEqual(result["pre_align_vertical_pwm"], 1600)
        self.assertTrue(result["pre_align_buoyancy_hold_active"])
        self.assertTrue(result["pre_dock_approach_speed_ok"])

    def test_vertical_gate_holds_depth_without_requesting_approach(self):
        controller = self.make_controller()
        controller.reset(timestamp=0.0, initial_pwm=1600)

        result = controller.update(
            camera_vz_m_s=0.0,
            desired_up_m_s=-0.05,
            vertical_allowed=False,
            timestamp=1.0,
        )

        self.assertEqual(result["pre_align_target_approach_speed_m_s"], 0.0)
        self.assertEqual(result["pre_align_vertical_pwm"], 1600)
        self.assertTrue(result["pre_align_buoyancy_hold_active"])

    def test_too_slow_increases_pwm_and_too_fast_reduces_pwm(self):
        slow = self.make_controller(pre_align_approach_speed_kp=1000.0)
        slow.reset(timestamp=0.0, initial_pwm=1600)
        slow_result = slow.update(-0.01, -0.03, True, timestamp=1.0)

        fast = self.make_controller(pre_align_approach_speed_kp=1000.0)
        fast.reset(timestamp=0.0, initial_pwm=1600)
        fast_result = fast.update(-0.05, -0.03, True, timestamp=1.0)

        self.assertGreater(slow_result["pre_align_vertical_pwm"], 1600)
        self.assertLess(fast_result["pre_align_vertical_pwm"], 1600)

    def test_output_clamp_and_slew_rate_are_applied(self):
        controller = self.make_controller(pre_align_approach_speed_kp=5000.0)
        controller.reset(timestamp=0.0, initial_pwm=1500)

        first = controller.update(0.0, -0.03, True, timestamp=0.5)
        second = controller.update(0.0, -0.03, True, timestamp=1.0)

        self.assertEqual(first["pre_align_vertical_pwm"], 1550)
        self.assertEqual(second["pre_align_vertical_pwm"], 1600)
        self.assertTrue(second["pre_align_pwm_saturated_high"])
        self.assertLessEqual(second["pre_align_vertical_pwm"], 1700)

    def test_ema_and_ready_tolerance_use_filtered_speed(self):
        controller = self.make_controller()
        controller.reset(timestamp=0.0, initial_pwm=1600)
        controller.update(0.0, 0.0, True, timestamp=1.0)

        result = controller.update(-0.05, 0.0, True, timestamp=2.0)

        self.assertAlmostEqual(result["pre_align_raw_approach_speed_m_s"], 0.05)
        self.assertAlmostEqual(result["pre_align_filtered_approach_speed_m_s"], 0.01)
        self.assertTrue(result["pre_dock_approach_speed_ok"])

    def test_invalid_pwm_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            self.make_controller(pre_align_buoyancy_hold_pwm=1701, pre_align_down_pwm_max=1700)
        with self.assertRaises(ValueError):
            self.make_controller(pre_align_buoyancy_hold_pwm=1600.5)
        with self.assertRaises(ValueError):
            self.make_controller(pre_align_down_pwm_max=1801)

    def test_post_confirm_hold_reports_and_outputs_exact_hold_pwm(self):
        controller = self.make_controller(pre_align_buoyancy_hold_pwm=1610)
        controller.update(0.0, -0.03, True, timestamp=1.0)

        status = controller.activate_buoyancy_hold()

        self.assertEqual(status["pre_align_vertical_pwm"], 1610)
        self.assertEqual(status["pre_align_vertical_pwm_raw"], 1610.0)
        self.assertTrue(status["pre_align_buoyancy_hold_active"])
        self.assertFalse(status["pre_align_pwm_saturated_low"])
        self.assertFalse(status["pre_align_pwm_saturated_high"])

    def test_non_finite_speed_fails_safe_and_filter_recovers(self):
        controller = self.make_controller()

        invalid = controller.update(float("nan"), -0.03, True, timestamp=1.0)
        recovered = controller.update(0.0, 0.0, False, timestamp=1.05)

        self.assertEqual(invalid["pre_align_vertical_pwm"], 1500)
        self.assertFalse(invalid["pre_align_input_valid"])
        self.assertFalse(invalid["pre_dock_approach_speed_ok"])
        self.assertEqual(invalid["pre_align_invalid_reason"], "non_finite_input")
        self.assertTrue(recovered["pre_align_input_valid"])
        self.assertEqual(recovered["pre_align_filtered_approach_speed_m_s"], 0.0)
        self.assertEqual(recovered["pre_align_vertical_pwm"], 1505)


if __name__ == "__main__":
    unittest.main()
