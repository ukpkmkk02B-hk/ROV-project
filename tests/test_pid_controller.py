import unittest

from modules.controller.pid_controller import MultiAxisPidController, PidAxisController


class PidAxisControllerTests(unittest.TestCase):
    def test_axis_controller_combines_p_i_d_terms_with_limits(self):
        controller = PidAxisController(kp=2.0, ki=0.5, kd=0.25, integral_limit=1.0, output_limit=10.0)

        first = controller.update(error=1.0, timestamp=1.0)
        second = controller.update(error=0.5, timestamp=3.0)

        self.assertEqual(first.output, 2.0)
        self.assertEqual(first.p, 2.0)
        self.assertEqual(first.i, 0.0)
        self.assertEqual(first.d, 0.0)
        self.assertAlmostEqual(second.p, 1.0)
        self.assertAlmostEqual(second.i, 0.5)
        self.assertAlmostEqual(second.d, -0.0625)
        self.assertAlmostEqual(second.output, 1.4375)

    def test_axis_controller_clamps_integral_and_output(self):
        controller = PidAxisController(kp=0.0, ki=10.0, kd=0.0, integral_limit=0.2, output_limit=1.0)

        controller.update(error=1.0, timestamp=1.0)
        result = controller.update(error=1.0, timestamp=2.0)

        self.assertAlmostEqual(result.integral, 0.2)
        self.assertAlmostEqual(result.i, 2.0)
        self.assertAlmostEqual(result.output, 1.0)

    def test_axis_controller_reset_clears_history(self):
        controller = PidAxisController(kp=1.0, ki=1.0, kd=1.0, integral_limit=10.0, output_limit=10.0)

        controller.update(error=1.0, timestamp=1.0)
        controller.update(error=0.0, timestamp=2.0)
        controller.reset()
        result = controller.update(error=0.5, timestamp=10.0)

        self.assertEqual(result.i, 0.0)
        self.assertEqual(result.d, 0.0)
        self.assertEqual(result.output, 0.5)


class MultiAxisPidControllerTests(unittest.TestCase):
    def test_multi_axis_controller_returns_outputs_and_prefixed_diagnostics(self):
        controller = MultiAxisPidController(
            {
                "forward": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "yaw": {"kp": 2.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.2},
            }
        )

        outputs, diagnostics = controller.update({"forward": 0.5, "yaw": -0.2}, timestamp=1.0)

        self.assertEqual(outputs["forward"], 0.4)
        self.assertEqual(outputs["yaw"], -0.2)
        self.assertEqual(diagnostics["pid_forward_error"], 0.5)
        self.assertEqual(diagnostics["pid_forward_p"], 0.5)
        self.assertEqual(diagnostics["pid_forward_output"], 0.4)
        self.assertEqual(diagnostics["pid_yaw_error"], -0.2)
        self.assertEqual(diagnostics["pid_yaw_p"], -0.4)
        self.assertEqual(diagnostics["pid_yaw_output"], -0.2)


if __name__ == "__main__":
    unittest.main()
