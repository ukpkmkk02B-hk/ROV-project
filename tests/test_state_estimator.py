import unittest

from modules.state.state_estimator import ConstantVelocityEKF


class ConstantVelocityEKFTests(unittest.TestCase):
    def test_update_initializes_state_and_reports_tracking(self):
        estimator = ConstantVelocityEKF(max_lost_frames=3)

        state = estimator.update({"x": 0.1, "y": -0.2, "z": 0.8, "yaw": 10.0}, timestamp=1.0)

        self.assertEqual(state["status"], "tracking")
        self.assertEqual(state["lost_frames"], 0)
        self.assertAlmostEqual(state["x"], 0.1)
        self.assertAlmostEqual(state["y"], -0.2)
        self.assertAlmostEqual(state["z"], 0.8)
        self.assertAlmostEqual(state["yaw"], 10.0)

    def test_update_estimates_velocity_from_two_measurements(self):
        estimator = ConstantVelocityEKF(max_lost_frames=3)
        estimator.update({"x": 0.0, "y": 0.0, "z": 0.8, "yaw": 0.0}, timestamp=1.0)

        state = estimator.update({"x": 0.2, "y": -0.1, "z": 0.7, "yaw": 5.0}, timestamp=1.5)

        self.assertEqual(state["status"], "tracking")
        self.assertAlmostEqual(state["vx"], 0.4)
        self.assertAlmostEqual(state["vy"], -0.2)
        self.assertAlmostEqual(state["vz"], -0.2)
        self.assertAlmostEqual(state["yaw_rate_deg_s"], 10.0)

    def test_predict_short_loss_uses_constant_velocity_then_marks_lost(self):
        estimator = ConstantVelocityEKF(max_lost_frames=2)
        estimator.update({"x": 0.0, "y": 0.0, "z": 0.8, "yaw": 0.0}, timestamp=1.0)
        estimator.update({"x": 0.2, "y": 0.0, "z": 0.8, "yaw": 10.0}, timestamp=2.0)

        predicted = estimator.predict(timestamp=3.0)
        lost = estimator.predict(timestamp=4.0)

        self.assertEqual(predicted["status"], "predicted")
        self.assertEqual(predicted["lost_frames"], 1)
        self.assertAlmostEqual(predicted["x"], 0.4)
        self.assertAlmostEqual(predicted["yaw"], 20.0)
        self.assertEqual(lost["status"], "lost")
        self.assertEqual(lost["lost_frames"], 2)

    def test_update_after_prediction_uses_last_real_measurement_for_velocity(self):
        estimator = ConstantVelocityEKF(max_lost_frames=5)
        estimator.update({"x": 0.0, "y": 0.0, "z": 0.8, "yaw": 0.0}, timestamp=1.0)
        estimator.update({"x": 0.2, "y": 0.0, "z": 0.8, "yaw": 10.0}, timestamp=2.0)
        estimator.predict(timestamp=3.0)

        state = estimator.update({"x": 0.21, "y": 0.0, "z": 0.8, "yaw": 11.0}, timestamp=3.1)

        self.assertLess(abs(state["vx"]), 0.02)
        self.assertLess(abs(state["yaw_rate_deg_s"]), 1.0)


if __name__ == "__main__":
    unittest.main()
