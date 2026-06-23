import math
import unittest

from modules.controller.visual_tracking_controller import VisualTrackingController


class VisualTrackingControllerTests(unittest.TestCase):
    def test_compute_command_maps_camera_errors_to_limited_body_velocity(self):
        controller = VisualTrackingController(
            desired_z_m=0.8,
            max_v_m_s=0.4,
            max_yaw_rate_deg_s=25.0,
            kp_lateral=2.0,
            kp_vertical=2.0,
            kp_distance=2.0,
            kp_yaw=2.0,
        )

        command = controller.compute_command({"x": 1.0, "y": -1.0, "z": 1.2, "yaw": 30.0})

        self.assertEqual(command["vx"], 0.4)
        self.assertEqual(command["vy"], -0.4)
        self.assertEqual(command["vz"], 0.4)
        self.assertAlmostEqual(command["yaw_rate"], -math.radians(25.0))
        self.assertAlmostEqual(command["v_yaw"], command["yaw_rate"])

    def test_pre_dock_ready_requires_distance_centering_and_yaw_tolerance(self):
        controller = VisualTrackingController(
            desired_z_m=0.8,
            pre_dock_position_tolerance_m=0.05,
            pre_dock_distance_tolerance_m=0.05,
            pre_dock_yaw_tolerance_deg=5.0,
        )

        self.assertTrue(controller.is_pre_dock_ready({"x": 0.02, "y": -0.03, "z": 0.82, "yaw": 4.0}))
        self.assertFalse(controller.is_pre_dock_ready({"x": 0.20, "y": -0.03, "z": 0.82, "yaw": 4.0}))
        self.assertFalse(controller.is_pre_dock_ready({"x": 0.02, "y": -0.03, "z": 1.00, "yaw": 4.0}))
        self.assertFalse(controller.is_pre_dock_ready({"x": 0.02, "y": -0.03, "z": 0.82, "yaw": 8.0}))


if __name__ == "__main__":
    unittest.main()
