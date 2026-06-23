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

        self.assertEqual(command["forward_m_s"], 0.4)
        self.assertEqual(command["right_m_s"], 0.4)
        self.assertEqual(command["up_m_s"], 0.4)
        self.assertAlmostEqual(command["yaw_rate_rad_s"], -math.radians(25.0))
        self.assertEqual(command["vx"], command["forward_m_s"])
        self.assertEqual(command["vy"], command["right_m_s"])
        self.assertEqual(command["vz"], command["up_m_s"])
        self.assertAlmostEqual(command["v_yaw"], command["yaw_rate_rad_s"])

    def test_pre_dock_ready_requires_distance_centering_and_yaw_tolerance(self):
        controller = VisualTrackingController(
            desired_z_m=0.8,
            pre_dock_position_tolerance_m=0.05,
            pre_dock_distance_tolerance_m=0.05,
            pre_dock_yaw_tolerance_deg=5.0,
        )

        self.assertTrue(
            controller.is_pre_dock_ready(
                {"x": 0.02, "y": -0.03, "z": 0.82, "yaw": 4.0, "has_valid_observation": True}
            )
        )
        self.assertFalse(
            controller.is_pre_dock_ready(
                {"x": 0.20, "y": -0.03, "z": 0.82, "yaw": 4.0, "has_valid_observation": True}
            )
        )
        self.assertFalse(
            controller.is_pre_dock_ready(
                {"x": 0.02, "y": -0.03, "z": 1.00, "yaw": 4.0, "has_valid_observation": True}
            )
        )
        self.assertFalse(
            controller.is_pre_dock_ready(
                {"x": 0.02, "y": -0.03, "z": 0.82, "yaw": 8.0, "has_valid_observation": True}
            )
        )

    def test_pre_dock_ready_rejects_predicted_state_without_recent_observation(self):
        controller = VisualTrackingController(desired_z_m=0.8)

        self.assertFalse(
            controller.is_pre_dock_ready(
                {"x": 0.0, "y": 0.0, "z": 0.8, "yaw": 0.0, "status": "predicted", "has_valid_observation": False}
            )
        )

    def test_pre_dock_ready_allows_short_predicted_state_with_recent_observation(self):
        controller = VisualTrackingController(
            desired_z_m=0.8,
            min_pre_dock_valid_frames=3,
            pre_dock_recent_observation_max_age_s=0.5,
        )

        self.assertTrue(
            controller.is_pre_dock_ready(
                {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.8,
                    "yaw": 0.0,
                    "status": "predicted",
                    "has_valid_observation": False,
                    "has_recent_valid_observation": True,
                    "latest_pose_age_s": 0.2,
                    "pre_dock_valid_frame_count": 3,
                }
            )
        )

    def test_pre_dock_diagnostics_report_blocking_reason(self):
        controller = VisualTrackingController(
            desired_z_m=0.8,
            min_pre_dock_valid_frames=3,
            pre_dock_recent_observation_max_age_s=0.5,
        )

        expired = controller.pre_dock_diagnostics(
            {
                "x": 0.0,
                "y": 0.0,
                "z": 0.8,
                "yaw": 0.0,
                "status": "predicted",
                "has_recent_valid_observation": True,
                "latest_pose_age_s": 0.8,
                "pre_dock_valid_frame_count": 3,
            }
        )
        low_count = controller.pre_dock_diagnostics(
            {
                "x": 0.0,
                "y": 0.0,
                "z": 0.8,
                "yaw": 0.0,
                "status": "tracking",
                "has_recent_valid_observation": True,
                "latest_pose_age_s": 0.1,
                "pre_dock_valid_frame_count": 2,
            }
        )

        self.assertFalse(expired["pre_dock_ready"])
        self.assertEqual(expired["pre_dock_block_reason"], "recent_observation_expired")
        self.assertFalse(low_count["pre_dock_ready"])
        self.assertEqual(low_count["pre_dock_block_reason"], "valid_frame_count_low")


if __name__ == "__main__":
    unittest.main()
