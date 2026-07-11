import math
import unittest

from modules.controller.visual_tracking_controller import VisualTrackingController


class VisualTrackingControllerTests(unittest.TestCase):
    def test_compute_command_maps_camera_errors_to_limited_body_velocity(self):
        controller = VisualTrackingController(
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

    def test_pid_mode_uses_pid_outputs_and_reports_diagnostics(self):
        controller = VisualTrackingController(
            max_v_m_s=0.4,
            max_yaw_rate_deg_s=25.0,
            control_mode="pid",
            pid_config={
                "forward": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "right": {"kp": 2.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "up": {"kp": 3.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "yaw": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": math.radians(25.0)},
            },
        )

        command = controller.compute_command(
            {
                "forward_m": 1.0,
                "right_m": -0.1,
                "up_m": 0.05,
                "yaw_error_deg": 10.0,
                "timestamp": 1.0,
            }
        )

        self.assertAlmostEqual(command["forward_m_s"], 0.4)
        self.assertAlmostEqual(command["right_m_s"], -0.2)
        self.assertAlmostEqual(command["up_m_s"], 0.15)
        self.assertAlmostEqual(command["yaw_rate_rad_s"], -math.radians(10.0))
        self.assertAlmostEqual(command["pid_forward_error"], 1.0)
        self.assertAlmostEqual(command["pid_right_error"], -0.1)
        self.assertAlmostEqual(command["pid_up_error"], 0.05)
        self.assertAlmostEqual(command["pid_yaw_error"], -math.radians(10.0))
        self.assertAlmostEqual(command["pid_forward_output"], command["forward_m_s"])
        self.assertAlmostEqual(command["pid_yaw_output"], command["yaw_rate_rad_s"])

    def test_centering_mapping_drives_forward_when_marker_is_above_image_center(self):
        controller = VisualTrackingController(
            max_v_m_s=0.4,
            control_mode="pid",
            camera_to_body={
                "forward_axis": "y",
                "forward_sign": -1.0,
                "right_axis": "x",
                "right_sign": 1.0,
                "up_axis": "z",
                "up_sign": -1.0,
            },
            pid_config={
                "forward": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "right": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "up": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "yaw": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": math.radians(25.0)},
            },
        )

        centered_but_far = controller.compute_command(
            {"x": 0.0, "y": 0.0, "z": 0.8, "yaw": 90.0, "timestamp": 1.0}
        )
        marker_above_center = controller.compute_command(
            {"x": 0.0, "y": -0.12, "z": 0.5, "yaw": 90.0, "timestamp": 2.0}
        )
        marker_below_center = controller.compute_command(
            {"x": 0.0, "y": 0.12, "z": 0.5, "yaw": 90.0, "timestamp": 3.0}
        )

        self.assertEqual(centered_but_far["forward_m_s"], 0.0)
        self.assertAlmostEqual(centered_but_far["up_m_s"], -0.4)
        self.assertEqual(centered_but_far["raw_forward_error_m"], 0.0)
        self.assertAlmostEqual(centered_but_far["raw_up_error_m"], -0.8)
        self.assertAlmostEqual(marker_above_center["forward_m_s"], 0.12)
        self.assertEqual(marker_above_center["up_m_s"], -0.4)
        self.assertAlmostEqual(marker_below_center["forward_m_s"], -0.12)
        self.assertEqual(marker_below_center["up_m_s"], -0.4)

    def test_deadband_suppresses_small_stationary_target_noise_before_pid(self):
        controller = VisualTrackingController(
            control_mode="pid",
            control_deadband_m=0.02,
            yaw_deadband_deg=2.0,
            pid_config={
                "forward": {"kp": 10.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "right": {"kp": 10.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "up": {"kp": 10.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "yaw": {"kp": 10.0, "ki": 0.0, "kd": 0.0, "output_limit": math.radians(25.0)},
            },
        )

        command = controller.compute_command(
            {
                "forward_m": 0.015,
                "right_m": -0.015,
                "up_m": 0.015,
                "yaw_error_deg": 1.5,
                "timestamp": 1.0,
            }
        )

        self.assertEqual(command["forward_m_s"], 0.0)
        self.assertEqual(command["right_m_s"], 0.0)
        self.assertEqual(command["up_m_s"], 0.0)
        self.assertEqual(command["yaw_rate_rad_s"], 0.0)
        self.assertEqual(command["pid_forward_error"], 0.0)
        self.assertEqual(command["pid_right_error"], 0.0)
        self.assertEqual(command["pid_up_error"], 0.0)
        self.assertEqual(command["pid_yaw_error"], 0.0)
        self.assertAlmostEqual(command["raw_forward_error_m"], 0.015)
        self.assertAlmostEqual(command["raw_right_error_m"], -0.015)
        self.assertAlmostEqual(command["raw_up_error_m"], 0.015)
        self.assertAlmostEqual(command["raw_yaw_error_deg"], 1.5)
        self.assertEqual(command["deadbanded_forward_error_m"], 0.0)
        self.assertEqual(command["deadbanded_right_error_m"], 0.0)
        self.assertEqual(command["deadbanded_up_error_m"], 0.0)
        self.assertEqual(command["deadbanded_yaw_error_deg"], 0.0)
        self.assertEqual(command["control_deadband_m"], 0.02)
        self.assertEqual(command["yaw_deadband_deg"], 2.0)

    def test_command_smoothing_limits_single_frame_control_jumps(self):
        controller = VisualTrackingController(
            control_mode="pid",
            command_smoothing_alpha=0.5,
            pid_config={
                "forward": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "right": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "up": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "yaw": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": math.radians(25.0)},
            },
        )

        first = controller.compute_command(
            {"forward_m": 1.0, "right_m": 0.0, "up_m": 0.0, "yaw_error_deg": 0.0, "timestamp": 1.0}
        )
        second = controller.compute_command(
            {"forward_m": 1.0, "right_m": 0.0, "up_m": 0.0, "yaw_error_deg": 0.0, "timestamp": 2.0}
        )

        self.assertAlmostEqual(first["raw_motion_forward_m_s"], 0.4)
        self.assertAlmostEqual(first["forward_m_s"], 0.2)
        self.assertAlmostEqual(second["forward_m_s"], 0.3)
        self.assertEqual(first["command_smoothing_alpha"], 0.5)

    def test_pre_dock_ready_requires_distance_centering_and_yaw_tolerance(self):
        controller = VisualTrackingController(
            pre_dock_position_tolerance_m=0.05,
            pre_dock_distance_tolerance_m=0.05,
            pre_dock_yaw_tolerance_deg=5.0,
        )

        self.assertTrue(
            controller.is_pre_dock_ready(
                {"x": 0.02, "y": -0.03, "z": 0.02, "yaw": 4.0, "has_valid_observation": True}
            )
        )
        self.assertFalse(
            controller.is_pre_dock_ready(
                {"x": 0.20, "y": -0.03, "z": 0.02, "yaw": 4.0, "has_valid_observation": True}
            )
        )
        self.assertFalse(
            controller.is_pre_dock_ready(
                {"x": 0.02, "y": -0.03, "z": 0.20, "yaw": 4.0, "has_valid_observation": True}
            )
        )
        self.assertFalse(
            controller.is_pre_dock_ready(
                {"x": 0.02, "y": -0.03, "z": 0.02, "yaw": 8.0, "has_valid_observation": True}
            )
        )

    def test_pre_dock_ready_rejects_predicted_state_without_recent_observation(self):
        controller = VisualTrackingController()

        self.assertFalse(
            controller.is_pre_dock_ready(
                {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "status": "predicted", "has_valid_observation": False}
            )
        )

    def test_pre_dock_ready_allows_short_predicted_state_with_recent_observation(self):
        controller = VisualTrackingController(
            min_pre_dock_valid_frames=3,
            pre_dock_recent_observation_max_age_s=0.5,
        )

        self.assertTrue(
            controller.is_pre_dock_ready(
                {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
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
