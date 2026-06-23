import math
import unittest

import numpy as np

from modules.perception.marker_tracker import (
    build_square_object_points,
    compute_marker_pixel_size,
    new_tracker_stats,
    update_tracker_stats,
    make_pose_dict,
    rotation_matrix_to_euler_deg,
    validate_pose_quality,
)


class MarkerTrackerHelperTests(unittest.TestCase):
    def test_build_square_object_points_uses_marker_size_in_meters(self):
        points = build_square_object_points(0.04)

        self.assertEqual(points.shape, (4, 3))
        np.testing.assert_allclose(
            points,
            np.array(
                [
                    [-0.02, 0.02, 0.0],
                    [0.02, 0.02, 0.0],
                    [0.02, -0.02, 0.0],
                    [-0.02, -0.02, 0.0],
                ],
                dtype=np.float32,
            ),
        )

    def test_rotation_matrix_to_euler_deg_reports_yaw_in_degrees(self):
        yaw_rad = math.radians(30.0)
        rotation = np.array(
            [
                [math.cos(yaw_rad), -math.sin(yaw_rad), 0.0],
                [math.sin(yaw_rad), math.cos(yaw_rad), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        roll, pitch, yaw = rotation_matrix_to_euler_deg(rotation)

        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(yaw, 30.0, places=6)

    def test_make_pose_dict_includes_required_tracking_fields(self):
        pose = make_pose_dict(
            marker_id=20,
            tvec=np.array([[0.1], [-0.2], [0.8]], dtype=np.float64),
            rvec=np.zeros((3, 1), dtype=np.float64),
            center=(320.0, 240.0),
            timestamp=123.5,
            euler_deg=(1.0, 2.0, 3.0),
        )

        self.assertEqual(pose["id"], 20)
        self.assertEqual(pose["x"], 0.1)
        self.assertEqual(pose["y"], -0.2)
        self.assertEqual(pose["z"], 0.8)
        self.assertEqual(pose["roll"], 1.0)
        self.assertEqual(pose["pitch"], 2.0)
        self.assertEqual(pose["yaw"], 3.0)
        self.assertEqual(pose["center_u"], 320.0)
        self.assertEqual(pose["center_v"], 240.0)
        self.assertEqual(pose["timestamp"], 123.5)
        self.assertIs(pose["detected"], True)

    def test_compute_marker_pixel_size_reports_average_edge_length(self):
        corners = np.array(
            [
                [10.0, 20.0],
                [50.0, 20.0],
                [50.0, 60.0],
                [10.0, 60.0],
            ],
            dtype=np.float32,
        )

        self.assertAlmostEqual(compute_marker_pixel_size(corners), 40.0)

    def test_validate_pose_quality_accepts_good_pose_and_records_valid_state(self):
        pose = {
            "x": 0.1,
            "y": -0.1,
            "z": 0.8,
            "yaw": 20.0,
            "marker_pixel_size_px": 80.0,
            "reprojection_error_px": 1.2,
        }

        self.assertTrue(validate_pose_quality(pose, {"max_reprojection_error_px": 5.0}))
        self.assertTrue(pose["pose_valid"])
        self.assertEqual(pose["reject_reason"], "")

    def test_validate_pose_quality_rejects_outliers_small_markers_and_bad_reprojection(self):
        config = {
            "max_abs_position_m": 5.0,
            "max_abs_yaw_deg": 180.0,
            "min_marker_pixel_size_px": 20.0,
            "max_reprojection_error_px": 4.0,
        }

        cases = [
            ({"x": 100.0, "y": 0.0, "z": 0.8, "yaw": 0.0}, "position_out_of_range"),
            ({"x": 0.0, "y": 0.0, "z": -0.1, "yaw": 0.0}, "z_out_of_range"),
            ({"x": 0.0, "y": 0.0, "z": 0.8, "yaw": 2875.0}, "yaw_out_of_range"),
            (
                {"x": 0.0, "y": 0.0, "z": 0.8, "yaw": 0.0, "marker_pixel_size_px": 12.0},
                "marker_too_small",
            ),
            (
                {"x": 0.0, "y": 0.0, "z": 0.8, "yaw": 0.0, "reprojection_error_px": 7.5},
                "reprojection_error_too_high",
            ),
        ]

        for pose, reason in cases:
            with self.subTest(reason=reason):
                self.assertFalse(validate_pose_quality(pose, config))
                self.assertFalse(pose["pose_valid"])
                self.assertEqual(pose["reject_reason"], reason)

    def test_tracker_stats_count_frame_level_results(self):
        stats = new_tracker_stats()

        update_tracker_stats(stats, "no_marker", detected_ids=[], target_id=20, timestamp=1.0)
        update_tracker_stats(stats, "target_id_not_found", detected_ids=[3], target_id=20, timestamp=2.0)
        update_tracker_stats(stats, "valid_pose", detected_ids=[20], target_id=20, timestamp=3.0)
        update_tracker_stats(stats, "quality_rejected", detected_ids=[20], target_id=20, timestamp=4.0)

        self.assertEqual(stats["tracker_frames_processed"], 4)
        self.assertEqual(stats["tracker_marker_frames"], 3)
        self.assertEqual(stats["tracker_target_frames"], 2)
        self.assertEqual(stats["tracker_valid_pose_frames"], 1)
        self.assertEqual(stats["tracker_invalid_pose_frames"], 1)
        self.assertEqual(stats["tracker_no_marker_frames"], 1)
        self.assertEqual(stats["tracker_target_id_missing_frames"], 1)
        self.assertEqual(stats["tracker_quality_rejected_frames"], 1)
        self.assertEqual(stats["tracker_last_frame_timestamp"], 4.0)
        self.assertEqual(stats["tracker_last_valid_pose_timestamp"], 3.0)


if __name__ == "__main__":
    unittest.main()
