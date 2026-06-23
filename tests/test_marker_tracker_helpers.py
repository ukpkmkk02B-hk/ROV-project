import math
import unittest

import numpy as np

from modules.perception.marker_tracker import (
    build_square_object_points,
    make_pose_dict,
    rotation_matrix_to_euler_deg,
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


if __name__ == "__main__":
    unittest.main()
