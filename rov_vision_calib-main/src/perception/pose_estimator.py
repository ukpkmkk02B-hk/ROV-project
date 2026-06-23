from __future__ import annotations

import cv2
import numpy as np


def estimate_single_marker_pose(corners, marker_size_m: float, camera_matrix, dist_coeffs):
    half = marker_size_m / 2.0
    object_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )
    image_points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        raise RuntimeError("solvePnP failed for ArUco marker")
    return rvec, tvec


def yaw_from_rvec_deg(rvec) -> float:
    rotation, _ = cv2.Rodrigues(rvec)
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    return float(np.degrees(yaw))
