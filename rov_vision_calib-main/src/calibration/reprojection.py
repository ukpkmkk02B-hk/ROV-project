from __future__ import annotations

import cv2
import numpy as np


def image_reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    observed = image_points.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((observed - projected) ** 2, axis=1))))


def per_image_errors(object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs) -> list[float]:
    return [
        image_reprojection_error(obj, img, rvec, tvec, camera_matrix, dist_coeffs)
        for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs)
    ]
