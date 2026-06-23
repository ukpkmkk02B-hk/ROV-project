from __future__ import annotations

from pathlib import Path

import numpy as np

from utils.yaml_io import load_yaml


def load_calibration(path: str | Path) -> tuple[np.ndarray, np.ndarray, dict]:
    data = load_yaml(path)
    status = data.get("calibration", {}).get("status", "calibrated")
    if status != "calibrated":
        raise ValueError(f"Calibration file is not calibrated yet: {path}")

    k_data = data.get("camera_matrix", {}).get("data")
    d_data = data.get("distortion_coefficients", {}).get("data")
    if not k_data or not d_data:
        raise ValueError(f"Calibration file has empty K or D: {path}")

    camera_matrix = np.array(k_data, dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(d_data, dtype=np.float64).reshape(-1, 1)
    return camera_matrix, dist_coeffs, data


def scaled_camera_matrix(camera_matrix: np.ndarray, from_size: tuple[int, int], to_size: tuple[int, int]) -> np.ndarray:
    from_w, from_h = from_size
    to_w, to_h = to_size
    sx = to_w / from_w
    sy = to_h / from_h
    scaled = camera_matrix.copy()
    scaled[0, 0] *= sx
    scaled[1, 1] *= sy
    scaled[0, 2] *= sx
    scaled[1, 2] *= sy
    return scaled
