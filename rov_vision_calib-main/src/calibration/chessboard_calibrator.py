from __future__ import annotations

import csv
import shutil
from pathlib import Path

import cv2
import numpy as np

from calibration.reprojection import per_image_errors
from utils.time_sync import now_iso
from utils.yaml_io import load_yaml, save_yaml


IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def make_object_points(pattern_size: tuple[int, int], square_size_m: float) -> np.ndarray:
    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_m
    return objp


def find_corners(gray: np.ndarray, pattern_size: tuple[int, int]):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not ok:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def calibrate_chessboard(
    images_dir: str | Path,
    output_dir: str | Path,
    camera_config_path: str | Path,
    calib_template_path: str | Path,
) -> dict:
    images_dir = Path(images_dir)
    output_dir = Path(output_dir)
    accepted_dir = output_dir / "accepted_images"
    rejected_dir = output_dir / "rejected_images"
    debug_dir = output_dir / "corners_debug"
    for d in (accepted_dir, rejected_dir, debug_dir):
        d.mkdir(parents=True, exist_ok=True)

    camera_cfg = load_yaml(camera_config_path)
    template = load_yaml(calib_template_path)
    calib_cfg = template["calibration"]
    pattern_size = tuple(calib_cfg["inner_corners"])
    square_size_m = float(calib_cfg["square_size_m"])
    object_template = make_object_points(pattern_size, square_size_m)

    object_points = []
    image_points = []
    accepted_names = []
    rejected = []
    image_size = None

    image_paths = sorted(p for p in images_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            rejected.append({"img": path.name, "reason": "Cannot read image"})
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])
        ok, corners = find_corners(gray, pattern_size)
        if not ok:
            rejected.append({"img": path.name, "reason": "Chessboard corners not found"})
            shutil.copy2(path, rejected_dir / path.name)
            continue

        object_points.append(object_template.copy())
        image_points.append(corners)
        accepted_names.append(path.name)
        shutil.copy2(path, accepted_dir / path.name)
        debug = image.copy()
        cv2.drawChessboardCorners(debug, pattern_size, corners, ok)
        cv2.imwrite(str(debug_dir / path.name), debug)

    if len(object_points) < 10:
        raise RuntimeError(f"Only {len(object_points)} valid images found. Capture more stable chessboard images.")
    if image_size is None:
        raise RuntimeError("No readable calibration images found.")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    errors = per_image_errors(object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs)

    result = template
    result["calibration"].update(
        {
            "status": "calibrated",
            "date": now_iso(),
            "camera_name": camera_cfg["camera"].get("name"),
            "camera_sensor": camera_cfg["camera"].get("sensor"),
            "device_node": camera_cfg["camera"].get("device"),
            "image_width": image_size[0],
            "image_height": image_size[1],
            "total_images": len(image_paths),
            "valid_images": len(accepted_names),
            "rejected_images": len(rejected),
            "rejected_list": rejected,
        }
    )
    result["camera_matrix"]["data"] = [float(x) for x in camera_matrix.reshape(-1)]
    result["distortion_coefficients"]["data"] = [float(x) for x in dist_coeffs.reshape(-1)]
    result["accuracy_evaluation"] = {
        "overall_reprojection_error_px": float(np.mean(errors)),
        "opencv_rms_px": float(rms),
        "per_image_reprojection_error_px": [
            {"img": name, "error_px": float(err)} for name, err in zip(accepted_names, errors)
        ],
    }

    save_yaml(output_dir / "calib_result.yaml", result)
    write_error_csv(output_dir / "per_image_error.csv", accepted_names, errors)
    write_report_csv(output_dir / "calib_report.csv", result)
    write_experiment_log(output_dir / "experiment_log.md", result)
    return result


def write_error_csv(path: Path, names: list[str], errors: list[float]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["image", "reprojection_error_px"])
        writer.writerows(zip(names, errors))


def write_report_csv(path: Path, result: dict) -> None:
    c = result["calibration"]
    a = result["accuracy_evaluation"]
    rows = [
        ("camera_sensor", c.get("camera_sensor")),
        ("device_node", c.get("device_node")),
        ("image_width", c.get("image_width")),
        ("image_height", c.get("image_height")),
        ("total_images", c.get("total_images")),
        ("valid_images", c.get("valid_images")),
        ("rejected_images", c.get("rejected_images")),
        ("overall_reprojection_error_px", a.get("overall_reprojection_error_px")),
        ("opencv_rms_px", a.get("opencv_rms_px")),
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value"])
        writer.writerows(rows)


def write_experiment_log(path: Path, result: dict) -> None:
    c = result["calibration"]
    a = result["accuracy_evaluation"]
    text = (
        "# Calibration Experiment Log\n\n"
        f"- Date: {c.get('date')}\n"
        f"- Camera: {c.get('camera_sensor')} ({c.get('device_node')})\n"
        f"- Model: {c.get('model')}\n"
        f"- Resolution: {c.get('image_width')} x {c.get('image_height')}\n"
        f"- Board squares: {c.get('board_squares')}\n"
        f"- Inner corners: {c.get('inner_corners')}\n"
        f"- Square size: {c.get('square_size_m')} m\n"
        f"- Distance range: {c.get('distance_range_m')} m\n"
        f"- Images: {c.get('valid_images')} valid / {c.get('total_images')} total\n"
        f"- Mean reprojection error: {a.get('overall_reprojection_error_px'):.4f} px\n"
    )
    path.write_text(text, encoding="utf-8")
