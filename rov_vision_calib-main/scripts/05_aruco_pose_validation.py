import argparse
import csv
import math
import time
from datetime import date
from pathlib import Path

import cv2
import numpy as np

import _bootstrap  # noqa: F401
from camera.camera_params import load_calibration
from camera.uvc_camera import open_camera
from perception.aruco_detector import ArucoDetector
from perception.pose_estimator import estimate_single_marker_pose, yaw_from_rvec_deg
from utils.yaml_io import load_yaml


STATIC_HEADER = [
    "date",
    "calib_file",
    "resolution",
    "marker_size_m",
    "true_z_m",
    "frame_count",
    "detect_count",
    "detect_rate",
    "x_mean_m",
    "x_std_m",
    "y_mean_m",
    "y_std_m",
    "z_mean_m",
    "z_std_m",
    "z_error_m",
    "yaw_mean_deg",
    "yaw_std_deg",
]

DIRECTION_HEADER = [
    "date",
    "position",
    "image_u",
    "image_v",
    "ex_px",
    "ey_px",
    "tvec_x_m",
    "tvec_y_m",
    "tvec_z_m",
    "control_x_sign",
    "control_y_sign",
    "result",
]

REPEATABILITY_HEADER = [
    "date",
    "calib_file",
    "resolution",
    "marker_size_m",
    "frame_count",
    "detect_count",
    "detect_rate",
    "x_mean_m",
    "x_std_m",
    "y_mean_m",
    "y_std_m",
    "z_mean_m",
    "z_std_m",
    "yaw_mean_deg",
    "yaw_std_deg",
]

LIVE_HEADER = ["time_s", "marker_id", "u_px", "v_px", "x_m", "y_m", "z_m", "yaw_deg"]


def open_video_writer(path: str | Path, frame_size: tuple[int, int], fps: float, fourcc: str) -> cv2.VideoWriter:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*fourcc), fps, frame_size)
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open video writer: {path}")
    return writer


def append_csv_row(path: str | Path, header: list[str], row: list) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(header)
        writer.writerow(row)


def mean_std(values: list[float]) -> tuple[float, float]:
    if not values:
        return math.nan, math.nan
    arr = np.asarray(values, dtype=np.float64)
    return float(np.mean(arr)), float(np.std(arr))


def detect_selected_marker(corners, ids, target_marker_id: int | None, allow_any_marker: bool):
    if ids is None or len(ids) == 0:
        return None, None
    flat_ids = ids.flatten()
    for marker_corners, found_id in zip(corners, flat_ids):
        found_id = int(found_id)
        if allow_any_marker or target_marker_id is None or found_id == target_marker_id:
            return marker_corners, found_id
    return None, None


def pose_record(marker_corners, marker_id, marker_size_m, camera_matrix, dist_coeffs) -> dict:
    rvec, tvec = estimate_single_marker_pose(marker_corners, marker_size_m, camera_matrix, dist_coeffs)
    center = np.asarray(marker_corners).reshape(4, 2).mean(axis=0)
    x, y, z = [float(v) for v in tvec.reshape(3)]
    return {
        "marker_id": int(marker_id),
        "u_px": float(center[0]),
        "v_px": float(center[1]),
        "x_m": x,
        "y_m": y,
        "z_m": z,
        "yaw_deg": yaw_from_rvec_deg(rvec),
        "rvec": rvec,
        "tvec": tvec,
    }


def maybe_show_or_save(
    frame,
    corners,
    ids,
    record,
    camera_matrix,
    dist_coeffs,
    axis_len_m: float,
    no_preview: bool,
    debug_dir: str | None,
    frame_index: int,
    save_debug_every: int,
    video_writer,
) -> bool:
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    if record is not None:
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, record["rvec"], record["tvec"], axis_len_m)

    if video_writer is not None:
        video_writer.write(frame)

    if debug_dir and save_debug_every > 0 and frame_index % save_debug_every == 0:
        out_dir = Path(debug_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out_dir / f"frame_{frame_index:06d}.png"), frame)

    if no_preview:
        return False
    cv2.imshow("aruco_pose_validation", frame)
    return (cv2.waitKey(1) & 0xFF) in (ord("q"), 27)


def collect_records(cap, detector, cfg, args, camera_matrix, dist_coeffs):
    marker_size_m = float(cfg["aruco"]["marker_size_m"])
    marker_id_cfg = cfg["aruco"].get("marker_id")
    target_marker_id = args.marker_id if args.marker_id is not None else marker_id_cfg
    target_marker_id = None if target_marker_id in (None, "any") else int(target_marker_id)
    allow_any_marker = args.allow_any_marker or target_marker_id is None
    axis_len_m = float(cfg["detection"].get("draw_axes_length_m", 0.05))

    records = []
    attempted = 0
    resolution = None
    video_writer = None
    try:
        while attempted < args.frame_count:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Failed to read frame from camera")
            attempted += 1
            h, w = frame.shape[:2]
            resolution = (w, h)
            if args.save_video and video_writer is None:
                video_writer = open_video_writer(args.save_video, resolution, args.video_fps, args.video_fourcc)

            corners, ids, _ = detector.detect(frame)
            marker_corners, found_id = detect_selected_marker(corners, ids, target_marker_id, allow_any_marker)
            record = None
            if marker_corners is not None:
                record = pose_record(marker_corners, found_id, marker_size_m, camera_matrix, dist_coeffs)
                records.append(record)

            should_quit = maybe_show_or_save(
                frame,
                corners,
                ids,
                record,
                camera_matrix,
                dist_coeffs,
                axis_len_m,
                args.no_preview,
                args.debug_dir,
                attempted,
                args.save_debug_every,
                video_writer,
            )
            if should_quit:
                break
    finally:
        if video_writer is not None:
            video_writer.release()

    return attempted, records, resolution


def detect_rate(detect_count: int, frame_count: int) -> float:
    if frame_count <= 0:
        return math.nan
    return float(detect_count / frame_count)


def values(records: list[dict], key: str) -> list[float]:
    return [float(r[key]) for r in records]


def sign_from_error(error_px: float, tolerance_px: float) -> str:
    if math.isnan(error_px) or abs(error_px) <= tolerance_px:
        return "zero"
    return "positive" if error_px < 0 else "negative"


def expected_direction_signs(position: str) -> tuple[str, str]:
    expected = {
        "center": ("zero", "zero"),
        "left": ("positive", "zero"),
        "right": ("negative", "zero"),
        "up": ("zero", "positive"),
        "down": ("zero", "negative"),
    }
    return expected[position]


def run_static(args, cfg, camera_matrix, dist_coeffs):
    if args.true_z_m is None:
        raise ValueError("--true-z-m is required for static validation")
    cap = open_camera(args.camera)
    detector = ArucoDetector(cfg["aruco"]["dictionary"], cfg["detection"].get("corner_refinement", True))
    frame_count, records, resolution = collect_records(cap, detector, cfg, args, camera_matrix, dist_coeffs)
    cap.release()
    if not args.no_preview:
        cv2.destroyAllWindows()

    x_mean, x_std = mean_std(values(records, "x_m"))
    y_mean, y_std = mean_std(values(records, "y_m"))
    z_mean, z_std = mean_std(values(records, "z_m"))
    yaw_mean, yaw_std = mean_std(values(records, "yaw_deg"))
    z_error = z_mean - float(args.true_z_m) if not math.isnan(z_mean) else math.nan
    resolution_text = f"{resolution[0]}x{resolution[1]}" if resolution else ""
    row = [
        args.date,
        Path(cfg["camera_calibration"]["file"]).name,
        resolution_text,
        float(cfg["aruco"]["marker_size_m"]),
        float(args.true_z_m),
        frame_count,
        len(records),
        detect_rate(len(records), frame_count),
        x_mean,
        x_std,
        y_mean,
        y_std,
        z_mean,
        z_std,
        z_error,
        yaw_mean,
        yaw_std,
    ]
    append_csv_row(args.output, STATIC_HEADER, row)


def run_direction(args, cfg, camera_matrix, dist_coeffs):
    if args.position is None:
        raise ValueError("--position is required for direction validation")
    cap = open_camera(args.camera)
    detector = ArucoDetector(cfg["aruco"]["dictionary"], cfg["detection"].get("corner_refinement", True))
    frame_count, records, resolution = collect_records(cap, detector, cfg, args, camera_matrix, dist_coeffs)
    cap.release()
    if not args.no_preview:
        cv2.destroyAllWindows()

    u_mean, _ = mean_std(values(records, "u_px"))
    v_mean, _ = mean_std(values(records, "v_px"))
    x_mean, _ = mean_std(values(records, "x_m"))
    y_mean, _ = mean_std(values(records, "y_m"))
    z_mean, _ = mean_std(values(records, "z_m"))
    if resolution:
        ex_px = u_mean - resolution[0] / 2.0
        ey_px = v_mean - resolution[1] / 2.0
    else:
        ex_px = math.nan
        ey_px = math.nan

    tolerance = float(args.pixel_tolerance)
    control_x_sign = sign_from_error(ex_px, tolerance)
    control_y_sign = sign_from_error(ey_px, tolerance)
    expected_x, expected_y = expected_direction_signs(args.position)
    result = "pass" if records and control_x_sign == expected_x and control_y_sign == expected_y else "fail"
    row = [
        args.date,
        args.position,
        u_mean,
        v_mean,
        ex_px,
        ey_px,
        x_mean,
        y_mean,
        z_mean,
        control_x_sign,
        control_y_sign,
        result,
    ]
    append_csv_row(args.output, DIRECTION_HEADER, row)


def run_repeatability(args, cfg, camera_matrix, dist_coeffs):
    cap = open_camera(args.camera)
    detector = ArucoDetector(cfg["aruco"]["dictionary"], cfg["detection"].get("corner_refinement", True))
    frame_count, records, resolution = collect_records(cap, detector, cfg, args, camera_matrix, dist_coeffs)
    cap.release()
    if not args.no_preview:
        cv2.destroyAllWindows()

    x_mean, x_std = mean_std(values(records, "x_m"))
    y_mean, y_std = mean_std(values(records, "y_m"))
    z_mean, z_std = mean_std(values(records, "z_m"))
    yaw_mean, yaw_std = mean_std(values(records, "yaw_deg"))
    resolution_text = f"{resolution[0]}x{resolution[1]}" if resolution else ""
    row = [
        args.date,
        Path(cfg["camera_calibration"]["file"]).name,
        resolution_text,
        float(cfg["aruco"]["marker_size_m"]),
        frame_count,
        len(records),
        detect_rate(len(records), frame_count),
        x_mean,
        x_std,
        y_mean,
        y_std,
        z_mean,
        z_std,
        yaw_mean,
        yaw_std,
    ]
    append_csv_row(args.output, REPEATABILITY_HEADER, row)


def run_live(args, cfg, camera_matrix, dist_coeffs):
    cap = open_camera(args.camera)
    detector = ArucoDetector(cfg["aruco"]["dictionary"], cfg["detection"].get("corner_refinement", True))
    marker_size_m = float(cfg["aruco"]["marker_size_m"])
    marker_id = cfg["aruco"].get("marker_id")
    marker_id = args.marker_id if args.marker_id is not None else marker_id
    marker_id = None if marker_id in (None, "any") else int(marker_id)
    allow_any_marker = args.allow_any_marker or marker_id is None
    axis_len_m = float(cfg["detection"].get("draw_axes_length_m", 0.05))

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    video_writer = None
    with open(args.output, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(LIVE_HEADER)
        print("Press q to stop.")
        frame_index = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Failed to read frame from camera")
            frame_index += 1
            h, w = frame.shape[:2]
            if args.save_video and video_writer is None:
                video_writer = open_video_writer(args.save_video, (w, h), args.video_fps, args.video_fourcc)
            corners, ids, _ = detector.detect(frame)
            marker_corners, found_id = detect_selected_marker(corners, ids, marker_id, allow_any_marker)
            record = None
            if marker_corners is not None:
                record = pose_record(marker_corners, found_id, marker_size_m, camera_matrix, dist_coeffs)
                writer.writerow(
                    [
                        time.time(),
                        record["marker_id"],
                        record["u_px"],
                        record["v_px"],
                        record["x_m"],
                        record["y_m"],
                        record["z_m"],
                        record["yaw_deg"],
                    ]
                )
            should_quit = maybe_show_or_save(
                frame,
                corners,
                ids,
                record,
                camera_matrix,
                dist_coeffs,
                axis_len_m,
                args.no_preview,
                args.debug_dir,
                frame_index,
                args.save_debug_every,
                video_writer,
            )
            if should_quit:
                break

    cap.release()
    if video_writer is not None:
        video_writer.release()
    if not args.no_preview:
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Run ArUco/AprilTag pose validation and write CSV logs.")
    parser.add_argument("--mode", choices=["live", "static", "direction", "repeatability"], default="live")
    parser.add_argument("--camera", default="config/camera_rov_front.yaml")
    parser.add_argument("--aruco", default="config/aruco_config.yaml")
    parser.add_argument("--output", required=True)
    parser.add_argument("--frame-count", type=int, default=None)
    parser.add_argument("--true-z-m", type=float, default=None)
    parser.add_argument("--position", choices=["center", "left", "right", "up", "down"], default=None)
    parser.add_argument("--pixel-tolerance", type=float, default=None)
    parser.add_argument("--marker-id", type=int, default=None)
    parser.add_argument("--allow-any-marker", action="store_true")
    parser.add_argument("--date", default=date.today().isoformat())
    parser.add_argument("--debug-dir", default=None)
    parser.add_argument("--save-debug-every", type=int, default=0)
    parser.add_argument("--save-video", default=None)
    parser.add_argument("--video-fps", type=float, default=30.0)
    parser.add_argument("--video-fourcc", default="MJPG")
    parser.add_argument("--no-preview", action="store_true")
    args = parser.parse_args()

    cfg = load_yaml(args.aruco)
    default_frame_count = int(cfg.get("validation", {}).get("default_frame_count", 300))
    args.frame_count = args.frame_count or default_frame_count
    args.pixel_tolerance = args.pixel_tolerance or float(cfg.get("validation", {}).get("direction_pixel_tolerance", 40))

    camera_matrix, dist_coeffs, _ = load_calibration(cfg["camera_calibration"]["file"])
    if args.mode == "static":
        run_static(args, cfg, camera_matrix, dist_coeffs)
    elif args.mode == "direction":
        run_direction(args, cfg, camera_matrix, dist_coeffs)
    elif args.mode == "repeatability":
        run_repeatability(args, cfg, camera_matrix, dist_coeffs)
    else:
        run_live(args, cfg, camera_matrix, dist_coeffs)


if __name__ == "__main__":
    main()
