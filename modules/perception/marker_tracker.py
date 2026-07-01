import logging
import queue
import threading
import time
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:  # Allows pure helper tests on machines without OpenCV.
    cv2 = None

from modules.comms.comm_base import CommunicationBase


def build_square_object_points(marker_size_m):
    half = float(marker_size_m) / 2.0
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )


def rotation_matrix_to_euler_deg(rotation):
    roll = np.degrees(np.arctan2(rotation[2, 1], rotation[2, 2]))
    pitch = np.degrees(
        np.arctan2(-rotation[2, 0], np.sqrt(rotation[2, 1] ** 2 + rotation[2, 2] ** 2))
    )
    yaw = np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return float(roll), float(pitch), float(yaw)


def make_pose_dict(marker_id, tvec, rvec, center, timestamp, euler_deg=None):
    tvec_flat = np.asarray(tvec, dtype=np.float64).reshape(3)
    if euler_deg is None:
        if cv2 is None:
            raise RuntimeError("OpenCV is required to convert rvec to Euler angles")
        rotation, _ = cv2.Rodrigues(rvec)
        euler_deg = rotation_matrix_to_euler_deg(rotation)

    roll, pitch, yaw = [float(v) for v in euler_deg]
    center_u, center_v = center
    return {
        "id": int(marker_id),
        "x": float(tvec_flat[0]),
        "y": float(tvec_flat[1]),
        "z": float(tvec_flat[2]),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "center_u": float(center_u),
        "center_v": float(center_v),
        "timestamp": float(timestamp),
        "detected": True,
    }


def compute_marker_pixel_size(corners):
    points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    edges = []
    for idx in range(4):
        next_idx = (idx + 1) % 4
        edges.append(np.linalg.norm(points[next_idx] - points[idx]))
    return float(np.mean(edges))


def normalize_detection_scale(value):
    try:
        scale = float(value)
    except (TypeError, ValueError):
        return 1.0
    if scale <= 0.0 or scale > 1.0:
        return 1.0
    return scale


def scale_detected_corners_to_original(corners, detection_scale):
    scale = normalize_detection_scale(detection_scale)
    if not corners:
        return corners
    return [np.asarray(marker_corners, dtype=np.float32) / scale for marker_corners in corners]


def compute_reprojection_error(object_points, image_points, rvec, tvec, camera_matrix, dist_coeffs):
    if cv2 is None:
        raise RuntimeError("OpenCV is required to compute reprojection error")

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
    image_points = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    return float(np.mean(np.linalg.norm(projected - image_points, axis=1)))


def _set_pose_quality(pose, valid, reason=""):
    pose["pose_valid"] = bool(valid)
    pose["reject_reason"] = "" if valid else reason
    return bool(valid)


def _optional_float(value):
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def validate_pose_quality(pose, config):
    if pose is None:
        return False

    max_abs_position_m = float(config.get("max_abs_position_m", 5.0))
    max_abs_yaw_deg = float(config.get("max_abs_yaw_deg", 180.0))
    min_marker_pixel_size_px = float(config.get("min_marker_pixel_size_px", 0.0))
    max_reprojection_error_px = float(config.get("max_reprojection_error_px", float("inf")))

    try:
        x = float(pose["x"])
        y = float(pose["y"])
        z = float(pose["z"])
        yaw = float(pose.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError):
        return _set_pose_quality(pose, False, "invalid_pose_fields")

    if abs(x) > max_abs_position_m or abs(y) > max_abs_position_m:
        return _set_pose_quality(pose, False, "position_out_of_range")
    if z <= 0.0 or z > max_abs_position_m:
        return _set_pose_quality(pose, False, "z_out_of_range")
    if abs(yaw) > max_abs_yaw_deg:
        return _set_pose_quality(pose, False, "yaw_out_of_range")

    marker_pixel_size = _optional_float(pose.get("marker_pixel_size_px"))
    if marker_pixel_size is not None and marker_pixel_size < min_marker_pixel_size_px:
        return _set_pose_quality(pose, False, "marker_too_small")

    reprojection_error = _optional_float(pose.get("reprojection_error_px"))
    if reprojection_error is not None and reprojection_error > max_reprojection_error_px:
        return _set_pose_quality(pose, False, "reprojection_error_too_high")

    return _set_pose_quality(pose, True)


TRACKER_COUNT_FIELDS = [
    "tracker_frames_processed",
    "tracker_marker_frames",
    "tracker_target_frames",
    "tracker_valid_pose_frames",
    "tracker_invalid_pose_frames",
    "tracker_no_marker_frames",
    "tracker_target_id_missing_frames",
    "tracker_pnp_failed_frames",
    "tracker_quality_rejected_frames",
    "tracker_capture_failed_frames",
]


def new_tracker_stats():
    stats = {field: 0 for field in TRACKER_COUNT_FIELDS}
    stats["tracker_last_frame_timestamp"] = ""
    stats["tracker_last_valid_pose_timestamp"] = ""
    return stats


def update_tracker_stats(stats, result, detected_ids=None, target_id=None, timestamp=None):
    detected_ids = [] if detected_ids is None else [int(v) for v in detected_ids]
    timestamp = "" if timestamp is None else float(timestamp)

    if result == "capture_failed":
        stats["tracker_capture_failed_frames"] += 1
        return stats

    stats["tracker_frames_processed"] += 1
    stats["tracker_last_frame_timestamp"] = timestamp

    if detected_ids:
        stats["tracker_marker_frames"] += 1
    if target_id is not None and int(target_id) in detected_ids:
        stats["tracker_target_frames"] += 1

    if result == "valid_pose":
        stats["tracker_valid_pose_frames"] += 1
        stats["tracker_last_valid_pose_timestamp"] = timestamp
    elif result in {"quality_rejected", "pnp_failed"}:
        stats["tracker_invalid_pose_frames"] += 1

    if result == "no_marker":
        stats["tracker_no_marker_frames"] += 1
    elif result == "target_id_not_found":
        stats["tracker_target_id_missing_frames"] += 1
    elif result == "pnp_failed":
        stats["tracker_pnp_failed_frames"] += 1
    elif result == "quality_rejected":
        stats["tracker_quality_rejected_frames"] += 1

    return stats


def _load_yaml(path):
    import yaml

    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _resolve_project_path(path):
    path = Path(path)
    if path.is_absolute():
        return path
    return Path(__file__).resolve().parents[2] / path


def apply_capture_settings(cap, config, cv2_module=None):
    cv2_module = cv2 if cv2_module is None else cv2_module
    width_prop = cv2_module.CAP_PROP_FRAME_WIDTH
    height_prop = cv2_module.CAP_PROP_FRAME_HEIGHT
    fourcc_prop = getattr(cv2_module, "CAP_PROP_FOURCC", None)
    fps_prop = getattr(cv2_module, "CAP_PROP_FPS", None)

    if config.get("fourcc") and fourcc_prop is not None:
        fourcc = str(config["fourcc"]).upper()
        if len(fourcc) == 4:
            cap.set(fourcc_prop, cv2_module.VideoWriter_fourcc(*fourcc))
    if config.get("frame_width"):
        cap.set(width_prop, int(config["frame_width"]))
    if config.get("frame_height"):
        cap.set(height_prop, int(config["frame_height"]))
    if config.get("fps") and fps_prop is not None:
        cap.set(fps_prop, float(config["fps"]))

    actual = {
        "frame_width": int(cap.get(width_prop)),
        "frame_height": int(cap.get(height_prop)),
    }
    if fourcc_prop is not None and config.get("fourcc"):
        fourcc_value = int(cap.get(fourcc_prop))
        actual["frame_fourcc"] = "".join(chr((fourcc_value >> 8 * idx) & 0xFF) for idx in range(4))
    if fps_prop is not None and config.get("fps"):
        actual["frame_fps"] = float(cap.get(fps_prop))

    return actual


def open_first_readable_capture(candidates, config, cv2_module=None, capture_factory=None):
    cv2_module = cv2 if cv2_module is None else cv2_module
    capture_factory = cv2_module.VideoCapture if capture_factory is None else capture_factory

    for dev in candidates:
        cap = capture_factory(dev)
        if not cap.isOpened():
            cap.release()
            continue

        actual_frame_size = apply_capture_settings(cap, config, cv2_module=cv2_module)
        ok, _ = cap.read()
        if ok:
            return cap, dev, actual_frame_size

        cap.release()

    return None, None, None


def frame_size_matches_config(config, actual_frame_size):
    requested_width = config.get("frame_width")
    requested_height = config.get("frame_height")
    if not requested_width and not requested_height:
        return True

    return (
        int(actual_frame_size.get("frame_width", 0)) == int(requested_width or actual_frame_size.get("frame_width", 0))
        and int(actual_frame_size.get("frame_height", 0)) == int(requested_height or actual_frame_size.get("frame_height", 0))
    )


def load_camera_parameters(config):
    if "camera_matrix" in config and "dist_coeffs" in config:
        return (
            np.array(config["camera_matrix"], dtype=np.float64).reshape(3, 3),
            np.array(config["dist_coeffs"], dtype=np.float64).reshape(-1, 1),
        )

    calibration_file = config.get("calibration_file")
    if not calibration_file:
        raise ValueError("vision tracking config requires camera_matrix/dist_coeffs or calibration_file")

    data = _load_yaml(_resolve_project_path(calibration_file))
    k_data = data.get("camera_matrix", {}).get("data")
    d_data = data.get("distortion_coefficients", {}).get("data")
    if not k_data or not d_data:
        raise ValueError(f"Calibration file has empty camera parameters: {calibration_file}")

    return (
        np.array(k_data, dtype=np.float64).reshape(3, 3),
        np.array(d_data, dtype=np.float64).reshape(-1, 1),
    )


class ArucoMarkerTracker(CommunicationBase):
    def __init__(self, config, surface=None):
        if cv2 is None:
            raise ImportError("opencv-contrib-python is required for ArucoMarkerTracker")
        if not hasattr(cv2, "aruco"):
            raise ImportError("cv2.aruco is required; install opencv-contrib-python")

        super().__init__(config.get("name", "ArucoMarkerTracker"))
        self.logger = logging.getLogger(__name__)
        self.surface = surface

        self.device_path = config.get("device")
        self.camera_id = config.get("camera_id", 0)
        self.marker_id = int(config.get("marker_id", 20))
        self.marker_size_m = float(config.get("marker_size_m", 0.04))
        self.min_detections = int(config.get("min_detections", 3))
        self.max_lost = int(config.get("max_lost", 10))
        self.enable_undistort_preview = bool(config.get("enable_undistort_preview", False))
        self.enable_preview_annotations = bool(config.get("enable_preview_annotations", True))
        self.detection_scale = normalize_detection_scale(config.get("detection_scale", 1.0))
        self.config = config
        self.actual_frame_size = None
        self.selected_device = None
        self.latest_diagnostics = {
            "device": "",
            "frame_width": "",
            "frame_height": "",
            "detected_ids": [],
            "rejected_count": 0,
            "pose_valid": False,
            "reject_reason": "not_started",
        }
        self._diagnostics_lock = threading.Lock()
        self.tracker_stats = new_tracker_stats()
        self._stats_lock = threading.Lock()
        self._latest_annotated_frame = None
        self._frame_lock = threading.Lock()

        self.camera_matrix, self.dist_coeffs = load_camera_parameters(config)
        self.object_points = build_square_object_points(self.marker_size_m)

        dictionary_name = config.get("dictionary", "DICT_4X4_50")
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        parameters = cv2.aruco.DetectorParameters()
        if config.get("corner_refinement", True):
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        self.cap = None
        self.running = False
        self.pose_queue = queue.Queue(maxsize=1)
        self.continuous_detections = 0
        self.continuous_lost = 0
        self.thread = None

    def start(self):
        candidates = []
        if self.device_path:
            candidates.append(self.device_path)
        if self.camera_id is not None:
            candidates.append(self.camera_id)
        if not candidates:
            candidates = [f"/dev/video{i}" for i in range(10)]

        self.cap, self.selected_device, self.actual_frame_size = open_first_readable_capture(candidates, self.config)
        if self.cap is None:
            self.logger.error(f"Unable to open a readable ArUco camera. Tried: {candidates}")
            self._update_diagnostics(reject_reason="camera_open_failed")
            return

        self.logger.info(f"Opened ArUco camera: {self.selected_device}")
        self.logger.info(f"ArUco camera actual resolution: {self.actual_frame_size}")
        if not frame_size_matches_config(self.config, self.actual_frame_size):
            self.logger.warning(
                "ArUco camera actual resolution differs from config; PnP distance may be biased: "
                f"requested=({self.config.get('frame_width')}, {self.config.get('frame_height')}), "
                f"actual={self.actual_frame_size}"
            )
        self._update_diagnostics(reject_reason="no_pose")
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        self.logger.info("ArucoMarkerTracker started")

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.logger.info("ArucoMarkerTracker stopped")

    def _run_loop(self):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                self.logger.warning("ArUco image capture failed")
                self._record_frame_result("capture_failed", [], time.time())
                self._update_diagnostics(reject_reason="capture_failed")
                time.sleep(0.02)
                continue

            pose = self.process_frame(frame, timestamp=time.time())
            if pose is not None:
                self._put_pose(pose)
                if pose.get("pose_valid", True):
                    self.continuous_detections += 1
                    self.continuous_lost = 0
                else:
                    self.continuous_detections = 0
                    self.continuous_lost += 1
            else:
                self.continuous_detections = 0
                self.continuous_lost += 1

            if self.surface:
                try:
                    preview = self._surface_preview_frame(frame)
                    success, jpg = cv2.imencode(".jpg", preview, encode_param)
                    if success:
                        self.surface.send_video_packet(jpg.tobytes(), 0x01, 0x00)
                except Exception as exc:
                    self.logger.warning(f"ArUco video forwarding failed: {exc}")

            time.sleep(0.02)

    def _put_pose(self, pose):
        try:
            self.pose_queue.put(pose, block=False)
        except queue.Full:
            try:
                self.pose_queue.get_nowait()
            except queue.Empty:
                pass
            self.pose_queue.put(pose, block=False)

    def _set_annotated_frame(self, frame):
        if frame is None:
            return
        with self._frame_lock:
            self._latest_annotated_frame = frame.copy()

    def get_annotated_frame(self):
        with self._frame_lock:
            if self._latest_annotated_frame is None:
                return None
            return self._latest_annotated_frame.copy()

    def _surface_preview_frame(self, frame):
        annotated = self.get_annotated_frame()
        if annotated is not None:
            return annotated
        if getattr(self, "enable_undistort_preview", False):
            return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        return frame.copy()

    def _draw_preview_text(self, frame, lines):
        for idx, line in enumerate(lines):
            y = 28 + idx * 24
            cv2.putText(
                frame,
                str(line),
                (16, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def _draw_detected_markers(self, frame, corners, ids):
        if ids is None:
            return
        if hasattr(cv2.aruco, "drawDetectedMarkers"):
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            return
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            points = np.asarray(marker_corners, dtype=np.int32).reshape(4, 2)
            cv2.polylines(frame, [points], True, (0, 255, 0), 2)
            cv2.putText(
                frame,
                str(int(marker_id)),
                tuple(points[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

    def _update_diagnostics(self, **updates):
        with self._diagnostics_lock:
            diagnostics = dict(self.latest_diagnostics)
            diagnostics["device"] = self.selected_device or self.device_path or self.camera_id or ""
            if self.actual_frame_size:
                diagnostics.update(self.actual_frame_size)
            diagnostics.update(updates)
            self.latest_diagnostics = diagnostics

    def _record_frame_result(self, result, detected_ids=None, timestamp=None):
        with self._stats_lock:
            update_tracker_stats(
                self.tracker_stats,
                result,
                detected_ids=detected_ids,
                target_id=self.marker_id,
                timestamp=timestamp,
            )

    def get_diagnostics(self):
        with self._diagnostics_lock:
            diagnostics = dict(self.latest_diagnostics)
        with self._stats_lock:
            diagnostics.update(dict(self.tracker_stats))
        if isinstance(diagnostics.get("detected_ids"), list):
            diagnostics["detected_ids"] = list(diagnostics["detected_ids"])
        return diagnostics

    def process_frame(self, frame, timestamp=None):
        timestamp = time.time() if timestamp is None else timestamp
        annotated = frame.copy() if self.enable_preview_annotations else None
        if self.detection_scale < 1.0:
            detection_frame = cv2.resize(
                frame,
                None,
                fx=self.detection_scale,
                fy=self.detection_scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            detection_frame = frame
        gray = cv2.cvtColor(detection_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        corners = scale_detected_corners_to_original(corners, self.detection_scale)
        detected_ids = [] if ids is None else [int(v) for v in ids.flatten()]
        rejected_count = 0 if rejected is None else len(rejected)
        if annotated is not None:
            self._draw_detected_markers(annotated, corners, ids)
        self._update_diagnostics(
            detected_ids=detected_ids,
            rejected_count=rejected_count,
            pose_valid=False,
            reject_reason="no_marker",
            detection_scale=self.detection_scale,
        )
        if ids is None:
            self._record_frame_result("no_marker", detected_ids, timestamp)
            if annotated is not None:
                self._draw_preview_text(annotated, [f"target id {self.marker_id}: no marker"])
                self._set_annotated_frame(annotated)
            return None

        for marker_corners, found_id in zip(corners, ids.flatten()):
            if int(found_id) != self.marker_id:
                continue

            image_points = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
            ok, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                self._record_frame_result("pnp_failed", detected_ids, timestamp)
                self._update_diagnostics(reject_reason="pnp_failed")
                if annotated is not None:
                    self._draw_preview_text(annotated, [f"target id {self.marker_id}: pnp_failed"])
                    self._set_annotated_frame(annotated)
                return None

            rotation, _ = cv2.Rodrigues(rvec)
            center = image_points.mean(axis=0)
            marker_pixel_size = compute_marker_pixel_size(image_points)
            reprojection_error = compute_reprojection_error(
                self.object_points,
                image_points,
                rvec,
                tvec,
                self.camera_matrix,
                self.dist_coeffs,
            )
            pose = make_pose_dict(
                marker_id=int(found_id),
                tvec=tvec,
                rvec=rvec,
                center=center,
                timestamp=timestamp,
                euler_deg=rotation_matrix_to_euler_deg(rotation),
            )
            pose["marker_pixel_size_px"] = marker_pixel_size
            pose["reprojection_error_px"] = reprojection_error
            validate_pose_quality(pose, self.config)
            result = "valid_pose" if pose["pose_valid"] else "quality_rejected"
            if annotated is not None:
                cv2.circle(annotated, (int(round(center[0])), int(round(center[1]))), 5, (0, 0, 255), -1)
                if hasattr(cv2, "drawFrameAxes"):
                    try:
                        cv2.drawFrameAxes(
                            annotated,
                            self.camera_matrix,
                            self.dist_coeffs,
                            rvec,
                            tvec,
                            self.marker_size_m * 0.5,
                        )
                    except cv2.error as exc:
                        self.logger.debug(f"drawFrameAxes failed: {exc}")
                self._draw_preview_text(
                    annotated,
                    [
                        f"id={int(found_id)} z={pose['z']:.3f}m yaw={pose['yaw']:.2f}deg",
                        f"valid={pose['pose_valid']} reject={pose['reject_reason'] or '-'} reproj={reprojection_error:.2f}px",
                    ],
                )
                self._set_annotated_frame(annotated)
            self._record_frame_result(result, detected_ids, timestamp)
            self._update_diagnostics(
                marker_pixel_size_px=marker_pixel_size,
                reprojection_error_px=reprojection_error,
                pose_valid=pose["pose_valid"],
                reject_reason=pose["reject_reason"],
            )
            return pose

        self._record_frame_result("target_id_not_found", detected_ids, timestamp)
        self._update_diagnostics(reject_reason="target_id_not_found")
        if annotated is not None:
            self._draw_preview_text(
                annotated,
                [f"target id {self.marker_id}: not found", f"detected ids: {detected_ids}"],
            )
            self._set_annotated_frame(annotated)
        return None

    def get_pose(self):
        try:
            return self.pose_queue.get_nowait()
        except queue.Empty:
            return None

    def target_detected(self):
        return self.continuous_detections >= self.min_detections

    def target_lost(self):
        return self.continuous_lost >= self.max_lost
