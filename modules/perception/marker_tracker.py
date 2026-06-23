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

    if config.get("frame_width"):
        cap.set(width_prop, int(config["frame_width"]))
    if config.get("frame_height"):
        cap.set(height_prop, int(config["frame_height"]))

    return {
        "frame_width": int(cap.get(width_prop)),
        "frame_height": int(cap.get(height_prop)),
    }


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
        self.config = config
        self.actual_frame_size = None

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
        self.thread = threading.Thread(target=self._run_loop, daemon=True)

    def start(self):
        candidates = []
        if self.device_path:
            candidates.append(self.device_path)
        if self.camera_id is not None:
            candidates.append(self.camera_id)
        if not candidates:
            candidates = [f"/dev/video{i}" for i in range(10)]

        for dev in candidates:
            cap = cv2.VideoCapture(dev)
            if cap.isOpened():
                self.cap = cap
                self.actual_frame_size = apply_capture_settings(self.cap, self.config)
                self.logger.info(f"成功打开ArUco摄像头 {dev}")
                self.logger.info(f"ArUco摄像头实际分辨率: {self.actual_frame_size}")
                if not frame_size_matches_config(self.config, self.actual_frame_size):
                    self.logger.warning(
                        "ArUco摄像头实际分辨率与配置不一致，PnP距离可能偏差: "
                        f"requested=({self.config.get('frame_width')}, {self.config.get('frame_height')}), "
                        f"actual={self.actual_frame_size}"
                    )
                break
            cap.release()

        if self.cap is None:
            self.logger.error(f"无法打开任何ArUco摄像头设备，尝试了: {candidates}")
            return

        self.running = True
        self.thread.start()
        self.logger.info("ArucoMarkerTracker 已启动")

    def stop(self):
        self.running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.logger.info("ArucoMarkerTracker 已停止")

    def _run_loop(self):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                self.logger.warning("ArUco图像采集失败")
                time.sleep(0.02)
                continue

            pose = self.process_frame(frame, timestamp=time.time())
            if pose is not None:
                self._put_pose(pose)
                self.continuous_detections += 1
                self.continuous_lost = 0
            else:
                self.continuous_detections = 0
                self.continuous_lost += 1

            if self.surface:
                try:
                    preview = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs) if self.enable_undistort_preview else frame
                    success, jpg = cv2.imencode(".jpg", preview, encode_param)
                    if success:
                        self.surface.send_video_packet(jpg.tobytes(), 0x01, 0x00)
                except Exception as exc:
                    self.logger.warning(f"ArUco视频转发失败: {exc}")

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

    def process_frame(self, frame, timestamp=None):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
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
                return None
            rotation, _ = cv2.Rodrigues(rvec)
            center = image_points.mean(axis=0)
            return make_pose_dict(
                marker_id=int(found_id),
                tvec=tvec,
                rvec=rvec,
                center=center,
                timestamp=time.time() if timestamp is None else timestamp,
                euler_deg=rotation_matrix_to_euler_deg(rotation),
            )
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
