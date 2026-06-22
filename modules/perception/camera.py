import cv2
import numpy as np
import queue
import threading
import time
import logging
import pupil_apriltags as apriltag
import os
import sys

from modules.comms.comm_base import CommunicationBase


class AprilTagCameraInterface(CommunicationBase):
    def __init__(self, config, surface=None):
        # 加载配置
        config_data = self._process_config(config)
        super().__init__(config_data.get("name", "AprilTagCamera"))

        # 参数初始化
        self.device_path = config_data.get("device")
        self.camera_id = config_data.get("camera_id", 0)
        self.marker_length = config_data.get("marker_length", 0.05)
        self.camera_matrix = np.array(config_data["camera_matrix"])
        self.dist_coeffs = np.array(config_data["dist_coeffs"]).reshape((5, 1))
        self.tag_family = config_data.get("tag_family", "tag36h11")

        self.min_detections = config_data.get("min_detections", 5)
        self.max_lost = config_data.get("max_lost", 3)

        self.surface = surface
        self.logger = logging.getLogger(__name__)
        self.detector = apriltag.Detector(families=self.tag_family)

        # 状态
        self.cap = None
        self.running = False
        self.input_queue = queue.Queue(maxsize=1)
        self.continuous_detections = 0
        self.continuous_lost = 0
        self._target_detected_value = False

        self.thread = threading.Thread(target=self._run_loop, daemon=True)

    def _process_config(self, config):
        return config if isinstance(config, dict) else {}

    def start(self):
        candidates = []
        if self.device_path:
            candidates.append(self.device_path)
        if self.camera_id is not None:
            candidates.append(self.camera_id)  # 假设是数字id，如0、1等
        # 如果 candidates 为空，尝试默认的 /dev/video0、/dev/video1、... /dev/video9
        if not candidates:
            candidates = [f'/dev/video{i}' for i in range(10)]

        self.cap = None
        for dev in candidates:
            cap = cv2.VideoCapture(dev)
            if cap.isOpened():
                self.cap = cap
                self.logger.info(f"成功打开摄像头 {dev}")
                break
            else:
                cap.release()

        if self.cap is None:
            self.logger.error(f"无法打开任何摄像头设备，尝试了: {candidates}")
            return

        self.running = True
        self.thread.start()
        self.logger.info("AprilTagCameraInterface 已启动")


    def stop(self):
        self.running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.logger.info("AprilTagCameraInterface 已停止")

    def _run_loop(self):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                self.logger.warning("图像采集失败")
                time.sleep(0.02)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_undistorted = cv2.undistort(gray, self.camera_matrix, self.dist_coeffs)
            detections = self.detector.detect(gray_undistorted)

            pose = None
            if detections:
                detection = detections[0]
                tag_id = detection.tag_id
                corners = detection.corners.astype(np.float32)

                obj_points = np.array([
                    [-self.marker_length / 2, -self.marker_length / 2, 0],
                    [self.marker_length / 2, -self.marker_length / 2, 0],
                    [self.marker_length / 2, self.marker_length / 2, 0],
                    [-self.marker_length / 2, self.marker_length / 2, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(obj_points, corners, self.camera_matrix, self.dist_coeffs)

                if success:
                    x, y, z = tvec.flatten()
                    R, _ = cv2.Rodrigues(rvec)
                    rx = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
                    ry = np.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2)))
                    rz = np.degrees(np.arctan2(R[1, 0], R[0, 0]))

                    pose = {
                        "id": int(tag_id),
                        "x": float(x),
                        "y": float(y),
                        "z": float(z),
                        "roll": float(rz),
                        "pitch": float(rx),
                        "yaw": float(ry)
                    }

                    try:
                        self.input_queue.put(pose, block=False)
                    except queue.Full:
                        pass

                    self.continuous_detections += 1
                    self.continuous_lost = 0
                    self._target_detected_value = True
                else:
                    self.continuous_detections = 0
                    self.continuous_lost += 1
                    if self.continuous_lost >= self.max_lost:
                        self._target_detected_value = False
            else:
                self.continuous_detections = 0
                self.continuous_lost += 1
                if self.continuous_lost >= self.max_lost:
                    self._target_detected_value = False

            # 视频转发
            if self.surface:
                try:
                    success, jpg = cv2.imencode('.jpg', frame, encode_param)
                    if success:
                        self.surface.send_video_packet(jpg.tobytes(),0x01, 0x00)  # 视频帧类型为0x01，子类型为0x00
                except Exception as e:
                    self.logger.warning(f"视频转发失败: {e}")

            time.sleep(0.02)

    def get_pose(self):
        try:
            return self.input_queue.get_nowait()
        except queue.Empty:
            return None

    def target_detected(self):
        return self.continuous_detections >= self.min_detections

    def target_lost(self):
        return self.continuous_lost >= self.max_lost


if __name__ == "__main__":
    config = {
        "name": "UVC_Camera_Main",
        "device": 0,
        "tag_family": "tag36h11",
        "marker_length": 0.092,
        "camera_matrix": [
            [342.10142903, 0, 310.67383732],
            [0, 342.30795475, 251.8921935],
            [0, 0, 1]
        ],
        "dist_coeffs": [0.05423003, -0.05146786, -0.00032794, -0.00140946, -0.00136574],
        "min_detections": 5,
        "max_lost": 3
    }

    camera_interface = AprilTagCameraInterface(config)
    camera_interface.start()

    try:
        while True:
            pose_data = camera_interface.get_pose()
            if pose_data:
                print(f"[POSE] ✅ {pose_data}")
            else:
                print(f"[POSE] ❌ 未检测到目标")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[EXIT] 手动终止测试...")
        camera_interface.stop()
