import argparse
import csv
import time

import numpy as np

import _bootstrap  # noqa: F401
from camera.camera_params import load_calibration
from camera.uvc_camera import open_camera
from control.visual_servo_p import compute_errors, p_control
from perception.aruco_detector import ArucoDetector
from perception.pose_estimator import estimate_single_marker_pose, yaw_from_rvec_deg
from utils.yaml_io import load_yaml


def main():
    parser = argparse.ArgumentParser(description="Compute visual-servo commands without sending them to the ROV.")
    parser.add_argument("--camera", default="config/camera_rov_front.yaml")
    parser.add_argument("--aruco", default="config/aruco_config.yaml")
    parser.add_argument("--servo", default="config/servo_config.yaml")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    aruco_cfg = load_yaml(args.aruco)
    servo_cfg = load_yaml(args.servo)
    camera_matrix, dist_coeffs, _ = load_calibration(aruco_cfg["camera_calibration"]["file"])
    detector = ArucoDetector(aruco_cfg["aruco"]["dictionary"], aruco_cfg["detection"].get("corner_refinement", True))
    marker_id = int(aruco_cfg["aruco"]["marker_id"])
    marker_size_m = float(aruco_cfg["aruco"]["marker_size_m"])
    cap = open_camera(args.camera)

    with open(args.output, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "u_px", "v_px", "z_m", "yaw_deg", "err_u_px", "err_v_px", "err_z_m", "err_yaw_deg", "rc_lateral", "rc_vertical", "rc_forward", "rc_yaw"])
        print("Dry run started. Press Ctrl+C to stop.")
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Failed to read frame from camera")
            corners, ids, _ = detector.detect(frame)
            if ids is None:
                continue
            for marker_corners, found_id in zip(corners, ids.flatten()):
                if int(found_id) != marker_id:
                    continue
                rvec, tvec = estimate_single_marker_pose(marker_corners, marker_size_m, camera_matrix, dist_coeffs)
                center = np.asarray(marker_corners).reshape(4, 2).mean(axis=0)
                yaw = yaw_from_rvec_deg(rvec)
                z = float(tvec.reshape(3)[2])
                errors = compute_errors(center[0], center[1], z, yaw, servo_cfg)
                command = p_control(errors, servo_cfg, safe=False)
                writer.writerow([time.time(), center[0], center[1], z, yaw, errors["u_px"], errors["v_px"], errors["z_m"], errors["yaw_deg"], command["lateral"], command["vertical"], command["forward"], command["yaw"]])
                f.flush()


if __name__ == "__main__":
    main()
