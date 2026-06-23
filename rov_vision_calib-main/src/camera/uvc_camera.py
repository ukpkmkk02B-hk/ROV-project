from __future__ import annotations

from pathlib import Path

import cv2

from utils.yaml_io import load_yaml


def open_camera(config_path: str | Path) -> cv2.VideoCapture:
    cfg = load_yaml(config_path)
    cam = cfg["camera"]
    device = cam.get("device", 0)
    backend_name = cam.get("backend", "").lower()
    backend = cv2.CAP_V4L2 if backend_name == "v4l2" else cv2.CAP_ANY

    cap = cv2.VideoCapture(device, backend)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera device: {device}")

    resolution = cam.get("resolution", {})
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(resolution.get("width", 1920)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(resolution.get("height", 1080)))
    cap.set(cv2.CAP_PROP_FPS, int(cam.get("fps", 30)))

    fourcc = cam.get("fourcc")
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))

    settings = cfg.get("settings", {})
    if settings.get("autofocus") is False:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    if settings.get("exposure_mode") == "manual":
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    if settings.get("exposure_value") is not None:
        cap.set(cv2.CAP_PROP_EXPOSURE, float(settings["exposure_value"]))
    if settings.get("gain") is not None:
        cap.set(cv2.CAP_PROP_GAIN, float(settings["gain"]))

    return cap
