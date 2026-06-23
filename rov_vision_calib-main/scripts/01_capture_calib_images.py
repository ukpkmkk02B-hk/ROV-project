import argparse
from pathlib import Path

import cv2

import _bootstrap  # noqa: F401
from camera.uvc_camera import open_camera
from utils.time_sync import timestamp_for_path


def main():
    parser = argparse.ArgumentParser(description="Capture chessboard images for underwater calibration.")
    parser.add_argument("--camera", default="config/camera_rov_front.yaml")
    parser.add_argument("--output", required=True)
    parser.add_argument("--prefix", default="calib")
    args = parser.parse_args()

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    cap = open_camera(args.camera)
    count = len(list(output.glob("*.png")))

    print("Press SPACE or s to save, q to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Failed to read frame from camera")
        cv2.imshow("calibration_capture", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key in (ord("s"), 32):
            count += 1
            path = output / f"{args.prefix}_{count:03d}_{timestamp_for_path()}.png"
            cv2.imwrite(str(path), frame)
            print(f"saved {path}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
