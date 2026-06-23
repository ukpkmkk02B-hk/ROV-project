import argparse
from pathlib import Path

import cv2

import _bootstrap  # noqa: F401
from camera.uvc_camera import open_camera
from utils.time_sync import timestamp_for_path


def main():
    parser = argparse.ArgumentParser(description="Capture video or frames for ArUco validation.")
    parser.add_argument("--camera", default="config/camera_rov_front.yaml")
    parser.add_argument("--output", required=True)
    parser.add_argument("--record-video", action="store_true")
    args = parser.parse_args()

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    cap = open_camera(args.camera)
    writer = None

    if args.record_video:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Failed to read first frame")
        h, w = frame.shape[:2]
        video_path = output / f"aruco_{timestamp_for_path()}.avi"
        writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"MJPG"), 30, (w, h))
        writer.write(frame)
        print(f"recording {video_path}; press q to stop")

    count = 0
    print("Press SPACE or s to save frame, q to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Failed to read frame from camera")
        if writer:
            writer.write(frame)
        cv2.imshow("aruco_validation_capture", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key in (ord("s"), 32):
            count += 1
            path = output / f"aruco_{count:03d}_{timestamp_for_path()}.png"
            cv2.imwrite(str(path), frame)
            print(f"saved {path}")

    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
