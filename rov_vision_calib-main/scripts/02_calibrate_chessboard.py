import argparse
from pathlib import Path

import _bootstrap  # noqa: F401
from calibration.chessboard_calibrator import calibrate_chessboard
from utils.yaml_io import save_yaml


def main():
    parser = argparse.ArgumentParser(description="Calibrate underwater equivalent pinhole model from chessboard images.")
    parser.add_argument("--camera", default="config/camera_rov_front.yaml")
    parser.add_argument("--calib-template", default="config/calib_underwater_20_50cm_1080p.yaml")
    parser.add_argument("--images", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--update-latest", default=None)
    args = parser.parse_args()

    result = calibrate_chessboard(args.images, args.output, args.camera, args.calib_template)
    if args.update_latest:
        save_yaml(args.update_latest, result)
    else:
        latest = Path(args.output) / "calib_result.yaml"
        print(f"Calibration saved to {latest}")

    if args.update_latest:
        print(f"Calibration saved to {Path(args.output) / 'calib_result.yaml'} and {args.update_latest}")


if __name__ == "__main__":
    main()
