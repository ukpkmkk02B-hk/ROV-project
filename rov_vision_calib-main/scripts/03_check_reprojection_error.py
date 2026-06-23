import argparse

import _bootstrap  # noqa: F401
from utils.yaml_io import load_yaml


def main():
    parser = argparse.ArgumentParser(description="Print reprojection error summary from a calibration result.")
    parser.add_argument("--result", required=True)
    args = parser.parse_args()

    data = load_yaml(args.result)
    calib = data.get("calibration", {})
    acc = data.get("accuracy_evaluation", {})
    mean = acc.get("overall_reprojection_error_px")

    print(f"status: {calib.get('status')}")
    print(f"resolution: {calib.get('image_width')} x {calib.get('image_height')}")
    print(f"valid/total: {calib.get('valid_images')} / {calib.get('total_images')}")
    print(f"rejected: {calib.get('rejected_images')}")
    print(f"mean reprojection error: {mean} px")
    if mean is not None:
        if mean < 0.5:
            print("grade: excellent")
        elif mean <= 1.0:
            print("grade: usable")
        elif mean <= 1.5:
            print("grade: temporary only")
        else:
            print("grade: recapture recommended")


if __name__ == "__main__":
    main()
