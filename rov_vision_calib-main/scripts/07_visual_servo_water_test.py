import argparse

import _bootstrap  # noqa: F401
from scripts_compat import run_dryrun_with_notice


def main():
    parser = argparse.ArgumentParser(description="Water-test entrypoint. This logs commands but does not send RC output.")
    parser.add_argument("--camera", default="config/camera_rov_front.yaml")
    parser.add_argument("--aruco", default="config/aruco_config.yaml")
    parser.add_argument("--servo", default="config/servo_config.yaml")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    run_dryrun_with_notice(args)


if __name__ == "__main__":
    main()
