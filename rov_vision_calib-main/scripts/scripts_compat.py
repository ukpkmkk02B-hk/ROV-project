import importlib.util
from pathlib import Path
import sys


def run_dryrun_with_notice(args):
    print("Water-test script is intentionally logging-only in this scaffold.")
    script = Path(__file__).with_name("06_visual_servo_dryrun.py")
    spec = importlib.util.spec_from_file_location("visual_servo_dryrun", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys.argv = [
        str(script),
        "--camera",
        args.camera,
        "--aruco",
        args.aruco,
        "--servo",
        args.servo,
        "--output",
        args.output,
    ]
    module.main()
