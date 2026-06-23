import unittest
from argparse import Namespace
from unittest.mock import patch

from tools.run_aruco_tracking_dryrun import format_control_direction, is_plausible_pose, main, resolve_log_path


class ArucoTrackingDryRunScriptTests(unittest.TestCase):
    def test_format_control_direction_summarizes_pose_and_command_signs(self):
        summary = format_control_direction(
            state={"x": 0.12, "y": -0.04, "z": 1.0, "yaw": 8.0},
            command={"vx": 0.08, "vy": -0.05, "vz": 0.02, "yaw_rate": -0.1},
        )

        self.assertIn("x=0.120", summary)
        self.assertIn("z=1.000", summary)
        self.assertIn("vx=+0.080", summary)
        self.assertIn("vy=-0.050", summary)
        self.assertIn("yaw_rate=-0.100", summary)

    def test_resolve_log_path_uses_cli_path_before_config_default(self):
        self.assertEqual(
            resolve_log_path({"dryrun_log_path": "logs/from_config.csv"}, "logs/from_cli.csv").as_posix(),
            "logs/from_cli.csv",
        )
        self.assertEqual(
            resolve_log_path({"dryrun_log_path": "logs/from_config.csv"}, None).as_posix(),
            "logs/from_config.csv",
        )

    def test_main_allows_omitted_log_argument(self):
        with patch(
            "tools.run_aruco_tracking_dryrun.parse_args",
            return_value=Namespace(
                config="config/settings.yaml",
                log=None,
                duration=60.0,
                print_interval=0.5,
            ),
        ), patch("tools.run_aruco_tracking_dryrun.run_dryrun") as run_dryrun:
            main()

        self.assertIsNone(run_dryrun.call_args.kwargs["log_path"])

    def test_is_plausible_pose_rejects_obvious_pnp_outliers(self):
        config = {"max_abs_position_m": 5.0, "max_abs_yaw_deg": 180.0}

        self.assertTrue(is_plausible_pose({"x": 0.1, "y": -0.1, "z": 0.8, "yaw": 20.0}, config))
        self.assertFalse(is_plausible_pose({"x": 100.0, "y": 0.0, "z": 0.8, "yaw": 20.0}, config))
        self.assertFalse(is_plausible_pose({"x": 0.1, "y": 0.0, "z": -0.1, "yaw": 20.0}, config))
        self.assertFalse(is_plausible_pose({"x": 0.1, "y": 0.0, "z": 0.8, "yaw": 2875.0}, config))


if __name__ == "__main__":
    unittest.main()
