import unittest
from argparse import Namespace
from unittest.mock import patch

from tools.run_aruco_tracking_dryrun import (
    build_rc_dryrun_mapper,
    format_control_direction,
    main,
    resolve_log_path,
    update_pre_dock_observation_state,
)


class ArucoTrackingDryRunScriptTests(unittest.TestCase):
    def test_format_control_direction_summarizes_pose_and_command_signs(self):
        summary = format_control_direction(
            state={"x": 0.12, "y": -0.04, "z": 1.0, "yaw": 8.0, "status": "tracking", "lost_frames": 0},
            command={"vx": 0.08, "vy": -0.05, "vz": 0.02, "yaw_rate": -0.1},
            pose={"pose_valid": True, "reject_reason": ""},
        )

        self.assertIn("status=tracking", summary)
        self.assertIn("lost=0", summary)
        self.assertIn("pose_valid=True", summary)
        self.assertIn("x=0.120", summary)
        self.assertIn("z=1.000", summary)
        self.assertIn("vx=+0.080", summary)
        self.assertIn("vy=-0.050", summary)
        self.assertIn("forward=+0.080", summary)
        self.assertIn("right=-0.050", summary)
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
                device=None,
                desired_z=None,
                yaw_offset=None,
            ),
        ), patch("tools.run_aruco_tracking_dryrun.run_dryrun") as run_dryrun:
            main()

        self.assertIsNone(run_dryrun.call_args.kwargs["log_path"])

    def test_main_passes_device_override_to_dryrun(self):
        with patch(
            "tools.run_aruco_tracking_dryrun.parse_args",
            return_value=Namespace(
                config="config/settings.yaml",
                log="logs/out.csv",
                duration=60.0,
                print_interval=0.5,
                device="/dev/video0",
                desired_z=None,
                yaw_offset=None,
            ),
        ), patch("tools.run_aruco_tracking_dryrun.run_dryrun") as run_dryrun:
            main()

        self.assertEqual(run_dryrun.call_args.kwargs["device_override"], "/dev/video0")

    def test_main_passes_desired_z_override_to_dryrun(self):
        with patch(
            "tools.run_aruco_tracking_dryrun.parse_args",
            return_value=Namespace(
                config="config/settings.yaml",
                log="logs/out.csv",
                duration=60.0,
                print_interval=0.5,
                device="/dev/video0",
                desired_z=0.2,
                yaw_offset=None,
            ),
        ), patch("tools.run_aruco_tracking_dryrun.run_dryrun") as run_dryrun:
            main()

        self.assertEqual(run_dryrun.call_args.kwargs["desired_z_override"], 0.2)

    def test_main_passes_yaw_offset_override_to_dryrun(self):
        with patch(
            "tools.run_aruco_tracking_dryrun.parse_args",
            return_value=Namespace(
                config="config/settings.yaml",
                log="logs/out.csv",
                duration=60.0,
                print_interval=0.5,
                device="/dev/video0",
                desired_z=0.177,
                yaw_offset=-90.0,
            ),
        ), patch("tools.run_aruco_tracking_dryrun.run_dryrun") as run_dryrun:
            main()

        self.assertEqual(run_dryrun.call_args.kwargs["yaw_offset_override"], -90.0)

    def test_build_rc_dryrun_mapper_outputs_default_channels_when_runtime_rc_disabled(self):
        mapper = build_rc_dryrun_mapper(
            {
                "rc_override": {
                    "enabled": False,
                    "neutral_pwm": 1500,
                    "min_pwm": 1400,
                    "max_pwm": 1600,
                    "pwm_per_m_s": 250,
                    "pwm_per_rad_s": 200,
                    "channels": {},
                }
            }
        )

        channels = mapper.map_motion_command(
            {"forward_m_s": 0.04, "right_m_s": -0.04, "up_m_s": 0.02, "yaw_rate_rad_s": 0.1}
        )

        self.assertEqual(channels["ch5"], 1510)
        self.assertEqual(channels["ch6"], 1490)
        self.assertEqual(channels["ch3"], 1505)
        self.assertEqual(channels["ch4"], 1520)

    def test_pre_dock_frame_count_survives_empty_poll_until_recent_pose_expires(self):
        count, last_counter, recent = update_pre_dock_observation_state(
            current_count=2,
            last_tracker_valid_pose_frames=10,
            diagnostics={"tracker_valid_pose_frames": 10},
            latest_pose_age_s=0.2,
            config={"pre_dock_recent_observation_max_age_s": 0.5},
            has_valid_observation=False,
        )

        self.assertEqual(count, 2)
        self.assertEqual(last_counter, 10)
        self.assertTrue(recent)

        count, last_counter, recent = update_pre_dock_observation_state(
            current_count=count,
            last_tracker_valid_pose_frames=last_counter,
            diagnostics={"tracker_valid_pose_frames": 13},
            latest_pose_age_s=0.25,
            config={"pre_dock_recent_observation_max_age_s": 0.5},
            has_valid_observation=False,
        )

        self.assertEqual(count, 5)
        self.assertEqual(last_counter, 13)
        self.assertTrue(recent)

        count, last_counter, recent = update_pre_dock_observation_state(
            current_count=count,
            last_tracker_valid_pose_frames=last_counter,
            diagnostics={"tracker_valid_pose_frames": 13},
            latest_pose_age_s=0.8,
            config={"pre_dock_recent_observation_max_age_s": 0.5},
            has_valid_observation=False,
        )

        self.assertEqual(count, 0)
        self.assertEqual(last_counter, 13)
        self.assertFalse(recent)


if __name__ == "__main__":
    unittest.main()
