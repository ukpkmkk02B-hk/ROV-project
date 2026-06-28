import unittest
from argparse import Namespace
from unittest.mock import patch

import numpy as np

from tools.run_aruco_tracking_dryrun import (
    build_rc_dryrun_mapper,
    compute_dryrun_command,
    format_control_direction,
    main,
    resolve_log_path,
    show_preview_frame,
    update_pre_dock_observation_state,
)
from modules.controller.visual_tracking_controller import VisualTrackingController


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
                preview=False,
                preview_scale=1.0,
                preview_fps=10.0,
                detection_scale=1.0,
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
                preview=False,
                preview_scale=1.0,
                preview_fps=10.0,
                detection_scale=1.0,
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
                preview=False,
                preview_scale=1.0,
                preview_fps=10.0,
                detection_scale=1.0,
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
                preview=False,
                preview_scale=1.0,
                preview_fps=10.0,
                detection_scale=1.0,
            ),
        ), patch("tools.run_aruco_tracking_dryrun.run_dryrun") as run_dryrun:
            main()

        self.assertEqual(run_dryrun.call_args.kwargs["yaw_offset_override"], -90.0)

    def test_main_passes_preview_options_to_dryrun(self):
        with patch(
            "tools.run_aruco_tracking_dryrun.parse_args",
            return_value=Namespace(
                config="config/settings.yaml",
                log="logs/out.csv",
                duration=20.0,
                print_interval=0.5,
                device="/dev/camera_main",
                desired_z=0.173,
                yaw_offset=None,
                preview=True,
                preview_scale=0.5,
                preview_fps=8.0,
                detection_scale=0.5,
            ),
        ), patch("tools.run_aruco_tracking_dryrun.run_dryrun") as run_dryrun:
            main()

        self.assertTrue(run_dryrun.call_args.kwargs["preview"])
        self.assertEqual(run_dryrun.call_args.kwargs["preview_scale"], 0.5)
        self.assertEqual(run_dryrun.call_args.kwargs["preview_fps"], 8.0)
        self.assertEqual(run_dryrun.call_args.kwargs["detection_scale"], 0.5)

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

    def test_show_preview_frame_draws_scaled_frame_and_quits_on_q(self):
        class FakeTracker:
            def get_annotated_frame(self):
                return np.zeros((12, 16, 3), dtype=np.uint8)

        class FakeCv2:
            FONT_HERSHEY_SIMPLEX = 0
            LINE_AA = 16
            INTER_AREA = 3

            def __init__(self):
                self.put_text_calls = []
                self.resize_calls = []
                self.imshow_calls = []

            def putText(self, *args):
                self.put_text_calls.append(args)

            def resize(self, frame, dsize, fx, fy, interpolation):
                self.resize_calls.append((fx, fy, interpolation))
                return frame

            def imshow(self, name, frame):
                self.imshow_calls.append((name, frame.shape))

            def waitKey(self, delay):
                return ord("q")

        fake_cv2 = FakeCv2()

        quit_requested = show_preview_frame(
            FakeTracker(),
            state={"status": "tracking", "lost_frames": 0},
            rc_override={"ch3": 1500, "ch4": 1501, "ch5": 1502, "ch6": 1503},
            pre_dock_ready=True,
            preview_scale=0.5,
            cv2_module=fake_cv2,
        )

        self.assertTrue(quit_requested)
        self.assertGreaterEqual(len(fake_cv2.put_text_calls), 3)
        self.assertEqual(fake_cv2.resize_calls, [(0.5, 0.5, fake_cv2.INTER_AREA)])
        self.assertEqual(fake_cv2.imshow_calls[0][0], "ArUco tracking dry-run")

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

    def test_lost_state_resets_controller_smoothing_before_reacquire(self):
        controller = VisualTrackingController(
            desired_z_m=0.8,
            control_mode="pid",
            command_smoothing_alpha=0.5,
            pid_config={
                "forward": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "right": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "up": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
                "yaw": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "output_limit": 0.4},
            },
        )
        state = {
            "status": "tracking",
            "forward_m": 1.0,
            "right_m": 0.0,
            "up_m": 0.0,
            "yaw_error_deg": 0.0,
            "timestamp": 1.0,
        }

        first = compute_dryrun_command(controller, state)
        lost = compute_dryrun_command(controller, {"status": "lost"})
        reacquired = compute_dryrun_command(controller, {**state, "timestamp": 2.0})

        self.assertAlmostEqual(first["forward_m_s"], 0.1)
        self.assertEqual(lost["forward_m_s"], 0.0)
        self.assertAlmostEqual(reacquired["forward_m_s"], 0.1)


if __name__ == "__main__":
    unittest.main()
