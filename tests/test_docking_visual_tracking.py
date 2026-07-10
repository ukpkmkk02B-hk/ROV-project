import importlib
import sys
import threading
import types
import unittest
from unittest.mock import patch


def import_docking_with_stubs():
    sys.modules.setdefault("yaml", types.SimpleNamespace(safe_load=lambda f: {}))
    sys.modules.setdefault("pupil_apriltags", types.SimpleNamespace(Detector=lambda **kwargs: object()))
    cv2_stub = types.SimpleNamespace()
    sys.modules.setdefault("cv2", cv2_stub)

    def validate_pose_quality_stub(pose, config):
        if not pose or pose.get("pose_valid") is False:
            return False
        max_abs_position = float((config or {}).get("max_abs_position_m", 5.0))
        for key in ("x", "y", "z"):
            if abs(float(pose.get(key, 0.0))) > max_abs_position:
                pose["pose_valid"] = False
                pose["reject_reason"] = "position_out_of_range"
                return False
        return True

    marker_tracker_module_name = "modules.perception.marker_tracker"
    original_marker_tracker = sys.modules.get(marker_tracker_module_name)
    sys.modules[marker_tracker_module_name] = types.SimpleNamespace(
        validate_pose_quality=validate_pose_quality_stub
    )

    pymavlink = types.ModuleType("pymavlink")
    mavutil = types.SimpleNamespace(mavlink=types.SimpleNamespace())
    pymavlink.mavutil = mavutil
    sys.modules["pymavlink"] = pymavlink
    sys.modules["pymavlink.mavutil"] = mavutil

    module_name = "modules.tasks.docking_task"
    sys.modules.pop(module_name, None)
    try:
        return importlib.import_module(module_name)
    finally:
        if original_marker_tracker is None:
            sys.modules.pop(marker_tracker_module_name, None)
        else:
            sys.modules[marker_tracker_module_name] = original_marker_tracker


class FakeCamera:
    def __init__(self, poses, detected=True, diagnostics=None):
        self.poses = list(poses)
        self.detected = detected
        self.diagnostics = diagnostics or {}

    def get_pose(self):
        if self.poses:
            return self.poses.pop(0)
        return None

    def target_detected(self):
        return self.detected

    def target_lost(self):
        return not self.detected

    def get_diagnostics(self):
        return dict(self.diagnostics)


class FakePixhawk:
    def __init__(self):
        self.commands = []
        self.stopped = False
        self.arm_calls = 0
        self.mode_calls = []

    def is_armed(self):
        return True

    def arm_vehicle(self):
        self.arm_calls += 1

    def set_mode(self, mode):
        self.mode_calls.append(mode)
        self.mode = mode
        return True

    def send_velocity_command(self, command):
        self.commands.append(command)

    def send_rc_override(self, channels):
        self.commands.append({"rc": channels})

    def stop_motion(self):
        self.stopped = True

    def stop(self):
        self.stopped = True


class FakeStateMachine:
    def __init__(self):
        self.events = []

    def notify_task_start(self, name):
        self.events.append(("start", name))

    def notify_task_stop(self, name):
        self.events.append(("stop", name))

    def notify_task_completed(self, name):
        self.events.append(("completed", name))

    def notify_task_failed(self, name):
        self.events.append(("failed", name))

    def start_task(self, name):
        self.events.append(("start_task", name))

    def promote_current_task(self, name):
        self.events.append(("promote", name))
        return True


class DockingVisualTrackingTests(unittest.TestCase):
    def test_dry_run_requires_consecutive_valid_observations_before_pre_dock_ready(self):
        module = import_docking_with_stubs()
        pose = {
            "x": 0.01,
            "y": -0.01,
            "z": 0.80,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 2.0,
            "timestamp": 10.0,
        }
        pixhawk = FakePixhawk()
        task = module.DockingTask(
            camera=FakeCamera([pose]),
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.8,
                "max_lost_frames": 3,
                "min_pre_dock_valid_frames": 2,
                "pre_dock_position_tolerance_m": 0.05,
                "pre_dock_distance_tolerance_m": 0.05,
                "pre_dock_yaw_tolerance_deg": 5.0,
            },
        )

        with patch("builtins.print"):
            task.start()
            task.run()
            task.camera.poses.append(dict(pose, timestamp=10.1))
            task.run()
        status = task.get_status()

        self.assertEqual(status["stage"], module.DockingTask.STATE_PRE_ALIGN)
        self.assertTrue(status["pre_dock_ready"])
        self.assertIn("control_cmd", status)
        self.assertEqual(pixhawk.commands, [])
        self.assertEqual(pixhawk.arm_calls, 0)
        self.assertEqual(pixhawk.mode_calls, [])

    def test_tracking_mission_does_not_auto_enter_pre_align_when_ready(self):
        module = import_docking_with_stubs()
        pose = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.8,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "timestamp": 10.0,
        }
        task = module.DockingTask(
            camera=FakeCamera([pose]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.8,
                "min_pre_dock_valid_frames": 1,
                "pre_dock_position_tolerance_m": 0.05,
                "pre_dock_distance_tolerance_m": 0.05,
                "pre_dock_yaw_tolerance_deg": 5.0,
            },
        )

        with patch("builtins.print"):
            task.start()
            task.run()

        status = task.get_status()
        self.assertEqual(status["name"], "tracking")
        self.assertEqual(status["mission_mode"], "tracking")
        self.assertEqual(status["stage"], module.DockingTask.STATE_TRACK)
        self.assertTrue(status["tracking_ready"])
        self.assertFalse(status["pre_dock_ready"])
        self.assertIsNotNone(status["last_pose"])
        self.assertIsNotNone(status["filtered_state"])
        self.assertIn("control_cmd", status)
        self.assertIn("rc_override", status)

    def test_engage_docking_promotes_tracking_without_clearing_state(self):
        module = import_docking_with_stubs()
        pose = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.8,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "timestamp": 10.0,
        }
        task = module.DockingTask(
            camera=FakeCamera([pose]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.8,
                "min_pre_dock_valid_frames": 1,
            },
        )

        with patch("builtins.print"):
            task.start()
            task.run()
            previous_pose = task.last_pose
            previous_state = task.filtered_state
            previous_rc = dict(task.last_rc_override)
            result = task.engage_docking(source="unit_test")

        status = task.get_status()
        self.assertTrue(result["accepted"])
        self.assertEqual(status["name"], "docking")
        self.assertEqual(status["mission_mode"], "docking")
        self.assertEqual(status["stage"], module.DockingTask.STATE_PRE_ALIGN)
        self.assertIs(task.last_pose, previous_pose)
        self.assertEqual(task.filtered_state, previous_state)
        self.assertEqual(task.last_rc_override, previous_rc)

    def test_engage_docking_resets_docking_timeout_clock(self):
        module = import_docking_with_stubs()
        pose = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.8,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "timestamp": 10.0,
        }
        pixhawk = FakePixhawk()
        task = module.DockingTask(
            camera=FakeCamera([pose]),
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.8,
                "min_pre_dock_valid_frames": 1,
            },
        )

        with patch("builtins.print"), patch.object(module.time, "time", return_value=0.0):
            task.start()
        with patch("builtins.print"), patch.object(module.time, "time", return_value=10.0):
            task.run()
        with patch.object(module.time, "time", return_value=70.0):
            result = task.engage_docking(source="unit_test")

        task.camera.poses.append(dict(pose, timestamp=71.0))
        with patch("builtins.print"), patch.object(module.time, "time", return_value=71.0):
            task.run()

        self.assertTrue(result["accepted"])
        self.assertNotEqual(task.status, "failed")
        self.assertNotEqual(task.stage, module.DockingTask.STATE_FAILED)
        self.assertFalse(pixhawk.stopped)

    def test_engage_docking_slew_starts_from_latest_tracking_ch3(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={
                "enable_motion": False,
                "output_backend": "rc_override",
                "desired_z_m": 0.5,
                "min_pre_dock_valid_frames": 1,
                "pre_align_buoyancy_hold_pwm": 1600,
                "pre_align_down_pwm_max": 1700,
                "camera_to_body": {"forward_axis": "y", "up_axis": "z", "up_sign": -1.0},
                "rc_override": {
                    "enabled": True,
                    "channels": {"forward": "ch5", "right": "ch6", "up": "ch3", "yaw": "ch4"},
                },
            },
        )
        state = {
            "forward_m": 0.0,
            "right_m": 0.0,
            "up_m": -0.5,
            "yaw_error_deg": 0.0,
            "vz": 0.0,
            "status": "tracking",
            "timestamp": 10.0,
            "has_recent_valid_observation": True,
            "latest_pose_age_s": 0.1,
            "pre_dock_valid_frame_count": 1,
        }
        task.status = "running"
        task.filtered_state = dict(state)
        task.last_rc_override = {f"ch{i}": 1500 for i in range(1, 9)}
        task.last_rc_override["ch3"] = 1600

        with patch("builtins.print"):
            result = task.engage_docking(source="unit_test")
            task._track(dict(state, timestamp=10.05))

        self.assertTrue(result["accepted"])
        self.assertEqual(task.last_rc_override["ch3"], 1600)

    def test_engage_docking_rejects_real_motion_on_non_rc_backend(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={"enable_motion": True, "output_backend": "mavlink_velocity"},
        )
        task.status = "running"
        task.filtered_state = {"has_recent_valid_observation": True}

        result = task.engage_docking(source="unit_test")

        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"], "docking_vertical_requires_rc_override")

    def test_manual_dock_confirm_rejects_during_tracking_mission(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={"enable_motion": False},
        )

        with patch("builtins.print"):
            task.start()
            result = task.confirm_manual_dock(source="unit_test")

        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"], "not_docking")

    def test_tracking_mission_does_not_timeout_after_sixty_seconds(self):
        module = import_docking_with_stubs()
        pose = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.8,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "timestamp": 61.0,
        }
        pixhawk = FakePixhawk()
        task = module.DockingTask(
            camera=FakeCamera([pose]),
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={"enable_motion": False, "desired_z_m": 0.8},
        )

        with patch("builtins.print"), patch.object(module.time, "time", return_value=61.0):
            task.start_time = 0.0
            task.status = "running"
            task.run()

        self.assertEqual(task.status, "running")
        self.assertEqual(task.stage, module.DockingTask.STATE_TRACK)
        self.assertFalse(pixhawk.stopped)

    def test_tracking_loss_waits_without_search_timeout_failure(self):
        module = import_docking_with_stubs()
        pixhawk = FakePixhawk()
        state_machine = FakeStateMachine()
        task = module.DockingTask(
            camera=FakeCamera([], detected=False),
            pixhawk=pixhawk,
            state_machine=state_machine,
            mission_mode="tracking",
            tracking_config={
                "enable_motion": True,
                "required_mode": "MANUAL",
                "output_backend": "rc_override",
                "rc_override": {
                    "enabled": True,
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    },
                },
            },
        )

        with patch("builtins.print"):
            task.start()
            for now in (10.0, 20.0, 30.0, 40.0):
                task.search_start_time = 1.0
                with patch.object(module.time, "time", return_value=now):
                    task.run()

        status = task.get_status()
        self.assertEqual(status["status"], "running")
        self.assertEqual(status["stage"], module.DockingTask.STATE_SEARCH)
        self.assertEqual(status["mission_mode"], "tracking")
        self.assertEqual(status["filtered_state"]["status"], "lost")
        self.assertEqual(status["attempts"], 0)
        self.assertNotIn(("failed", "tracking"), state_machine.events)
        self.assertEqual(pixhawk.commands[-1], {"rc": {f"ch{i}": 1500 for i in range(1, 9)}})

    def test_tracking_reacquires_after_loss_without_restart(self):
        module = import_docking_with_stubs()
        first_pose = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.8,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "timestamp": 10.0,
        }
        reacquired_pose = dict(first_pose, timestamp=20.0)
        camera = FakeCamera([first_pose, None, reacquired_pose])
        task = module.DockingTask(
            camera=camera,
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={"enable_motion": False, "desired_z_m": 0.8},
        )

        with patch("builtins.print"):
            task.start()
            task.run()
            camera.detected = False
            task.run()
            lost_status = task.get_status()
            camera.detected = True
            task.run()

        self.assertEqual(lost_status["stage"], module.DockingTask.STATE_SEARCH)
        self.assertEqual(lost_status["filtered_state"]["status"], "lost")
        status = task.get_status()
        self.assertEqual(status["status"], "running")
        self.assertEqual(status["stage"], module.DockingTask.STATE_TRACK)
        self.assertEqual(status["mission_mode"], "tracking")
        self.assertEqual(status["last_pose"], reacquired_pose)

    def test_docking_loss_downgrades_to_tracking_before_reacquire(self):
        module = import_docking_with_stubs()
        pose = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.8,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "timestamp": 10.0,
        }
        state_machine = FakeStateMachine()
        camera = FakeCamera([pose, None, dict(pose, timestamp=20.0)])
        task = module.DockingTask(
            camera=camera,
            pixhawk=FakePixhawk(),
            state_machine=state_machine,
            mission_mode="tracking",
            tracking_config={"enable_motion": False, "desired_z_m": 0.8, "min_pre_dock_valid_frames": 1},
        )

        with patch("builtins.print"):
            task.start()
            task.run()
            engage_result = task.engage_docking(source="unit_test")
            camera.detected = False
            task.run()
            lost_status = task.get_status()
            confirm_result = task.confirm_manual_dock(source="unit_test")
            camera.detected = True
            task.run()

        self.assertTrue(engage_result["accepted"])
        self.assertEqual(lost_status["mission_mode"], "tracking")
        self.assertEqual(lost_status["name"], "tracking")
        self.assertEqual(lost_status["stage"], module.DockingTask.STATE_SEARCH)
        self.assertFalse(lost_status["pre_dock_ready"])
        self.assertFalse(confirm_result["accepted"])
        self.assertEqual(confirm_result["reason"], "not_docking")
        self.assertIn(("promote", "tracking"), state_machine.events)
        status = task.get_status()
        self.assertEqual(status["mission_mode"], "tracking")
        self.assertEqual(status["stage"], module.DockingTask.STATE_TRACK)

    def test_no_new_pose_while_target_visible_keeps_last_rc_override(self):
        module = import_docking_with_stubs()
        pose = {
            "x": 0.05,
            "y": 0.0,
            "z": 0.9,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "timestamp": 10.0,
        }
        pixhawk = FakePixhawk()
        task = module.DockingTask(
            camera=FakeCamera([pose, None]),
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={
                "enable_motion": True,
                "required_mode": "MANUAL",
                "output_backend": "rc_override",
                "desired_z_m": 0.8,
                "rc_override": {
                    "enabled": True,
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    },
                },
            },
        )

        with patch("builtins.print"):
            task.start()
            task.run()
            first_rc = pixhawk.commands[-1]
            task.run()

        neutral_rc = {"rc": {f"ch{i}": 1500 for i in range(1, 9)}}
        self.assertNotEqual(first_rc, neutral_rc)
        self.assertEqual(pixhawk.commands[-1], first_rc)
        self.assertEqual(task.stage, module.DockingTask.STATE_TRACK)
        self.assertEqual(task.get_status()["filtered_state"]["status"], "tracking")

    def test_invalid_pose_while_target_visible_keeps_last_rc_override(self):
        module = import_docking_with_stubs()
        valid_pose = {
            "x": 0.05,
            "y": 0.0,
            "z": 0.9,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "timestamp": 10.0,
        }
        invalid_pose = dict(valid_pose, timestamp=10.1, pose_valid=False, reject_reason="reprojection_error_high")
        pixhawk = FakePixhawk()
        task = module.DockingTask(
            camera=FakeCamera([valid_pose, invalid_pose], detected=True),
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={
                "enable_motion": True,
                "required_mode": "MANUAL",
                "output_backend": "rc_override",
                "desired_z_m": 0.8,
                "rc_override": {
                    "enabled": True,
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    },
                },
            },
        )

        with patch("builtins.print"):
            task.start()
            task.run()
            first_rc = pixhawk.commands[-1]
            task.run()

        neutral_rc = {"rc": {f"ch{i}": 1500 for i in range(1, 9)}}
        self.assertNotEqual(first_rc, neutral_rc)
        self.assertEqual(pixhawk.commands[-1], first_rc)
        self.assertEqual(task.stage, module.DockingTask.STATE_TRACK)
        self.assertEqual(task.get_status()["filtered_state"]["status"], "tracking")

    def test_capture_failed_diagnostics_forces_neutral_rc_even_before_target_lost_threshold(self):
        module = import_docking_with_stubs()
        pose = {
            "x": 0.05,
            "y": 0.0,
            "z": 0.9,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "timestamp": 10.0,
        }
        pixhawk = FakePixhawk()
        camera = FakeCamera([pose, None], detected=True, diagnostics={"reject_reason": "capture_failed"})
        task = module.DockingTask(
            camera=camera,
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={
                "enable_motion": True,
                "required_mode": "MANUAL",
                "output_backend": "rc_override",
                "desired_z_m": 0.8,
                "rc_override": {
                    "enabled": True,
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    },
                },
            },
        )

        with patch("builtins.print"):
            task.start()
            task.run()
            task.run()

        self.assertEqual(pixhawk.commands[-1], {"rc": {f"ch{i}": 1500 for i in range(1, 9)}})
        self.assertEqual(task.stage, module.DockingTask.STATE_SEARCH)
        self.assertEqual(task.get_status()["filtered_state"]["status"], "lost")

    def test_failure_sends_neutral_without_closing_pixhawk_connection(self):
        module = import_docking_with_stubs()
        pixhawk = FakePixhawk()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            tracking_config={
                "enable_motion": True,
                "output_backend": "rc_override",
                "rc_override": {
                    "enabled": True,
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    },
                },
            },
        )

        with patch("builtins.print"):
            task._handle_failure()

        self.assertEqual(task.status, "failed")
        self.assertFalse(pixhawk.stopped)
        self.assertEqual(pixhawk.commands[-1], {"rc": {f"ch{i}": 1500 for i in range(1, 9)}})

    def test_recent_predicted_state_can_trigger_pre_dock_ready(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.8,
                "min_pre_dock_valid_frames": 1,
                "pre_dock_recent_observation_max_age_s": 0.5,
            },
        )
        task.valid_observation_count = 1
        task.last_valid_observation_time = 100.0
        task.stage = module.DockingTask.STATE_PRE_ALIGN

        with patch.object(module.time, "time", return_value=100.2), patch("builtins.print"):
            state = task._annotate_state(
                {"x": 0.0, "y": 0.0, "z": 0.8, "yaw": 0.0, "status": "predicted", "lost_frames": 1},
                has_valid_observation=False,
            )
            task._track(state)

        self.assertEqual(task.stage, module.DockingTask.STATE_PRE_ALIGN)
        self.assertTrue(task.get_status()["pre_dock_ready"])

    def test_expired_predicted_state_does_not_trigger_pre_dock_ready(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.8,
                "min_pre_dock_valid_frames": 1,
                "pre_dock_recent_observation_max_age_s": 0.5,
            },
        )
        task.valid_observation_count = 1
        task.last_valid_observation_time = 100.0

        with patch.object(module.time, "time", return_value=101.0), patch("builtins.print"):
            state = task._annotate_state(
                {"x": 0.0, "y": 0.0, "z": 0.8, "yaw": 0.0, "status": "predicted", "lost_frames": 1},
                has_valid_observation=False,
            )
            task._track(state)

        self.assertFalse(task.get_status()["pre_dock_ready"])

    def test_motion_enabled_uses_manual_required_mode(self):
        module = import_docking_with_stubs()
        pixhawk = FakePixhawk()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={"enable_motion": True, "required_mode": "MANUAL"},
        )

        with patch("builtins.print"):
            task.start()

        self.assertEqual(pixhawk.mode_calls, ["MANUAL"])

    def test_motion_enabled_rejects_non_manual_required_mode_before_mode_change_or_arm(self):
        module = import_docking_with_stubs()

        class OrderedPixhawk(FakePixhawk):
            def __init__(self):
                super().__init__()
                self.events = []

            def arm_vehicle(self):
                self.events.append("arm")
                super().arm_vehicle()

            def set_mode(self, mode):
                self.events.append(f"mode:{mode}")
                return super().set_mode(mode)

        pixhawk = OrderedPixhawk()
        state_machine = FakeStateMachine()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=state_machine,
            tracking_config={"enable_motion": True, "required_mode": "STABILIZE"},
        )

        with patch("builtins.print"):
            task.start()

        self.assertEqual(task.status, "failed")
        self.assertEqual(pixhawk.events, [])
        self.assertEqual(pixhawk.mode_calls, [])
        self.assertEqual(pixhawk.arm_calls, 0)
        self.assertEqual(task.last_failure_reason, "unsupported_required_mode: STABILIZE")
        self.assertIn(("failed", "docking"), state_machine.events)

    def test_motion_enabled_requires_manual_arm_by_default(self):
        module = import_docking_with_stubs()

        class OrderedPixhawk(FakePixhawk):
            def __init__(self):
                super().__init__()
                self.armed = False
                self.events = []

            def is_armed(self):
                return self.armed

            def arm_vehicle(self):
                self.events.append("arm")
                self.arm_calls += 1
                self.armed = True

            def set_mode(self, mode):
                self.events.append(f"mode:{mode}")
                return super().set_mode(mode)

        pixhawk = OrderedPixhawk()
        state_machine = FakeStateMachine()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=state_machine,
            mission_mode="tracking",
            tracking_config={"enable_motion": True, "required_mode": "MANUAL"},
        )

        with patch("builtins.print"), patch.object(module.time, "sleep", return_value=None):
            task.start()

        self.assertEqual(task.status, "failed")
        self.assertEqual(pixhawk.events, ["mode:MANUAL"])
        self.assertEqual(pixhawk.mode_calls, ["MANUAL"])
        self.assertEqual(pixhawk.arm_calls, 0)
        self.assertIn(("failed", "tracking"), state_machine.events)
        self.assertEqual(task.last_failure_reason, "vehicle_not_armed")

    def test_motion_enabled_allows_auto_arm_when_explicitly_configured(self):
        module = import_docking_with_stubs()

        class OrderedPixhawk(FakePixhawk):
            def __init__(self):
                super().__init__()
                self.armed = False
                self.events = []

            def is_armed(self):
                return self.armed

            def arm_vehicle(self):
                self.events.append("arm")
                self.arm_calls += 1
                self.armed = True

            def set_mode(self, mode):
                self.events.append(f"mode:{mode}")
                return super().set_mode(mode)

        pixhawk = OrderedPixhawk()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={
                "enable_motion": True,
                "required_mode": "MANUAL",
                "allow_auto_arm_on_start": True,
            },
        )

        with patch("builtins.print"), patch.object(module.time, "sleep", return_value=None):
            task.start()

        self.assertEqual(pixhawk.events, ["mode:MANUAL", "arm"])
        self.assertEqual(pixhawk.mode_calls, ["MANUAL"])
        self.assertEqual(pixhawk.arm_calls, 1)

    def test_rc_override_backend_requires_explicit_mapping_before_motion(self):
        module = import_docking_with_stubs()
        pixhawk = FakePixhawk()
        state_machine = FakeStateMachine()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=state_machine,
            tracking_config={"enable_motion": True, "output_backend": "rc_override", "rc_override": {"enabled": True}},
        )

        with patch("builtins.print"):
            task.start()

        self.assertEqual(task.status, "failed")
        self.assertIn(("failed", "docking"), state_machine.events)

    def test_rc_override_backend_requires_explicit_enable_before_motion(self):
        module = import_docking_with_stubs()
        pixhawk = FakePixhawk()
        state_machine = FakeStateMachine()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=state_machine,
            tracking_config={
                "enable_motion": True,
                "output_backend": "rc_override",
                "rc_override": {
                    "enabled": False,
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    },
                },
            },
        )

        with patch("builtins.print"):
            task.start()

        self.assertEqual(task.status, "failed")
        self.assertEqual(pixhawk.arm_calls, 0)
        self.assertEqual(pixhawk.mode_calls, [])
        self.assertIn(("failed", "docking"), state_machine.events)

    def test_motion_enabled_rejects_unknown_output_backend_before_arming(self):
        module = import_docking_with_stubs()
        pixhawk = FakePixhawk()
        state_machine = FakeStateMachine()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=state_machine,
            tracking_config={"enable_motion": True, "output_backend": "unknown_backend"},
        )

        with patch("builtins.print"):
            task.start()

        self.assertEqual(task.status, "failed")
        self.assertEqual(pixhawk.arm_calls, 0)
        self.assertEqual(pixhawk.mode_calls, [])

    def test_motion_enabled_docking_rejects_mavlink_velocity_backend(self):
        module = import_docking_with_stubs()
        pixhawk = FakePixhawk()
        state_machine = FakeStateMachine()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=state_machine,
            mission_mode="docking",
            tracking_config={"enable_motion": True, "output_backend": "mavlink_velocity"},
        )

        with patch("builtins.print"):
            task.start()

        self.assertEqual(task.status, "failed")
        self.assertEqual(task.last_failure_reason, "docking_vertical_requires_rc_override")
        self.assertEqual(pixhawk.mode_calls, [])
        self.assertIn(("failed", "docking"), state_machine.events)

    def test_invalid_pose_is_not_used_to_update_filter_or_control(self):
        module = import_docking_with_stubs()
        invalid_pose = {
            "x": 100.0,
            "y": 0.0,
            "z": 0.8,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "timestamp": 10.0,
        }
        task = module.DockingTask(
            camera=FakeCamera([invalid_pose]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            tracking_config={
                "enable_motion": False,
                "max_abs_position_m": 5.0,
                "max_lost_frames": 3,
            },
        )

        with patch("builtins.print"):
            task.start()
            task.run()

        status = task.get_status()
        self.assertEqual(status["stage"], module.DockingTask.STATE_SEARCH)
        self.assertFalse(status["pre_dock_ready"])
        self.assertIsNone(status["last_pose"])
        self.assertIsNone(status["filtered_state"])

    def test_stationary_child_docking_mode_is_reported_without_child_motion(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            tracking_config={
                "enable_motion": False,
                "target_motion_mode": "stationary_child",
                "child_command_mode": "disabled",
                "control_deadband_m": 0.01,
                "command_smoothing_alpha": 0.6,
            },
        )

        with patch("builtins.print"):
            task.start()

        status = task.get_status()
        self.assertEqual(status["target_motion_mode"], "stationary_child")
        self.assertEqual(status["child_command_mode"], "disabled")
        self.assertFalse(status["enable_motion"])
        self.assertEqual(task.tracking_ctrl.control_deadband_m, 0.01)
        self.assertEqual(task.tracking_ctrl.command_smoothing_alpha, 0.6)

    def test_manual_dock_confirm_rejects_before_visual_pre_align(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            tracking_config={"enable_motion": False},
        )

        with patch("builtins.print"):
            task.start()
            result = task.confirm_manual_dock(source="unit_test")

        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"], "not_pre_aligned")
        status = task.get_status()
        self.assertFalse(status["manual_dock_confirmed"])
        self.assertEqual(status["dock_completion_rejected_reason"], "not_pre_aligned")
        self.assertEqual(status["status"], "running")

    def test_manual_dock_confirm_enters_buoyancy_hold_without_starting_charging(self):
        module = import_docking_with_stubs()
        state_machine = FakeStateMachine()
        pixhawk = FakePixhawk()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=state_machine,
            tracking_config={"enable_motion": False},
        )

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_PRE_ALIGN
            task.pre_dock_ready = True
            result = task.confirm_manual_dock(source="unit_test")

        self.assertTrue(result["accepted"])
        status = task.get_status()
        self.assertEqual(status["stage"], module.DockingTask.STATE_DOCKED_HOLD)
        self.assertEqual(status["status"], "running")
        self.assertTrue(status["manual_dock_confirmed"])
        self.assertTrue(status["docked_hold_active"])
        self.assertEqual(status["rc_override"]["ch3"], 1600)
        self.assertEqual(status["dock_completion_source"], "unit_test")
        self.assertFalse(status["charging_start_requested"])
        self.assertFalse(pixhawk.stopped)
        self.assertNotIn(("completed", "docking"), state_machine.events)
        self.assertNotIn(("start_task", "charging"), state_machine.events)

    def test_manual_dock_confirm_does_not_start_charging_during_docked_hold(self):
        module = import_docking_with_stubs()
        state_machine = FakeStateMachine()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=state_machine,
            tracking_config={"enable_motion": False, "start_charging_after_dock": True},
        )

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_PRE_ALIGN
            task.pre_dock_ready = True
            result = task.confirm_manual_dock(source="unit_test")

        self.assertTrue(result["accepted"])
        self.assertFalse(task.get_status()["charging_start_requested"])
        self.assertNotIn(("start_task", "charging"), state_machine.events)

    def test_tracking_capture_ch3_uses_latest_final_rc_override(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.8,
                "output_backend": "rc_override",
                "rc_override": {
                    "enabled": True,
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    },
                },
            },
        )
        state = {
            "forward_m": 0.8,
            "right_m": 0.0,
            "up_m": 0.1,
            "yaw_error_deg": 0.0,
            "status": "tracking",
            "timestamp": 10.0,
            "has_recent_valid_observation": True,
            "latest_pose_age_s": 0.1,
            "pre_dock_valid_frame_count": 3,
        }

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_TRACK
            task._track(state)
            result = task.capture_tracking_ch3(source="unit_test")

        self.assertTrue(result["accepted"])
        self.assertEqual(result["captured_hold_ch3_pwm"], task.last_rc_override["ch3"])
        self.assertTrue(task.get_status()["captured_hold_ch3_available"])

    def test_hold_captured_ch3_keeps_ch3_fixed_and_removes_vertical_pid_output(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="tracking",
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.8,
                "output_backend": "rc_override",
                "rc_override": {
                    "enabled": True,
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    },
                },
            },
        )
        state = {
            "forward_m": 0.8,
            "right_m": 0.0,
            "up_m": 0.2,
            "yaw_error_deg": 0.0,
            "status": "tracking",
            "timestamp": 10.0,
            "has_recent_valid_observation": True,
            "latest_pose_age_s": 0.1,
            "pre_dock_valid_frame_count": 3,
        }

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_TRACK
            task._track(state)
            captured = task.capture_tracking_ch3(source="unit_test")["captured_hold_ch3_pwm"]
            mode_result = task.set_tracking_vertical_mode("hold_captured_ch3")
            task._track(dict(state, up_m=-0.2, timestamp=10.1))

        self.assertTrue(mode_result["accepted"])
        self.assertEqual(task.last_command["up_m_s"], 0.0)
        self.assertEqual(task.last_command["vz"], 0.0)
        self.assertEqual(task.last_rc_override["ch3"], captured)
        self.assertEqual(task.get_status()["tracking_vertical_mode"], "hold_captured_ch3")

    def test_tracking_capture_ch3_rejects_without_recent_final_rc(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            tracking_config={
                "enable_motion": False,
                "output_backend": "rc_override",
                "rc_override": {
                    "enabled": True,
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    },
                },
            },
        )

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_TRACK
            task.filtered_state = {"has_recent_valid_observation": True, "latest_pose_age_s": 0.1}
            task.last_rc_override = {}
            result = task.capture_tracking_ch3(source="unit_test")

        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"], "final_rc_unavailable")

    def test_pre_align_lock_horizontal_keeps_vertical_only(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.5,
                "pre_align_axis_mode": "lock_horizontal",
                "camera_to_body": {
                    "forward_axis": "y",
                    "right_axis": "x",
                    "up_axis": "z",
                    "up_sign": -1.0,
                },
            },
        )
        state = {
            "forward_m": 0.0,
            "right_m": 0.0,
            "up_m": -0.8,
            "yaw_error_deg": 0.0,
            "status": "tracking",
            "timestamp": 10.0,
            "has_recent_valid_observation": True,
            "latest_pose_age_s": 0.1,
            "pre_dock_valid_frame_count": 3,
        }

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_PRE_ALIGN
            task._track(state)

        self.assertEqual(task.last_command["forward_m_s"], 0.0)
        self.assertEqual(task.last_command["right_m_s"], 0.0)
        self.assertEqual(task.last_command["yaw_rate_rad_s"], 0.0)
        self.assertNotEqual(task.last_command["up_m_s"], 0.0)

    def test_docking_pre_align_stays_active_while_distance_is_not_ready(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="docking",
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.5,
                "min_pre_dock_valid_frames": 1,
                "pre_dock_position_tolerance_m": 0.05,
                "pre_dock_distance_tolerance_m": 0.05,
                "pre_dock_yaw_tolerance_deg": 5.0,
                "pre_align_axis_mode": "small_correction",
                "pre_align_max_v_m_s": 0.05,
                "camera_to_body": {
                    "forward_axis": "y",
                    "right_axis": "x",
                    "up_axis": "z",
                    "up_sign": -1.0,
                },
                "control_mode": "pid",
                "pid": {
                    "forward": {"kp": 1.0, "output_limit": 0.4},
                    "right": {"kp": 1.0, "output_limit": 0.4},
                    "up": {"kp": 1.0, "output_limit": 0.4},
                    "yaw": {"kp": 1.0, "output_limit_deg_s": 10.0},
                },
            },
        )
        state = {
            "forward_m": 0.0,
            "right_m": 0.0,
            "up_m": -0.8,
            "yaw_error_deg": 0.0,
            "status": "tracking",
            "timestamp": 10.0,
            "has_recent_valid_observation": True,
            "latest_pose_age_s": 0.1,
            "pre_dock_valid_frame_count": 1,
        }

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_PRE_ALIGN
            task._track(state)

        self.assertEqual(task.stage, module.DockingTask.STATE_PRE_ALIGN)
        self.assertFalse(task.pre_dock_ready)
        self.assertAlmostEqual(task.last_command["up_m_s"], -0.05)
        self.assertTrue(task.last_command["pre_align_vertical_allowed"])

    def test_docking_pre_align_gates_vertical_motion_until_centered(self):
        module = import_docking_with_stubs()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="docking",
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.5,
                "min_pre_dock_valid_frames": 1,
                "pre_dock_position_tolerance_m": 0.05,
                "pre_dock_distance_tolerance_m": 0.05,
                "pre_dock_yaw_tolerance_deg": 5.0,
                "pre_align_axis_mode": "small_correction",
                "pre_align_max_v_m_s": 0.05,
                "camera_to_body": {
                    "forward_axis": "y",
                    "right_axis": "x",
                    "up_axis": "z",
                    "up_sign": -1.0,
                },
                "control_mode": "pid",
                "pid": {
                    "forward": {"kp": 1.0, "output_limit": 0.4},
                    "right": {"kp": 1.0, "output_limit": 0.4},
                    "up": {"kp": 1.0, "output_limit": 0.4},
                    "yaw": {"kp": 1.0, "output_limit_deg_s": 10.0},
                },
            },
        )
        state = {
            "forward_m": 0.12,
            "right_m": 0.0,
            "up_m": -0.8,
            "yaw_error_deg": 0.0,
            "status": "tracking",
            "timestamp": 10.0,
            "has_recent_valid_observation": True,
            "latest_pose_age_s": 0.1,
            "pre_dock_valid_frame_count": 1,
        }

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_PRE_ALIGN
            task._track(state)

        self.assertEqual(task.stage, module.DockingTask.STATE_PRE_ALIGN)
        self.assertFalse(task.pre_dock_ready)
        self.assertNotEqual(task.last_command["forward_m_s"], 0.0)
        self.assertEqual(task.last_command["up_m_s"], 0.0)
        self.assertFalse(task.last_command["pre_align_vertical_allowed"])
        self.assertEqual(task.last_command["pre_align_vertical_gated_reason"], "position_not_centered")

    def test_docking_vertical_ch3_can_exceed_global_mapper_limit_within_dedicated_limit(self):
        module = import_docking_with_stubs()
        task = self._make_vertical_docking_task(module)
        state = self._vertical_state(up_m=-0.8, vz=0.0, timestamp=10.1)

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_PRE_ALIGN
            task.docking_vertical_ctrl.reset(timestamp=10.0, initial_pwm=1600)
            task._track(state)

        self.assertEqual(task.last_command["pre_align_target_approach_speed_m_s"], 0.03)
        self.assertEqual(task.last_rc_override["ch3"], 1610)
        self.assertGreater(task.last_rc_override["ch3"], task.rc_mapper.max_pwm)
        self.assertLessEqual(task.last_rc_override["ch3"], 1700)

    def test_pre_align_not_centered_uses_zero_target_speed_and_buoyancy_hold(self):
        module = import_docking_with_stubs()
        task = self._make_vertical_docking_task(module)
        state = self._vertical_state(forward_m=0.2, up_m=-0.8, vz=0.0, timestamp=10.1)

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_PRE_ALIGN
            task.docking_vertical_ctrl.reset(timestamp=10.0, initial_pwm=1600)
            task._track(state)

        self.assertEqual(task.last_command["pre_align_target_approach_speed_m_s"], 0.0)
        self.assertEqual(task.last_rc_override["ch3"], 1600)
        self.assertTrue(task.last_command["pre_align_buoyancy_hold_active"])

    def test_high_approach_speed_blocks_pre_dock_ready(self):
        module = import_docking_with_stubs()
        task = self._make_vertical_docking_task(module)
        state = self._vertical_state(up_m=-0.5, vz=-0.05, timestamp=10.0)

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_PRE_ALIGN
            task._track(state)

        self.assertTrue(task.tracking_ready)
        self.assertFalse(task.pre_dock_ready)
        self.assertEqual(task.filtered_state["pre_dock_block_reason"], "approach_speed_high")
        self.assertFalse(task.filtered_state["pre_dock_approach_speed_ok"])

    def test_non_finite_approach_speed_forces_neutral_docking_ch3(self):
        module = import_docking_with_stubs()
        task = self._make_vertical_docking_task(module)
        state = self._vertical_state(up_m=-0.5, vz=float("nan"), timestamp=10.0)

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_PRE_ALIGN
            task._track(state)

        self.assertEqual(task.last_rc_override["ch3"], 1500)
        self.assertFalse(task.pre_dock_ready)
        self.assertEqual(task.filtered_state["pre_dock_block_reason"], "approach_speed_invalid")
        self.assertFalse(task.filtered_state["pre_align_input_valid"])

    def test_docked_hold_ignores_marker_loss_and_docking_timeout(self):
        module = import_docking_with_stubs()
        task = self._make_vertical_docking_task(
            module,
            camera=FakeCamera([], detected=False),
            enable_motion=True,
            pre_align_buoyancy_hold_pwm=1620,
        )

        with patch("builtins.print"):
            task.start()
            task.stage = module.DockingTask.STATE_PRE_ALIGN
            task.pre_dock_ready = True
            task.confirm_manual_dock(source="unit_test")
            task.start_time = 0.0
            with patch.object(module.time, "time", return_value=100.0):
                task.run()

        status = task.get_status()
        self.assertEqual(status["stage"], module.DockingTask.STATE_DOCKED_HOLD)
        self.assertEqual(status["status"], "running")
        self.assertEqual(status["mission_mode"], "docking")
        self.assertEqual(status["rc_override"]["ch3"], 1620)
        self.assertEqual(status["pre_align_vertical_pwm"], 1620)
        self.assertTrue(status["docked_hold_active"])
        self.assertEqual(task.pixhawk.commands[-1]["rc"]["ch3"], 1620)

    def _make_vertical_docking_task(self, module, camera=None, pixhawk=None, **config_overrides):
        config = {
            "enable_motion": False,
            "output_backend": "rc_override",
            "desired_z_m": 0.5,
            "min_pre_dock_valid_frames": 1,
            "pre_align_buoyancy_hold_pwm": 1600,
            "pre_align_down_pwm_max": 1700,
            "pre_align_target_approach_speed_m_s": 0.03,
            "pre_align_approach_speed_kp": 2000,
            "pre_dock_approach_speed_tolerance_m_s": 0.01,
            "pre_align_max_v_m_s": 0.05,
            "camera_to_body": {
                "forward_axis": "y",
                "right_axis": "x",
                "up_axis": "z",
                "up_sign": -1.0,
            },
            "rc_override": {
                "enabled": True,
                "max_pwm": 1600,
                "channels": {"forward": "ch5", "right": "ch6", "up": "ch3", "yaw": "ch4"},
            },
        }
        config.update(config_overrides)
        return module.DockingTask(
            camera=camera or FakeCamera([]),
            pixhawk=pixhawk or FakePixhawk(),
            state_machine=FakeStateMachine(),
            mission_mode="docking",
            tracking_config=config,
        )

    @staticmethod
    def _vertical_state(forward_m=0.0, right_m=0.0, up_m=-0.5, vz=0.0, timestamp=10.0):
        return {
            "forward_m": forward_m,
            "right_m": right_m,
            "up_m": up_m,
            "yaw_error_deg": 0.0,
            "vz": vz,
            "status": "tracking",
            "timestamp": timestamp,
            "has_recent_valid_observation": True,
            "latest_pose_age_s": 0.1,
            "pre_dock_valid_frame_count": 1,
        }

    def test_stop_sends_neutral_rc_without_closing_pixhawk_connection(self):
        module = import_docking_with_stubs()
        pixhawk = FakePixhawk()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            tracking_config={
                "enable_motion": True,
                "output_backend": "rc_override",
                "rc_override": {
                    "enabled": True,
                    "channels": {
                        "forward": "ch5",
                        "right": "ch6",
                        "up": "ch3",
                        "yaw": "ch4",
                    },
                },
            },
        )

        with patch("builtins.print"):
            task.start()
            task.stop()

        self.assertEqual(task.status, "stopped")
        self.assertFalse(pixhawk.stopped)
        self.assertEqual(pixhawk.commands[-1], {"rc": {f"ch{i}": 1500 for i in range(1, 9)}})

    def test_stop_waits_for_inflight_docked_hold_and_leaves_neutral_as_final_output(self):
        module = import_docking_with_stubs()

        class BlockingPixhawk(FakePixhawk):
            def __init__(self):
                super().__init__()
                self.hold_send_started = threading.Event()
                self.release_hold_send = threading.Event()
                self.block_once = True

            def send_rc_override(self, channels):
                if channels.get("ch3") == 1600 and self.block_once:
                    self.block_once = False
                    self.hold_send_started.set()
                    self.release_hold_send.wait(timeout=2.0)
                super().send_rc_override(channels)

        pixhawk = BlockingPixhawk()
        task = self._make_vertical_docking_task(module, pixhawk=pixhawk, enable_motion=True)
        task.status = "running"
        task.stage = module.DockingTask.STATE_DOCKED_HOLD
        hold_thread = threading.Thread(target=task._hold_after_confirm)
        stop_thread = threading.Thread(target=task.stop)

        with patch("builtins.print"):
            hold_thread.start()
            self.assertTrue(pixhawk.hold_send_started.wait(timeout=1.0))
            stop_thread.start()
            pixhawk.release_hold_send.set()
            hold_thread.join(timeout=2.0)
            stop_thread.join(timeout=2.0)

        self.assertFalse(hold_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertEqual(task.status, "stopped")
        self.assertEqual(pixhawk.commands[-1], {"rc": {f"ch{i}": 1500 for i in range(1, 9)}})


if __name__ == "__main__":
    unittest.main()
