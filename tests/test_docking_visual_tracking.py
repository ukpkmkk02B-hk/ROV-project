import importlib
import sys
import types
import unittest
from unittest.mock import patch


def import_docking_with_stubs():
    sys.modules.setdefault("yaml", types.SimpleNamespace(safe_load=lambda f: {}))
    sys.modules.setdefault("pupil_apriltags", types.SimpleNamespace(Detector=lambda **kwargs: object()))
    cv2_stub = types.SimpleNamespace()
    sys.modules.setdefault("cv2", cv2_stub)

    pymavlink = types.ModuleType("pymavlink")
    mavutil = types.SimpleNamespace(mavlink=types.SimpleNamespace())
    pymavlink.mavutil = mavutil
    sys.modules["pymavlink"] = pymavlink
    sys.modules["pymavlink.mavutil"] = mavutil

    module_name = "modules.tasks.docking_task"
    sys.modules.pop(module_name, None)
    return importlib.import_module(module_name)


class FakeCamera:
    def __init__(self, poses):
        self.poses = list(poses)
        self.detected = True

    def get_pose(self):
        if self.poses:
            return self.poses.pop(0)
        return None

    def target_detected(self):
        return self.detected

    def target_lost(self):
        return not self.detected


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

    def test_predicted_state_does_not_trigger_pre_dock_ready(self):
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
        camera = FakeCamera([pose, None])
        camera.detected = True
        task = module.DockingTask(
            camera=camera,
            pixhawk=FakePixhawk(),
            state_machine=FakeStateMachine(),
            tracking_config={
                "enable_motion": False,
                "desired_z_m": 0.8,
                "max_lost_frames": 3,
                "min_pre_dock_valid_frames": 1,
            },
        )

        with patch("builtins.print"):
            task.start()
            task.run()
            task.run()

        self.assertEqual(task.filtered_state["status"], "predicted")
        self.assertFalse(task.get_status()["pre_dock_ready"])

    def test_motion_enabled_uses_configured_required_mode(self):
        module = import_docking_with_stubs()
        pixhawk = FakePixhawk()
        task = module.DockingTask(
            camera=FakeCamera([]),
            pixhawk=pixhawk,
            state_machine=FakeStateMachine(),
            tracking_config={"enable_motion": True, "required_mode": "GUIDED"},
        )

        with patch("builtins.print"):
            task.start()

        self.assertEqual(pixhawk.mode_calls, ["GUIDED"])

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
        self.assertEqual(status["filtered_state"]["status"], "lost")


if __name__ == "__main__":
    unittest.main()
