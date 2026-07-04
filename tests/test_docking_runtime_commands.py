import unittest

from modules.tasks.docking_runtime import (
    handle_docking_runtime_command,
    should_send_manual_rc_state,
    stop_docking_safely,
)


class FakeDockingTask:
    def __init__(self, enable_motion=False, status="running", mission_mode="docking", recent_observation=True):
        self.enable_motion = enable_motion
        self.status = status
        self.mission_mode = mission_mode
        self.filtered_state = {"has_recent_valid_observation": recent_observation}
        self.calls = []

    def set_tracking_vertical_mode(self, mode):
        self.calls.append(("tracking_mode", mode))
        return {"accepted": True, "mode": mode}

    def capture_tracking_ch3(self, source="surface"):
        self.calls.append(("capture_ch3", source))
        return {"accepted": True, "captured_hold_ch3_pwm": 1520}

    def set_pre_align_axis_mode(self, mode):
        self.calls.append(("prealign_mode", mode))
        return {"accepted": True, "mode": mode}

    def engage_docking(self, source="surface"):
        self.calls.append(("engage_docking", source))
        self.mission_mode = "docking"
        return {"accepted": True, "mission_mode": "docking", "stage": "pre_align"}


class FakeScheduler:
    def __init__(self, current_task=None):
        self.current_task = current_task
        self.stop_current_task_calls = 0
        self.start_task_calls = []
        self.promoted_to = []
        self.pending_task_names = []
        self.clear_pending_calls = []
        self.reset_error_state_calls = 0

    def stop_current_task(self):
        self.stop_current_task_calls += 1
        if self.current_task and hasattr(self.current_task.get("instance"), "stop"):
            self.current_task["instance"].stop()
        self.current_task = None
        return True

    def start_task(self, name):
        self.start_task_calls.append(name)
        self.pending_task_names.append(name)
        return True

    def promote_current_task(self, name):
        self.promoted_to.append(name)
        if self.current_task:
            self.current_task["name"] = name
        return True

    def has_pending_task(self, name):
        return name in self.pending_task_names

    def clear_pending_tasks(self, names):
        names = set(names)
        self.clear_pending_calls.append(tuple(sorted(names)))
        removed = [name for name in self.pending_task_names if name in names]
        self.pending_task_names = [name for name in self.pending_task_names if name not in names]
        return len(removed)

    def reset_error_state(self):
        self.reset_error_state_calls += 1
        return True


class FakePixhawk:
    def __init__(self):
        self.rc_commands = []

    def send_rc_override(self, channels):
        self.rc_commands.append(dict(channels))


def scheduler_with_docking(task):
    return FakeScheduler({"name": "docking", "instance": task})


def scheduler_with_tracking(task):
    return FakeScheduler({"name": "tracking", "instance": task})


class DockingRuntimeCommandTests(unittest.TestCase):
    def test_rejects_tracking_capture_when_no_docking_task_is_running(self):
        result = handle_docking_runtime_command(FakeScheduler(), "tracking capture ch3")

        self.assertTrue(result["handled"])
        self.assertFalse(result["tracking_capture_ch3"]["accepted"])
        self.assertEqual(result["tracking_capture_ch3"]["reason"], "visual_task_not_running")

    def test_dispatches_tracking_mode_and_capture_to_current_docking_task(self):
        task = FakeDockingTask()
        scheduler = scheduler_with_docking(task)

        mode_result = handle_docking_runtime_command(scheduler, "tracking mode hold_captured_ch3")
        capture_result = handle_docking_runtime_command(scheduler, "tracking capture ch3")

        self.assertEqual(task.calls, [("tracking_mode", "hold_captured_ch3"), ("capture_ch3", "surface")])
        self.assertEqual(mode_result["tracking_mode"]["mode"], "hold_captured_ch3")
        self.assertEqual(capture_result["tracking_capture_ch3"]["captured_hold_ch3_pwm"], 1520)

    def test_dispatches_prealign_mode_to_current_docking_task(self):
        task = FakeDockingTask()

        result = handle_docking_runtime_command(scheduler_with_docking(task), "prealign mode small_correction")

        self.assertEqual(task.calls, [("prealign_mode", "small_correction")])
        self.assertEqual(result["prealign_mode"]["mode"], "small_correction")

    def test_tracking_start_queues_tracking_task(self):
        scheduler = FakeScheduler()

        result = handle_docking_runtime_command(scheduler, "tracking start")

        self.assertTrue(result["handled"])
        self.assertTrue(result["tracking_start"]["accepted"])
        self.assertEqual(scheduler.start_task_calls, ["tracking"])

    def test_tracking_start_rejects_when_visual_task_is_already_pending(self):
        scheduler = FakeScheduler()

        first = handle_docking_runtime_command(scheduler, "tracking start")
        second = handle_docking_runtime_command(scheduler, "tracking start")

        self.assertTrue(first["tracking_start"]["accepted"])
        self.assertFalse(second["tracking_start"]["accepted"])
        self.assertEqual(second["tracking_start"]["reason"], "visual_task_already_pending")
        self.assertEqual(scheduler.start_task_calls, ["tracking"])

    def test_docking_start_rejects_without_running_tracking_task(self):
        result = handle_docking_runtime_command(FakeScheduler(), "docking start")

        self.assertTrue(result["handled"])
        self.assertFalse(result["docking_start"]["accepted"])
        self.assertEqual(result["docking_start"]["reason"], "tracking_task_not_running")

    def test_docking_start_promotes_running_tracking_task(self):
        task = FakeDockingTask(mission_mode="tracking", recent_observation=True)
        scheduler = scheduler_with_tracking(task)

        result = handle_docking_runtime_command(scheduler, "docking start")

        self.assertTrue(result["handled"])
        self.assertTrue(result["docking_start"]["accepted"])
        self.assertEqual(task.calls, [("engage_docking", "surface")])
        self.assertEqual(scheduler.promoted_to, ["docking"])
        self.assertEqual(scheduler.current_task["name"], "docking")

    def test_docking_start_rejects_tracking_without_recent_observation(self):
        task = FakeDockingTask(mission_mode="tracking", recent_observation=False)

        result = handle_docking_runtime_command(scheduler_with_tracking(task), "docking start")

        self.assertTrue(result["handled"])
        self.assertFalse(result["docking_start"]["accepted"])
        self.assertEqual(result["docking_start"]["reason"], "recent_observation_expired")

    def test_prealign_mode_rejects_while_only_tracking(self):
        task = FakeDockingTask(mission_mode="tracking")

        result = handle_docking_runtime_command(scheduler_with_tracking(task), "prealign mode small_correction")

        self.assertTrue(result["handled"])
        self.assertFalse(result["prealign_mode"]["accepted"])
        self.assertEqual(result["prealign_mode"]["reason"], "docking_task_not_running")

    def test_unrelated_command_is_not_handled(self):
        result = handle_docking_runtime_command(scheduler_with_docking(FakeDockingTask()), "forward")

        self.assertFalse(result["handled"])

    def test_manual_rc_output_is_blocked_only_by_running_motion_enabled_docking(self):
        self.assertFalse(should_send_manual_rc_state(scheduler_with_docking(FakeDockingTask(enable_motion=True))))
        self.assertFalse(
            should_send_manual_rc_state(
                scheduler_with_tracking(FakeDockingTask(enable_motion=True, mission_mode="tracking"))
            )
        )
        self.assertTrue(should_send_manual_rc_state(scheduler_with_docking(FakeDockingTask(enable_motion=False))))
        self.assertTrue(
            should_send_manual_rc_state(scheduler_with_docking(FakeDockingTask(enable_motion=True, status="stopped")))
        )
        self.assertTrue(should_send_manual_rc_state(FakeScheduler({"name": "charging", "instance": object()})))

    def test_stop_docking_safely_sends_neutral_before_stopping_task(self):
        task = FakeDockingTask(enable_motion=True)
        task.stop = lambda: task.calls.append(("stop", "called"))
        scheduler = scheduler_with_docking(task)
        scheduler.pending_task_names = ["tracking", "docking"]
        pixhawk = FakePixhawk()
        rc_state = {"ch3": 1700, "ch4": 1520, "ch5": 1600, "ch6": 1400}

        result = stop_docking_safely(scheduler, pixhawk, rc_state)

        self.assertTrue(result["accepted"])
        self.assertEqual(scheduler.stop_current_task_calls, 1)
        self.assertEqual(task.calls[-1], ("stop", "called"))
        self.assertEqual(rc_state, {f"ch{i}": 1500 for i in range(1, 9)})
        self.assertEqual(pixhawk.rc_commands[0], {f"ch{i}": 1500 for i in range(1, 9)})
        self.assertEqual(scheduler.pending_task_names, [])
        self.assertEqual(scheduler.clear_pending_calls, [("docking", "tracking"), ("docking", "tracking")])

    def test_stop_docking_safely_rejects_when_no_docking_task_is_running(self):
        pixhawk = FakePixhawk()
        rc_state = {"ch3": 1700}

        result = stop_docking_safely(FakeScheduler(), pixhawk, rc_state)

        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"], "visual_task_not_running")
        self.assertEqual(pixhawk.rc_commands, [])

    def test_reset_command_resets_scheduler_and_neutralizes_rc(self):
        scheduler = FakeScheduler()
        pixhawk = FakePixhawk()
        rc_state = {"ch3": 1700, "ch4": 1520}

        result = handle_docking_runtime_command(scheduler, "reset", pixhawk=pixhawk, rc_state=rc_state)

        self.assertTrue(result["handled"])
        self.assertTrue(result["reset"]["accepted"])
        self.assertEqual(scheduler.reset_error_state_calls, 1)
        self.assertEqual(rc_state, {f"ch{i}": 1500 for i in range(1, 9)})


if __name__ == "__main__":
    unittest.main()
