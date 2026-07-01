import unittest

from modules.tasks.docking_runtime import (
    handle_docking_runtime_command,
    should_send_manual_rc_state,
    stop_docking_safely,
)


class FakeDockingTask:
    def __init__(self, enable_motion=False, status="running"):
        self.enable_motion = enable_motion
        self.status = status
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


class FakeScheduler:
    def __init__(self, current_task=None):
        self.current_task = current_task
        self.stop_current_task_calls = 0

    def stop_current_task(self):
        self.stop_current_task_calls += 1
        if self.current_task and hasattr(self.current_task.get("instance"), "stop"):
            self.current_task["instance"].stop()
        self.current_task = None
        return True


class FakePixhawk:
    def __init__(self):
        self.rc_commands = []

    def send_rc_override(self, channels):
        self.rc_commands.append(dict(channels))


def scheduler_with_docking(task):
    return FakeScheduler({"name": "docking", "instance": task})


class DockingRuntimeCommandTests(unittest.TestCase):
    def test_rejects_tracking_capture_when_no_docking_task_is_running(self):
        result = handle_docking_runtime_command(FakeScheduler(), "tracking capture ch3")

        self.assertTrue(result["handled"])
        self.assertFalse(result["tracking_capture_ch3"]["accepted"])
        self.assertEqual(result["tracking_capture_ch3"]["reason"], "docking_task_not_running")

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

    def test_unrelated_command_is_not_handled(self):
        result = handle_docking_runtime_command(scheduler_with_docking(FakeDockingTask()), "forward")

        self.assertFalse(result["handled"])

    def test_manual_rc_output_is_blocked_only_by_running_motion_enabled_docking(self):
        self.assertFalse(should_send_manual_rc_state(scheduler_with_docking(FakeDockingTask(enable_motion=True))))
        self.assertTrue(should_send_manual_rc_state(scheduler_with_docking(FakeDockingTask(enable_motion=False))))
        self.assertTrue(
            should_send_manual_rc_state(scheduler_with_docking(FakeDockingTask(enable_motion=True, status="stopped")))
        )
        self.assertTrue(should_send_manual_rc_state(FakeScheduler({"name": "charging", "instance": object()})))

    def test_stop_docking_safely_sends_neutral_before_stopping_task(self):
        task = FakeDockingTask(enable_motion=True)
        task.stop = lambda: task.calls.append(("stop", "called"))
        scheduler = scheduler_with_docking(task)
        pixhawk = FakePixhawk()
        rc_state = {"ch3": 1700, "ch4": 1520, "ch5": 1600, "ch6": 1400}

        result = stop_docking_safely(scheduler, pixhawk, rc_state)

        self.assertTrue(result["accepted"])
        self.assertEqual(scheduler.stop_current_task_calls, 1)
        self.assertEqual(task.calls[-1], ("stop", "called"))
        self.assertEqual(rc_state, {f"ch{i}": 1500 for i in range(1, 9)})
        self.assertEqual(pixhawk.rc_commands[0], {f"ch{i}": 1500 for i in range(1, 9)})

    def test_stop_docking_safely_rejects_when_no_docking_task_is_running(self):
        pixhawk = FakePixhawk()
        rc_state = {"ch3": 1700}

        result = stop_docking_safely(FakeScheduler(), pixhawk, rc_state)

        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"], "docking_task_not_running")
        self.assertEqual(pixhawk.rc_commands, [])


if __name__ == "__main__":
    unittest.main()
