import unittest

from modules.tasks.docking_confirmation import confirm_current_docking_task


class FakeDockingTask:
    def __init__(self, result):
        self.result = result
        self.calls = []

    def confirm_manual_dock(self, source="surface"):
        self.calls.append(source)
        return self.result


class FakeScheduler:
    def __init__(self, current_task=None):
        self.current_task = current_task


class DockingConfirmationTests(unittest.TestCase):
    def test_rejects_when_no_docking_task_is_running(self):
        result = confirm_current_docking_task(FakeScheduler())

        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"], "no_active_docking_task")

    def test_rejects_when_current_task_is_not_docking(self):
        scheduler = FakeScheduler({"name": "charging", "instance": object()})

        result = confirm_current_docking_task(scheduler)

        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"], "no_active_docking_task")

    def test_delegates_to_current_docking_task(self):
        task = FakeDockingTask({"accepted": True, "reason": ""})
        scheduler = FakeScheduler({"name": "docking", "instance": task})

        result = confirm_current_docking_task(scheduler, source="surface")

        self.assertTrue(result["accepted"])
        self.assertEqual(task.calls, ["surface"])


if __name__ == "__main__":
    unittest.main()
