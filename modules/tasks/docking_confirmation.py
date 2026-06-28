def confirm_current_docking_task(scheduler, source="surface"):
    """Confirm the active docking task after visual pre-alignment."""
    current_task = getattr(scheduler, "current_task", None)
    if not current_task or current_task.get("name") != "docking":
        return {
            "accepted": False,
            "reason": "no_active_docking_task",
            "source": source,
        }

    task = current_task.get("instance")
    if not hasattr(task, "confirm_manual_dock"):
        return {
            "accepted": False,
            "reason": "active_task_cannot_confirm_dock",
            "source": source,
        }

    return task.confirm_manual_dock(source=source)
