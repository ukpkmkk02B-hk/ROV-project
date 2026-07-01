def current_docking_task(scheduler):
    current_task = getattr(scheduler, "current_task", None)
    if not current_task or current_task.get("name") != "docking":
        return None
    return current_task.get("instance")


def handle_docking_runtime_command(scheduler, rov_cmd, source="surface"):
    command = str(rov_cmd or "").strip()
    if command.startswith("tracking mode "):
        mode = command.split("tracking mode ", 1)[1].strip()
        task = current_docking_task(scheduler)
        if task is None:
            return _rejected("tracking_mode", "docking_task_not_running")
        return {"handled": True, "tracking_mode": task.set_tracking_vertical_mode(mode)}

    if command == "tracking capture ch3":
        task = current_docking_task(scheduler)
        if task is None:
            return _rejected("tracking_capture_ch3", "docking_task_not_running")
        return {"handled": True, "tracking_capture_ch3": task.capture_tracking_ch3(source=source)}

    if command.startswith("prealign mode "):
        mode = command.split("prealign mode ", 1)[1].strip()
        task = current_docking_task(scheduler)
        if task is None:
            return _rejected("prealign_mode", "docking_task_not_running")
        return {"handled": True, "prealign_mode": task.set_pre_align_axis_mode(mode)}

    return {"handled": False}


def should_send_manual_rc_state(scheduler):
    task = current_docking_task(scheduler)
    if task is None:
        return True
    if getattr(task, "status", None) != "running":
        return True
    return not bool(getattr(task, "enable_motion", False))


def neutralize_rc_state(rc_state, neutral_pwm=1500):
    for channel in [f"ch{i}" for i in range(1, 9)]:
        rc_state[channel] = int(neutral_pwm)
    return rc_state


def stop_docking_safely(scheduler, pixhawk, rc_state, neutral_pwm=1500):
    task = current_docking_task(scheduler)
    if task is None:
        return _rejected("docking_stop", "docking_task_not_running")["docking_stop"]

    neutral = dict(neutralize_rc_state(rc_state, neutral_pwm))
    neutral_sent = False
    neutral_error = ""
    try:
        pixhawk.send_rc_override(neutral)
        neutral_sent = True
    except Exception as exc:
        neutral_error = str(exc)

    stopped = bool(scheduler.stop_current_task())
    return {
        "accepted": stopped,
        "reason": "" if stopped else "stop_current_task_failed",
        "neutral_sent": neutral_sent,
        "neutral_error": neutral_error,
        "rc_override": neutral,
    }


def _rejected(key, reason):
    return {
        "handled": True,
        key: {
            "accepted": False,
            "reason": reason,
        },
    }
