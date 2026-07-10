VISUAL_TASK_NAMES = {"tracking", "docking"}


def current_visual_task(scheduler):
    current_task = getattr(scheduler, "current_task", None)
    if not current_task or current_task.get("name") not in VISUAL_TASK_NAMES:
        return None
    return current_task.get("instance")


def current_tracking_task(scheduler):
    current_task = getattr(scheduler, "current_task", None)
    if not current_task or current_task.get("name") != "tracking":
        return None
    task = current_task.get("instance")
    if getattr(task, "mission_mode", "tracking") != "tracking":
        return None
    return task


def current_docking_task(scheduler):
    current_task = getattr(scheduler, "current_task", None)
    if not current_task or current_task.get("name") != "docking":
        return None
    task = current_task.get("instance")
    if getattr(task, "mission_mode", "docking") != "docking":
        return None
    return task


def handle_docking_runtime_command(scheduler, rov_cmd, source="surface", pixhawk=None, rc_state=None):
    command = str(rov_cmd or "").strip()

    if command == "tracking start":
        if current_visual_task(scheduler) is not None:
            return _rejected("tracking_start", "visual_task_already_running")
        if _has_pending_visual_task(scheduler):
            return _rejected("tracking_start", "visual_task_already_pending")
        neutral_result = _send_neutral_if_available(pixhawk, rc_state)
        accepted = bool(scheduler.start_task("tracking"))
        return {
            "handled": True,
            "tracking_start": {
                "accepted": accepted,
                "reason": "" if accepted else "start_task_failed",
                **neutral_result,
            },
        }

    if command == "tracking stop":
        task = current_tracking_task(scheduler)
        if task is None:
            return _rejected("tracking_stop", "tracking_task_not_running")
        _clear_pending_visual_tasks(scheduler)
        neutral_result = _send_neutral_if_available(pixhawk, rc_state)
        stopped = bool(scheduler.stop_current_task())
        _clear_pending_visual_tasks(scheduler)
        return {
            "handled": True,
            "tracking_stop": {
                "accepted": stopped,
                "reason": "" if stopped else "stop_current_task_failed",
                **neutral_result,
            },
        }

    if command == "reset":
        _clear_pending_visual_tasks(scheduler)
        neutral_result = _send_neutral_if_available(pixhawk, rc_state)
        reset = bool(scheduler.reset_error_state())
        return {
            "handled": True,
            "reset": {
                "accepted": reset,
                "reason": "" if reset else "not_in_error_state",
                **neutral_result,
            },
        }

    if command == "docking start":
        task = current_tracking_task(scheduler)
        if task is None:
            return _rejected("docking_start", "tracking_task_not_running")
        if not getattr(task, "filtered_state", None) or not task.filtered_state.get("has_recent_valid_observation"):
            return _rejected("docking_start", "recent_observation_expired")
        result = task.engage_docking(source=source)
        if result.get("accepted"):
            promoted = bool(scheduler.promote_current_task("docking"))
            result["promoted"] = promoted
            if not promoted:
                result["accepted"] = False
                result["reason"] = "promote_current_task_failed"
        return {"handled": True, "docking_start": result}

    if command.startswith("tracking mode "):
        mode = command.split("tracking mode ", 1)[1].strip()
        task = current_visual_task(scheduler)
        if task is None:
            return _rejected("tracking_mode", "visual_task_not_running")
        return {"handled": True, "tracking_mode": task.set_tracking_vertical_mode(mode)}

    if command == "tracking capture ch3":
        task = current_visual_task(scheduler)
        if task is None:
            return _rejected("tracking_capture_ch3", "visual_task_not_running")
        return {"handled": True, "tracking_capture_ch3": task.capture_tracking_ch3(source=source)}

    if command.startswith("prealign mode "):
        mode = command.split("prealign mode ", 1)[1].strip()
        task = current_docking_task(scheduler)
        if task is None:
            return _rejected("prealign_mode", "docking_task_not_running")
        return {"handled": True, "prealign_mode": task.set_pre_align_axis_mode(mode)}

    return {"handled": False}


def should_send_manual_rc_state(scheduler):
    if _has_pending_visual_task(scheduler):
        return False
    task = current_visual_task(scheduler)
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
    task = current_visual_task(scheduler)
    if task is None:
        if _has_pending_visual_task(scheduler):
            _clear_pending_visual_tasks(scheduler)
            neutral_result = _send_neutral_if_available(pixhawk, rc_state, neutral_pwm)
            return {
                "accepted": True,
                "reason": "pending_visual_task_cleared",
                **neutral_result,
            }
        return _rejected("visual_stop", "visual_task_not_running")["visual_stop"]

    _clear_pending_visual_tasks(scheduler)
    neutral = dict(neutralize_rc_state(rc_state, neutral_pwm))
    neutral_sent = False
    neutral_error = ""
    try:
        pixhawk.send_rc_override(neutral)
        neutral_sent = True
    except Exception as exc:
        neutral_error = str(exc)

    stopped = bool(scheduler.stop_current_task())
    _clear_pending_visual_tasks(scheduler)
    return {
        "accepted": stopped,
        "reason": "" if stopped else "stop_current_task_failed",
        "neutral_sent": neutral_sent,
        "neutral_error": neutral_error,
        "rc_override": neutral,
    }


def stop_docked_hold_before_disarm(scheduler, pixhawk, rc_state, neutral_pwm=1500):
    """Neutralize and stop only an active docked-hold task before disarming."""
    task = current_docking_task(scheduler)
    if task is None or getattr(task, "stage", None) != "docked_hold":
        return None
    return stop_docking_safely(scheduler, pixhawk, rc_state, neutral_pwm)


def _send_neutral_if_available(pixhawk, rc_state, neutral_pwm=1500):
    if pixhawk is None or rc_state is None:
        return {"neutral_sent": False, "neutral_error": ""}
    neutral = dict(neutralize_rc_state(rc_state, neutral_pwm))
    try:
        pixhawk.send_rc_override(neutral)
        return {"neutral_sent": True, "neutral_error": "", "rc_override": neutral}
    except Exception as exc:
        return {"neutral_sent": False, "neutral_error": str(exc), "rc_override": neutral}


def _has_pending_visual_task(scheduler):
    if hasattr(scheduler, "has_pending_tasks"):
        return bool(scheduler.has_pending_tasks(VISUAL_TASK_NAMES))
    return any(
        bool(getattr(scheduler, "has_pending_task", lambda _name: False)(task_name))
        for task_name in VISUAL_TASK_NAMES
    )


def _clear_pending_visual_tasks(scheduler):
    clear = getattr(scheduler, "clear_pending_tasks", None)
    if clear is None:
        return 0
    return int(clear(VISUAL_TASK_NAMES) or 0)


def _rejected(key, reason):
    return {
        "handled": True,
        key: {
            "accepted": False,
            "reason": reason,
        },
    }
