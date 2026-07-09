MANUAL_ROV_RC_COMMANDS = {
    "forward": {"ch3": 1500, "ch4": 1500, "ch5": 1400, "ch6": 1500},
    "backward": {"ch3": 1500, "ch4": 1500, "ch5": 1600, "ch6": 1500},
    "left": {"ch3": 1500, "ch4": 1500, "ch5": 1500, "ch6": 1600},
    "right": {"ch3": 1500, "ch4": 1500, "ch5": 1500, "ch6": 1400},
    "turn_left": {"ch3": 1500, "ch4": 1600, "ch5": 1500, "ch6": 1500},
    "turn_right": {"ch3": 1500, "ch4": 1400, "ch5": 1500, "ch6": 1500},
    "up": {"ch3": 1400, "ch4": 1500, "ch5": 1500, "ch6": 1500},
    "down": {"ch3": 1700, "ch4": 1500, "ch5": 1500, "ch6": 1500},
}


def apply_manual_rov_command(rc_state, command):
    """Apply a manual ROV direction command to the shared RC override state."""
    updates = MANUAL_ROV_RC_COMMANDS.get(str(command or "").strip())
    if updates is None:
        return False
    rc_state.update(updates)
    return True
