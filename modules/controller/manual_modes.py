MODE_COMMANDS = {"MANUAL", "STABILIZE", "ALT_HOLD"}
SUPPORTED_MODE = "MANUAL"


def normalize_supported_mode(mode_name):
    mode = str(mode_name or "").strip().upper()
    if mode != SUPPORTED_MODE:
        raise ValueError("only MANUAL mode is supported")
    return mode


def mode_command_result(command):
    mode = str(command or "").strip().upper()
    if mode not in MODE_COMMANDS:
        return {"is_mode_command": False, "accepted": False, "mode": str(command or "").strip(), "reason": ""}
    if mode != SUPPORTED_MODE:
        return {"is_mode_command": True, "accepted": False, "mode": mode, "reason": "manual_mode_only"}
    return {"is_mode_command": True, "accepted": True, "mode": mode, "reason": ""}
