def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def rc_command(neutral: int, delta: float, max_delta: int) -> int:
    delta = clamp(delta, -max_delta, max_delta)
    return int(round(neutral + delta))


def neutral_commands(neutral: int, channels: dict) -> dict:
    return {name: int(neutral) for name in channels}
