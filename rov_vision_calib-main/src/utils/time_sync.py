from datetime import datetime


def now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def timestamp_for_path() -> str:
    return datetime.now().strftime("%Y_%m_%d_%H%M%S")
