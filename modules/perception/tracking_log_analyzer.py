import csv
from collections import Counter
from pathlib import Path


RANGE_FIELDS = [
    "pose_z",
    "pose_yaw",
    "filtered_z",
    "filtered_yaw",
    "cmd_vx",
    "cmd_vy",
    "cmd_vz",
    "cmd_yaw_rate",
]


def _as_float(value):
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_int(value):
    if value in (None, ""):
        return 0
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return 0


def _truthy(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def analyze_tracking_log(path):
    path = Path(path)
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    sample_count = len(rows)
    detected_count = sum(1 for row in rows if _truthy(row.get("detected")))
    pre_dock_ready_count = sum(1 for row in rows if _truthy(row.get("pre_dock_ready")))
    status_counts = Counter(row.get("tracking_status", "") or "unknown" for row in rows)
    max_lost_frames = max((_as_int(row.get("lost_frames")) for row in rows), default=0)

    ranges = {}
    for field in RANGE_FIELDS:
        values = [_as_float(row.get(field)) for row in rows]
        values = [value for value in values if value is not None]
        if values:
            ranges[field] = {"min": min(values), "max": max(values)}

    return {
        "path": str(path),
        "sample_count": sample_count,
        "detected_count": detected_count,
        "detected_rate": detected_count / sample_count if sample_count else 0.0,
        "pre_dock_ready_count": pre_dock_ready_count,
        "status_counts": dict(status_counts),
        "max_lost_frames": max_lost_frames,
        "ranges": ranges,
    }


def format_analysis_report(summary):
    lines = [
        f"log: {summary.get('path', '')}",
        f"samples: {summary['sample_count']}",
        f"detected: {summary['detected_count']} ({summary['detected_rate'] * 100:.1f}%)",
        f"pre_dock_ready: {summary['pre_dock_ready_count']}",
        f"max_lost_frames: {summary['max_lost_frames']}",
        "status_counts:",
    ]

    for status, count in sorted(summary.get("status_counts", {}).items()):
        lines.append(f"  {status}: {count}")

    if summary.get("ranges"):
        lines.append("ranges:")
        for field, values in sorted(summary["ranges"].items()):
            lines.append(f"  {field}: {values['min']:.3f} .. {values['max']:.3f}")

    return "\n".join(lines)
