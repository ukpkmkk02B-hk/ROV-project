import csv
from collections import Counter
from pathlib import Path


VALID_POSE_RANGE_FIELDS = [
    "pose_z",
    "pose_yaw",
    "filtered_z",
    "filtered_yaw",
    "body_forward_m",
    "body_right_m",
    "body_up_m",
    "yaw_raw_deg",
    "yaw_error_deg",
    "marker_pixel_size_px",
    "reprojection_error_px",
]

ALL_SAMPLE_RANGE_FIELDS = [
    "cmd_vx",
    "cmd_vy",
    "cmd_vz",
    "cmd_yaw_rate",
    "motion_forward_m_s",
    "motion_right_m_s",
    "motion_up_m_s",
    "motion_yaw_rate_rad_s",
    "mavlink_vx",
    "mavlink_vy",
    "mavlink_vz",
    "mavlink_yaw_rate",
    "rc_ch1",
    "rc_ch2",
    "rc_ch3",
    "rc_ch4",
    "rc_ch5",
    "rc_ch6",
    "rc_ch7",
    "rc_ch8",
]

TRACKER_COUNT_FIELDS = [
    "tracker_frames_processed",
    "tracker_marker_frames",
    "tracker_target_frames",
    "tracker_valid_pose_frames",
    "tracker_invalid_pose_frames",
    "tracker_no_marker_frames",
    "tracker_target_id_missing_frames",
    "tracker_pnp_failed_frames",
    "tracker_quality_rejected_frames",
    "tracker_capture_failed_frames",
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


def _field_range(rows, field):
    values = [_as_float(row.get(field)) for row in rows]
    values = [value for value in values if value is not None]
    if not values:
        return None
    return {"min": min(values), "max": max(values)}


def _last_counter(rows, field):
    for row in reversed(rows):
        if row.get(field) not in (None, ""):
            return _as_int(row.get(field))
    return 0


def analyze_tracking_log(path):
    path = Path(path)
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    sample_count = len(rows)
    detected_count = sum(1 for row in rows if _truthy(row.get("detected")))
    valid_pose_count = sum(1 for row in rows if _truthy(row.get("detected")) and _truthy(row.get("pose_valid")))
    pre_dock_ready_count = sum(1 for row in rows if _truthy(row.get("pre_dock_ready")))
    pre_dock_block_reason_counts = Counter(
        row.get("pre_dock_block_reason", "") or "none"
        for row in rows
        if not _truthy(row.get("pre_dock_ready")) and (row.get("pre_dock_block_reason", "") or "")
    )
    status_counts = Counter(row.get("tracking_status", "") or "unknown" for row in rows)
    reject_reason_counts = Counter(
        row.get("reject_reason", "") or "none"
        for row in rows
        if not (_truthy(row.get("detected")) and _truthy(row.get("pose_valid")))
        and (row.get("reject_reason", "") or "")
    )
    max_lost_frames = max((_as_int(row.get("lost_frames")) for row in rows), default=0)

    ranges = {}
    valid_pose_rows = [row for row in rows if _truthy(row.get("detected")) and _truthy(row.get("pose_valid"))]
    for field in VALID_POSE_RANGE_FIELDS:
        field_range = _field_range(valid_pose_rows, field)
        if field_range:
            ranges[field] = field_range
    for field in ALL_SAMPLE_RANGE_FIELDS:
        field_range = _field_range(rows, field)
        if field_range:
            ranges[field] = field_range

    tracker_counts = {field: _last_counter(rows, field) for field in TRACKER_COUNT_FIELDS}
    tracker_frames = tracker_counts["tracker_frames_processed"]
    tracker_rates = {
        "tracker_marker_rate": tracker_counts["tracker_marker_frames"] / tracker_frames if tracker_frames else 0.0,
        "tracker_target_rate": tracker_counts["tracker_target_frames"] / tracker_frames if tracker_frames else 0.0,
        "tracker_valid_pose_rate": tracker_counts["tracker_valid_pose_frames"] / tracker_frames if tracker_frames else 0.0,
        "tracker_invalid_pose_rate": tracker_counts["tracker_invalid_pose_frames"] / tracker_frames if tracker_frames else 0.0,
        "tracker_no_marker_rate": tracker_counts["tracker_no_marker_frames"] / tracker_frames if tracker_frames else 0.0,
    }

    summary = {
        "path": str(path),
        "sample_count": sample_count,
        "detected_count": detected_count,
        "detected_rate": detected_count / sample_count if sample_count else 0.0,
        "valid_pose_count": valid_pose_count,
        "valid_pose_rate": valid_pose_count / sample_count if sample_count else 0.0,
        "pre_dock_ready_count": pre_dock_ready_count,
        "pre_dock_block_reason_counts": dict(pre_dock_block_reason_counts),
        "status_counts": dict(status_counts),
        "reject_reason_counts": dict(reject_reason_counts),
        "max_lost_frames": max_lost_frames,
        "ranges": ranges,
    }
    summary.update(tracker_counts)
    summary.update(tracker_rates)
    return summary


def format_analysis_report(summary):
    lines = [
        f"log: {summary.get('path', '')}",
        f"samples: {summary['sample_count']}",
        f"detected: {summary['detected_count']} ({summary['detected_rate'] * 100:.1f}%)",
        f"valid_pose: {summary.get('valid_pose_count', 0)} ({summary.get('valid_pose_rate', 0.0) * 100:.1f}%)",
        f"pre_dock_ready: {summary['pre_dock_ready_count']}",
        f"max_lost_frames: {summary['max_lost_frames']}",
        "status_counts:",
    ]

    for status, count in sorted(summary.get("status_counts", {}).items()):
        lines.append(f"  {status}: {count}")

    if summary.get("reject_reason_counts"):
        lines.append("reject_reasons:")
        for reason, count in sorted(summary.get("reject_reason_counts", {}).items()):
            lines.append(f"  {reason}: {count}")

    if summary.get("pre_dock_block_reason_counts"):
        lines.append("pre_dock_block_reasons:")
        for reason, count in sorted(summary.get("pre_dock_block_reason_counts", {}).items()):
            lines.append(f"  {reason}: {count}")

    if summary.get("tracker_frames_processed"):
        lines.append("tracker_frame_counts:")
        lines.append(f"  tracker_frames: {summary.get('tracker_frames_processed', 0)}")
        lines.append(
            "  tracker_marker_frames: "
            f"{summary.get('tracker_marker_frames', 0)} ({summary.get('tracker_marker_rate', 0.0) * 100:.1f}%)"
        )
        lines.append(
            "  tracker_target_frames: "
            f"{summary.get('tracker_target_frames', 0)} ({summary.get('tracker_target_rate', 0.0) * 100:.1f}%)"
        )
        lines.append(
            "  tracker_valid_pose_frames: "
            f"{summary.get('tracker_valid_pose_frames', 0)} ({summary.get('tracker_valid_pose_rate', 0.0) * 100:.1f}%)"
        )
        lines.append(
            "  tracker_invalid_pose_frames: "
            f"{summary.get('tracker_invalid_pose_frames', 0)} ({summary.get('tracker_invalid_pose_rate', 0.0) * 100:.1f}%)"
        )
        lines.append(
            "  tracker_no_marker_frames: "
            f"{summary.get('tracker_no_marker_frames', 0)} ({summary.get('tracker_no_marker_rate', 0.0) * 100:.1f}%)"
        )
        if summary.get("tracker_target_id_missing_frames"):
            lines.append(f"  tracker_target_id_missing_frames: {summary['tracker_target_id_missing_frames']}")
        if summary.get("tracker_pnp_failed_frames"):
            lines.append(f"  tracker_pnp_failed_frames: {summary['tracker_pnp_failed_frames']}")
        if summary.get("tracker_quality_rejected_frames"):
            lines.append(f"  tracker_quality_rejected_frames: {summary['tracker_quality_rejected_frames']}")
        if summary.get("tracker_capture_failed_frames"):
            lines.append(f"  tracker_capture_failed_frames: {summary['tracker_capture_failed_frames']}")

    if summary.get("ranges"):
        lines.append("ranges:")
        for field, values in sorted(summary["ranges"].items()):
            lines.append(f"  {field}: {values['min']:.3f} .. {values['max']:.3f}")

    return "\n".join(lines)
