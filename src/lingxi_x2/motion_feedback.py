"""Describe final encoder error without equating publication with arrival."""
from __future__ import annotations

import math

from .models import ARM_JOINT_NAMES, ArmState
from .safety import validate_arm_target


def arm_tracking_report(target, arm: ArmState, *, now_ns: int,
                        not_before_ns: int, max_age_ns: int) -> dict:
    """A receive-clock snapshot, not settled-window or physical latency evidence.

    Do not report numbers from an incomplete, faulted or old snapshot as a
    successful movement. No tolerance, compensation or controller gain is inferred.
    """
    target = validate_arm_target(target)
    if (type(now_ns) is not int or type(not_before_ns) is not int
            or type(max_age_ns) is not int or max_age_ns <= 0
            or not 0 <= not_before_ns <= now_ns):
        raise ValueError("Invalid feedback observation clocks or age bound")
    report = {"status": "unavailable", "motion_verified": False,
              "joint_names": list(ARM_JOINT_NAMES), "desired_rad": list(target),
              "error_convention": "encoder_minus_desired",
              "evidence": "single encoder snapshot; not settled arrival or hardware acceptance",
              "source": arm.source, "received_monotonic_ns": arm.timestamp.monotonic_ns,
              "source_stamp_ns": arm.timestamp.sec * 1_000_000_000 + arm.timestamp.nanosec}
    if tuple(j.name for j in arm.joints) != ARM_JOINT_NAMES:
        report["reason"] = "incomplete_or_reordered_feedback"
    elif arm.domain_state != 0 or any(j.fault_code != 0 for j in arm.joints):
        report["reason"] = "faulted_or_unknown_feedback_state"
    elif any(not math.isfinite(j.position_rad) for j in arm.joints):
        report["reason"] = "nonfinite_feedback"
    elif not 0 <= now_ns - arm.timestamp.monotonic_ns <= max_age_ns:
        report["reason"] = "stale_or_future_feedback"
    elif arm.timestamp.monotonic_ns <= not_before_ns:
        report["reason"] = "no_feedback_received_after_final_publish"
    else:
        measured = [j.position_rad for j in arm.joints]
        error = [q - d for q, d in zip(measured, target, strict=True)]
        report.update(status="observed", encoder_rad=measured,
                      error_rad=error, max_abs_error_rad=max(map(abs, error)))
    return report
