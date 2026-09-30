"""Pre-publication tracking checks; no transport or recovery commands.

Tracking tolerance is an explicit protocol input, never inferred from the
requested amplitude. Passing this guard does not validate an emergency stop.
"""
import math

from .errors import SafetyInterlockError
from .models import ARM_JOINT_NAMES
from .safety import validate_arm_target


def check_tracking(target, arm, *, now_ns, max_error_rad, max_age_ns=100_000_000):
    if not math.isfinite(max_error_rad) or max_error_rad <= 0:
        raise ValueError("Explicit finite positive tracking error bound required")
    if type(max_age_ns) is not int or max_age_ns <= 0:
        raise ValueError("Positive integer feedback age bound required")
    target = validate_arm_target(target)
    if tuple(j.name for j in arm.joints) != ARM_JOINT_NAMES:
        raise SafetyInterlockError("Incomplete or reordered arm feedback")
    if arm.domain_state != 0:
        raise SafetyInterlockError("Arm domain is not explicitly ready")
    if not 0 <= now_ns - arm.timestamp.monotonic_ns <= max_age_ns:
        raise SafetyInterlockError("Stale or future arm feedback")
    for desired, joint in zip(target, arm.joints, strict=True):
        if joint.fault_code != 0 or not math.isfinite(joint.position_rad):
            raise SafetyInterlockError("Invalid, unknown or faulted arm feedback")
        if abs(desired - joint.position_rad) > max_error_rad:
            raise SafetyInterlockError(f"Tracking error bound exceeded: {joint.name}")
