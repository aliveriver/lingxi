from __future__ import annotations

import math
from collections.abc import Sequence

from .errors import SafetyInterlockError
from .models import ARM_JOINT_NAMES


_DEG = math.pi / 180.0

# Official AimDK 1.1.0 "joint_name_and_limit" guaranteed ranges for X2 Ultra.
_LEFT_ARM_LIMITS_DEG = (
    (-176.5, 116.5),
    (-3.5, 174.5),
    (-146.5, 146.5),
    (-135.0, 0.0),
    (-146.5, 146.5),
    (-30.0, 30.0),
    (-86.5, 41.5),
)
# Right roll axes are mirrored; repeating left limits admitted unsafe positive
# right shoulder roll. Intersect the guaranteed ranges with BOTH audited v1.3
# official and installed PC2 SDK bounds. This does not authorize physical motion.
_RIGHT_ARM_LIMITS_DEG = tuple(
    (-high, -low) if index in (1, 6) else (low, high)
    for index, (low, high) in enumerate(_LEFT_ARM_LIMITS_DEG)
)
_MODEL_INTERSECTION_RAD = (
    (-3.08, 2.04), (-0.061, 2.993), (-2.556, 2.556), (-2.3556, 0.),
    (-2.556, 2.556), (-0.5236, 0.5236), (-1.5097, 0.724),
    (-3.08, 2.04), (-2.993, 0.061), (-2.556, 2.556), (-2.3556, 0.),
    (-2.556, 2.556), (-0.5236, 0.5236), (-0.724, 1.5097),
)
ARM_LIMITS_RAD = tuple(
    (max(low * _DEG, model_low), min(high * _DEG, model_high))
    for (low, high), (model_low, model_high) in zip(
        _LEFT_ARM_LIMITS_DEG + _RIGHT_ARM_LIMITS_DEG, _MODEL_INTERSECTION_RAD, strict=True)
)


def validate_arm_target(values: Sequence[float]) -> tuple[float, ...]:
    if len(values) != len(ARM_JOINT_NAMES):
        raise SafetyInterlockError(f"Arm target must contain 14 values, got {len(values)}")
    result = tuple(float(value) for value in values)
    for name, value, (low, high) in zip(ARM_JOINT_NAMES, result, ARM_LIMITS_RAD, strict=True):
        if not math.isfinite(value):
            raise SafetyInterlockError(f"{name} is not finite")
        if not low <= value <= high:
            raise SafetyInterlockError(f"{name}={value:.4f} rad is outside [{low:.4f}, {high:.4f}]")
    return result


def validate_hand_target(values: Sequence[float]) -> tuple[float, ...]:
    if len(values) != 10:
        raise SafetyInterlockError(f"OmniHand target must contain 10 values, got {len(values)}")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise SafetyInterlockError("OmniHand target contains a non-finite value")
    return result


def validate_acceptance_delta(value: float) -> float:
    result = float(value)
    if not math.isfinite(result) or not 0.01 <= abs(result) <= 1.0:
        raise SafetyInterlockError("Acceptance-test delta must be finite and within 0.01-0.02 rad")
    return result


def validate_step(current: Sequence[float], target: Sequence[float], maximum: float, label: str) -> None:
    if len(current) != len(target):
        raise SafetyInterlockError(f"{label} feedback/target length mismatch")
    largest = max(abs(float(goal) - float(now)) for now, goal in zip(current, target, strict=True))
    if largest > maximum:
        raise SafetyInterlockError(
            f"{label} largest one-shot change is {largest:.4f} rad; configured limit is {maximum:.4f} rad"
        )
