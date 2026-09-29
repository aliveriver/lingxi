from __future__ import annotations

import math
from collections.abc import Sequence

from .errors import SafetyInterlockError
from .models import ARM_JOINT_NAMES


_DEG = math.pi / 180.0

# Official AimDK 1.1.0 "joint_name_and_limit" guaranteed ranges for X2 Ultra.
_ONE_ARM_LIMITS_DEG = (
    (-176.5, 116.5),
    (-3.5, 174.5),
    (-146.5, 146.5),
    (-135.0, 0.0),
    (-146.5, 146.5),
    (-30.0, 30.0),
    (-86.5, 41.5),
)
ARM_LIMITS_RAD = tuple((low * _DEG, high * _DEG) for _ in range(2) for low, high in _ONE_ARM_LIMITS_DEG)


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
    if not math.isfinite(result) or not 0.01 <= abs(result) <= 0.02:
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
