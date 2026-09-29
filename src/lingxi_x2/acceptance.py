"""Fixed-command-baseline diagnostics: do not rebase targets onto encoder error."""
import math

from .errors import SafetyInterlockError
from .safety import validate_acceptance_delta, validate_arm_target
from .compensation import quintic


def fixed_baseline_plan(baseline, joint_index, delta_rad, rate_hz=50.0, *,
                        profile="linear", ramp_duration_s=1.0):
    baseline = validate_arm_target(baseline)
    if not 0 <= joint_index < 14 or not math.isfinite(rate_hz) or not 20 <= rate_hz <= 200:
        raise SafetyInterlockError("Invalid joint or diagnostic rate")
    delta = validate_acceptance_delta(delta_rad)
    if profile not in {"linear", "quintic"} or not math.isfinite(ramp_duration_s) or not 1 <= ramp_duration_s <= 3:
        raise SafetyInterlockError("Diagnostic profile must be linear/quintic with 1–3 second ramps")
    target = list(baseline)
    target[joint_index] += delta
    target = validate_arm_target(target)
    steps = math.ceil(rate_hz)
    ramp_steps = math.ceil(rate_hz * ramp_duration_s)
    def fraction(step):
        progress = step / ramp_steps
        return quintic(progress) if profile == "quintic" else progress
    for _ in range(steps):
        yield "baseline_hold", baseline
    for step in range(1, ramp_steps + 1):
        yield "target_ramp", tuple(a + (b-a) * fraction(step) for a, b in zip(baseline, target))
    for _ in range(steps):
        yield "target_hold", target
    for step in range(1, ramp_steps + 1):
        yield "recovery_ramp", tuple(a + (b-a) * fraction(step) for a, b in zip(target, baseline))
    for _ in range(steps):
        yield "recovery_hold", baseline
