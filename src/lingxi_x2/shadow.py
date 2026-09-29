"""Read-only model experiments: record proposals, never execute them.

VLA/WAM adapters implement Policy or use JointActionAdapter around an inference
callable. The journal records proposed actions and wall latency, not successes.
"""
from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

from .experiments import Policy, PolicyAction
from .models import ArmCommand, HandCommand, Observation, Side
from .recording import observation_to_dict


def validate_proposal(action: PolicyAction) -> None:
    if not isinstance(action, PolicyAction) or type(action.stop) is not bool:
        raise ValueError("Policy must return PolicyAction with a boolean stop")
    if action.stop and (action.arm is not None or action.hands):
        raise ValueError("A stop action must not contain motion targets")
    if action.arm is not None:
        arm = action.arm
        if not isinstance(arm, ArmCommand) or len(arm.positions_rad) != 14:
            raise ValueError("Arm proposal requires 14 joint positions")
        if arm.velocities_rad_s is not None or arm.efforts_nm is not None:
            raise ValueError("Model adapter supports position proposals only")
        if arm.stiffness_nm_rad != 20. or arm.damping_nm_s_rad != 2.:
            raise ValueError("Model adapter cannot set MC gains")
        values = (*arm.positions_rad, arm.duration_s)
        if not all(math.isfinite(v) for v in values) or not 0 < arm.duration_s <= 30:
            raise ValueError("Invalid arm proposal values/duration")
    seen = set()
    for hand in action.hands:
        if not isinstance(hand, HandCommand) or not isinstance(hand.side, Side) or hand.side in seen:
            raise ValueError("Hand proposals require distinct valid sides")
        seen.add(hand.side)
        if len(hand.positions_rad) != 10 or not all(math.isfinite(v) for v in (*hand.positions_rad, hand.duration_s)):
            raise ValueError("Hand proposal requires 10 finite joint positions")
        if not 0 < hand.duration_s <= 30:
            raise ValueError("Invalid hand proposal duration")


class JointActionAdapter:
    """Wrap a user-supplied VLA/WAM inference callable with an explicit contract.

    Output must be {units: 'rad', representation: 'absolute_joint_positions',
    duration_s: float, arm_positions_rad?: 14 values,
    hand_positions_rad?: {left/right: 10 values}} or {stop: true}.
    No implicit unit conversion, relative integration, IK or chunk flattening.
    """

    def __init__(self, infer: Callable[[Observation], Mapping], reset: Callable[[], None] | None = None):
        self.infer = infer
        self._reset = reset

    def reset(self):
        if self._reset is not None:
            self._reset()

    def act(self, observation: Observation) -> PolicyAction:
        raw = self.infer(observation)
        if not isinstance(raw, Mapping):
            raise ValueError("Inference output must be a mapping")
        if dict(raw) == {"stop": True} and type(raw["stop"]) is bool:
            return PolicyAction(stop=True)
        allowed = {"units", "representation", "duration_s", "arm_positions_rad", "hand_positions_rad"}
        if set(raw) - allowed or not {"units", "representation", "duration_s"} <= set(raw):
            raise ValueError("Unrecognized model action schema")
        if raw["units"] != "rad" or raw["representation"] != "absolute_joint_positions":
            raise ValueError("Model action must explicitly use absolute joint positions in rad")
        duration = float(raw["duration_s"])
        if not math.isfinite(duration) or not 0 < duration <= 30:
            raise ValueError("Invalid model action duration")
        arm = ArmCommand.from_positions(raw["arm_positions_rad"], duration) if "arm_positions_rad" in raw else None
        hands = tuple(HandCommand.from_positions(side, values, duration)
                      for side, values in raw.get("hand_positions_rad", {}).items())
        action = PolicyAction(arm, hands)
        validate_proposal(action)
        return action


class ShadowExperimentRunner:
    """One inference per observation, journal flushed at every event.

    The policy is trusted caller code. This runner has no actuation capability;
    it cannot sandbox a malicious policy that opens its own hardware connection.
    A deadline is measured after act() returns; Python inference is not forcibly
    cancelled. On violation, log the event and stop, with no next action.
    """

    def __init__(self, policy: Policy):
        self.policy = policy

    def run(self, observations: Iterable[Observation], output: str | Path, *, max_steps: int,
            inference_deadline_s: float = 1., source: str = "unspecified", include_images: bool = False) -> dict:
        if type(max_steps) is not int or max_steps <= 0:
            raise ValueError("max_steps must be a positive integer")
        if not math.isfinite(inference_deadline_s) or inference_deadline_s <= 0:
            raise ValueError("inference_deadline_s must be finite and positive")
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        count, previous, max_latency, reason = 0, None, 0., "max_steps"
        with path.open("x", encoding="utf-8") as journal:
            def emit(value):
                journal.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
                journal.flush()
            emit({"record_type": "metadata", "format": "lingxi-x2-shadow-v1", "source": source,
                  "created_unix_ns": time.time_ns(), "execution": "none", "motion_verified": False,
                  "inference_deadline_s": inference_deadline_s,
                  "validation_scope": "proposal schema and finite values, not collision/limits/physical safety"})
            try:
                self.policy.reset()
                iterator = iter(observations)
                while count < max_steps:
                    try:
                        observation = next(iterator)
                    except StopIteration:
                        reason = "end_of_observations"
                        break
                    captured = observation.captured_monotonic_ns
                    if previous is not None and captured <= previous:
                        raise ValueError("Observation capture times must strictly increase")
                    previous = captured
                    # Persist exactly what was offered before inference can fail.
                    emit({"record_type": "observation", "step": count,
                          "observation": observation_to_dict(observation, include_images)})
                    started = time.monotonic_ns()
                    action = self.policy.act(observation)
                    latency = (time.monotonic_ns() - started) / 1e9
                    max_latency = max(max_latency, latency)
                    validate_proposal(action)
                    missed = latency > inference_deadline_s
                    emit({"record_type": "proposal", "step": count, "action": asdict(action),
                          "inference_latency_s": latency, "deadline_missed": missed, "executed": False})
                    count += 1
                    if missed or action.stop:
                        reason = "inference_deadline_missed" if missed else "policy_stop"
                        break
            except Exception as exc:
                emit({"record_type": "error", "step": count, "error_type": type(exc).__name__, "error": str(exc),
                      "executed": False})
                raise
            result = {"record_type": "summary", "proposals": count, "reason": reason,
                      "max_inference_latency_s": max_latency, "executed_actions": 0, "motion_verified": False}
            emit(result)
        return result
