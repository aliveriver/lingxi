"""Independent, offline quintic planning and bounded static position bias.

No transport or live gravity estimator. Plans are declared scenarios, never
hardware authorization. Bias depends on desired pose, not encoder error.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, StrictBool, model_validator

from .gravity import StaticArmModel
from .models import ARM_JOINT_NAMES
from .safety import validate_arm_target


class _StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class Payload(_StrictModel):
    mass_kg: float = Field(ge=0, strict=True)
    com_wrist_m: tuple[float, float, float]


class CompensationRequest(_StrictModel):
    baseline_rad: dict[str, float]
    target_rad: dict[str, float]
    motion_joints: tuple[str, ...]
    compensation_joints: tuple[str, ...]
    enabled: StrictBool
    gravity_torso_m_s2: tuple[float, float, float]
    payloads: dict[str, Payload]
    assumed_stiffness_nm_rad: float = Field(gt=0, strict=True)
    bias_limit_rad: float = Field(gt=0, le=.02, strict=True)
    max_bias_rate_rad_s: float = Field(gt=0, le=.1, strict=True)
    max_command_rate_rad_s: float = Field(gt=0, le=.2, strict=True)
    max_command_excursion_rad: float = Field(gt=0, le=.04, strict=True)
    rate_hz: float = Field(ge=20, le=100, strict=True)
    ramp_duration_s: float = Field(ge=.1, le=30, strict=True)
    move_duration_s: float = Field(ge=.1, le=30, strict=True)
    hold_duration_s: float = Field(ge=.1, le=10, strict=True)

    @model_validator(mode="after")
    def validate_scenario(self):
        names = set(ARM_JOINT_NAMES)
        if set(self.baseline_rad) != names or set(self.target_rad) != names:
            raise ValueError("baseline_rad and target_rad must name all 14 joints")
        for label in ("motion_joints", "compensation_joints"):
            selected = getattr(self, label)
            if len(set(selected)) != len(selected) or not set(selected) <= names:
                raise ValueError(f"{label} must contain unique canonical joint names")
        if self.enabled and not self.compensation_joints:
            raise ValueError("Enabled compensation requires explicit compensation_joints")
        for name in ARM_JOINT_NAMES:
            delta = self.target_rad[name] - self.baseline_rad[name]
            if name not in self.motion_joints and delta != 0:
                raise ValueError(f"Unselected motion joint changed: {name}")
            if abs(delta) > .02 + 1e-12:
                raise ValueError("Diagnostic desired excursion cannot exceed 0.02 rad")
        if set(self.payloads) != {"left", "right"}:
            raise ValueError("Explicit left and right payloads required")
        if not 9 <= np.linalg.norm(self.gravity_torso_m_s2) <= 10.5:
            raise ValueError("Expected declared torso gravity magnitude 9–10.5 m/s²")
        validate_arm_target(tuple(self.baseline_rad[n] for n in ARM_JOINT_NAMES))
        validate_arm_target(tuple(self.target_rad[n] for n in ARM_JOINT_NAMES))
        return self


def quintic(progress: float) -> float:
    """Zero endpoint velocity and acceleration; domain is strictly [0, 1]."""
    if not math.isfinite(progress) or not 0 <= progress <= 1:
        raise ValueError("Quintic progress must be finite and within [0, 1]")
    return progress ** 3 * (10 + progress * (-15 + 6 * progress))


def plan_compensated_joint_session(urdf: str | Path, request: dict) -> dict:
    """Prevalidate every frame before returning any publishable mock targets.

    Only amplitude is clipped (and reported). Rate, envelope and joint-limit
    violations reject the whole plan; they never silently distort the path.
    """
    req = CompensationRequest.model_validate(request)
    model = StaticArmModel(urdf)
    baseline = np.array([req.baseline_rad[n] for n in ARM_JOINT_NAMES])
    delta = np.array([req.target_rad[n] for n in ARM_JOINT_NAMES]) - baseline
    selected = np.array([n in req.compensation_joints for n in ARM_JOINT_NAMES])
    lower = np.array([j.lower for side in ("left", "right") for j in model.arms[side]])
    upper = np.array([j.upper for side in ("left", "right") for j in model.arms[side]])
    effort = np.array([j.effort for side in ("left", "right") for j in model.arms[side]])
    frames = []
    previous_bias, previous_command = np.zeros(14), baseline.copy()
    max_bias_rate = max_command_rate = 0.

    def append(phase, progress, motion_fraction, bias_fraction):
        nonlocal previous_bias, previous_command, max_bias_rate, max_command_rate
        desired = baseline + delta * motion_fraction
        tau = []
        for side, offset in (("left", 0), ("right", 7)):
            payload = req.payloads[side]
            result = model.evaluate(side, desired[offset:offset + 7], req.gravity_torso_m_s2,
                                    payload_mass_kg=payload.mass_kg,
                                    payload_com_wrist_m=payload.com_wrist_m)
            tau.extend(result["gravity_torque_nm"])
        tau = np.array(tau)
        if req.enabled and np.any(np.abs(tau[selected]) > effort[selected]):
            raise ValueError("Selected gravity torque exceeds URDF effort metadata")
        raw = tau / req.assumed_stiffness_nm_rad
        if not np.isfinite(raw).all():
            raise ValueError("Nonfinite gravity/stiffness bias")
        clipped = np.clip(raw, -req.bias_limit_rad, req.bias_limit_rad)
        bias = np.where(selected, clipped * bias_fraction, 0.) if req.enabled else np.zeros(14)
        command = desired + bias
        validate_arm_target(command)
        if np.any(command < lower) or np.any(command > upper):
            raise ValueError("Compensated command outside URDF limits; no implicit clipping")
        if np.any(np.abs(command - baseline) > req.max_command_excursion_rad + 1e-12):
            raise ValueError("Compensated command exceeds fixed-baseline excursion envelope")
        bias_rate = float(np.max(np.abs(bias - previous_bias)) * req.rate_hz)
        command_rate = float(np.max(np.abs(command - previous_command)) * req.rate_hz)
        if bias_rate > req.max_bias_rate_rad_s + 1e-12:
            raise ValueError("Bias rate exceeded; lengthen ramp/move duration")
        if command_rate > req.max_command_rate_rad_s + 1e-12:
            raise ValueError("Command rate exceeded; lengthen ramp/move duration")
        max_bias_rate = max(max_bias_rate, bias_rate)
        max_command_rate = max(max_command_rate, command_rate)
        frames.append({"index": len(frames), "time_from_start_s": len(frames) / req.rate_hz,
                       "phase": phase, "phase_progress": progress,
                       "bias_fraction": bias_fraction if req.enabled else 0.,
                       "desired_rad": desired.tolist(), "gravity_torque_nm": tau.tolist(),
                       "unbounded_bias_rad": raw.tolist(), "applied_bias_rad": bias.tolist(),
                       "saturated_joints": [n for n, mask, r in zip(ARM_JOINT_NAMES, selected, raw)
                                            if req.enabled and bias_fraction > 0 and mask
                                            and abs(r) > req.bias_limit_rad],
                       "command_rad": command.tolist()})
        previous_bias, previous_command = bias, command

    append("baseline_hold", 0., 0., 0.)
    for phase, duration in (("baseline_hold", req.hold_duration_s),
                            ("bias_ramp_in", req.ramp_duration_s),
                            ("move", req.move_duration_s),
                            ("target_hold", req.hold_duration_s),
                            ("return", req.move_duration_s),
                            ("bias_ramp_out", req.ramp_duration_s),
                            ("recovery_hold", req.hold_duration_s)):
        steps = math.ceil(duration * req.rate_hz)
        for step in range(1, steps + 1):
            progress = step / steps
            s = quintic(progress)
            motion_fraction = s if phase == "move" else 1 - s if phase == "return" else float(phase == "target_hold")
            bias_fraction = s if phase == "bias_ramp_in" else 1 - s if phase == "bias_ramp_out" else float(phase in {"move", "target_hold", "return"})
            append(phase, progress, motion_fraction, bias_fraction)
    return {"kind": "offline_compensated_joint_plan", "schema_version": 1,
            "hardware_validated": False, "hardware_execution_allowed": False,
            "gravity_source": "declared_fixed_offline_scenario_not_live_feedback",
            "model": {"path": str(urdf), "sha256": model.sha256, "robot_name": model.robot_name},
            "joint_names": list(ARM_JOINT_NAMES), "request": req.model_dump(mode="json"),
            "summary": {"frame_count": len(frames), "duration_s": len(frames) / req.rate_hz,
                        "max_bias_rate_rad_s": max_bias_rate,
                        "max_command_rate_rad_s": max_command_rate,
                        "saturated_frame_count": sum(bool(f["saturated_joints"]) for f in frames)},
            "frames": frames}


def read_compensation_plan(path: str | Path, *, urdf: str | Path | None = None) -> dict:
    """Verify the saved scenario, model hash and every frame before replay.

    An explicit URDF override supports moving artifacts between machines.
    Replay is only data output; it cannot execute the saved targets.
    """
    from .replay import strict_json
    saved = strict_json(Path(path).read_text(encoding="utf-8"))
    if not isinstance(saved, dict) or not isinstance(saved.get("model"), dict) or "request" not in saved:
        raise ValueError("Expected a saved compensation plan")
    model_path = urdf if urdf is not None else saved["model"].get("path")
    if not isinstance(model_path, (str, Path)):
        raise ValueError("A URDF path is required for plan replay")
    checked = plan_compensated_joint_session(model_path, saved["request"])
    # Only a path relocation is allowed. No stored frame is trusted as a command.
    checked["model"]["path"] = saved["model"].get("path")
    def same_computation(actual, expected):
        # NumPy/CPU implementations differ by a few ULPs. Metadata remains
        # exact; only recomputed numeric outputs tolerate absolute roundoff.
        if type(actual) is not type(expected):
            return False
        if isinstance(expected, float):
            return math.isclose(actual, expected, rel_tol=0., abs_tol=1e-12)
        if isinstance(expected, dict):
            return actual.keys() == expected.keys() and all(same_computation(actual[k], v) for k, v in expected.items())
        if isinstance(expected, list):
            return len(actual) == len(expected) and all(same_computation(a, b) for a, b in zip(actual, expected))
        return actual == expected

    outputs = {"frames", "summary"}
    if (saved.keys() != checked.keys()
            or any(saved[k] != checked[k] for k in checked.keys() - outputs)
            or any(not same_computation(saved[k], checked[k]) for k in outputs)):
        raise ValueError("Saved compensation plan differs from recomputed model/scenario")
    return saved


def replay_compensation_plan(path: str | Path, *, urdf: str | Path | None = None,
                             speed: float | None = None):
    """Yield checked plan frames with original relative time; no robot/client."""
    import time
    if speed is not None and (not math.isfinite(speed) or not .01 <= speed <= 100):
        raise ValueError("Replay speed must be finite and within [0.01, 100]")
    plan = read_compensation_plan(path, urdf=urdf)
    period = 1 / plan["request"]["rate_hz"]
    deadline = time.monotonic()
    for frame in plan["frames"]:
        if speed is not None:
            time.sleep(max(0., deadline - time.monotonic()))
        started = time.monotonic()
        yield {"kind": "compensation_plan_replay", "execution": "none",
               "hardware_validated": False, "model_sha256": plan["model"]["sha256"], **frame}
        if speed is not None:
            deadline = max(started, time.monotonic()) + period / speed
