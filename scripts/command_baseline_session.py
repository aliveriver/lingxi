"""Bounded single-joint test preserving the already-active standing HAL targets."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

from lingxi_x2.acceptance import fixed_baseline_plan
from lingxi_x2.backends.ros_environment import reexec_with_ros_environment
from lingxi_x2.client import X2Client
from lingxi_x2.config import load_config
from lingxi_x2.control_trace import ControlTrace
from lingxi_x2.errors import SafetyInterlockError
from lingxi_x2.models import ARM_JOINT_NAMES
from lingxi_x2.publication import publish_points
from lingxi_x2.safety import validate_acceptance_delta


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--confirm-hardware", action="store_true", required=True)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--delta-rad", type=float, default=.01,
                        help="Signed diagnostic offset, magnitude 0.01–0.02 rad")
    args = parser.parse_args()
    delta = validate_acceptance_delta(args.delta_rad)
    if args.trace.exists():
        raise SystemExit("Choose an unused trace path")
    reexec_with_ros_environment()
    config = load_config("config/x2.yaml")
    config.control.command_echo = True
    result = {"kind": "fixed_command_baseline_session", "joint_index": 0, "delta_rad": delta, "events": []}
    frames = []
    code = 0
    with X2Client(config) as client, ControlTrace(client._backend) as trace:
        def event(phase):
            result["events"].append({"phase": phase, "monotonic_ns": time.monotonic_ns(),
                                     "arm": asdict(client.arm_state()), "mc": asdict(client.mc_state())})
        try:
            time.sleep(3.)
            report = client.control_preflight("arm")
            result["preflight"] = report.as_dict()
            failures = [c for c in report.checks if not c.passed and c.name not in {"control_enabled", "mc_mode"}]
            if failures or client.mc_state().action != "STAND_DEFAULT":
                raise SafetyInterlockError(f"Standing preflight failed: {failures}")
            snap = trace.snapshot()
            rows = snap["samples"]["hal_arm"]
            if snap["errors"] or not rows or time.monotonic_ns()-rows[-1]["received_monotonic_ns"] > .1e9:
                raise SafetyInterlockError("Missing fresh HAL command baseline")
            recent = [r for r in rows if r["received_monotonic_ns"] >= rows[-1]["received_monotonic_ns"]-.5e9]
            if len(recent) < 50:
                raise SafetyInterlockError("Insufficient baseline samples")
            baseline = tuple(j["position"] for j in rows[-1]["joints"])
            if any(tuple(j["name"] for j in row["joints"]) != ARM_JOINT_NAMES for row in recent):
                raise SafetyInterlockError("HAL joint ordering mismatch")
            if any(abs(j["position"]-base) > 1e-5 for row in recent for j, base in zip(row["joints"], baseline)):
                raise SafetyInterlockError("Standing HAL targets are not stationary")
            # Validate the entire bounded plan before changing MC mode.
            plan = list(fixed_baseline_plan(baseline, 0, delta, config.control.publish_rate_hz))
            result["baseline_command_rad"] = baseline
            result["baseline_hal_sample"] = rows[-1]
            event("before_mode")
            config.control.enabled = True
            try:
                result["mode_entry"] = client.set_motion_mode("UPPERBODY_REMOTE_SPLIT", confirm_hardware=True)
                def points():
                    previous = None
                    for phase, target in plan:
                        if phase != previous:
                            event(phase)
                            if phase == "target_ramp":
                                # With unchanged command targets, substantial drift
                                # invalidates a differential movement measurement.
                                start = result["events"][-2]["arm"]["joints"][0]["position_rad"]
                                current = result["events"][-1]["arm"]["joints"][0]["position_rad"]
                                if abs(current-start) > .005:
                                    raise SafetyInterlockError("Fixed-baseline qualification drift exceeds .005 rad")
                            previous = phase
                        yield phase, target
                client._publication_frames = frames
                with client._backend.command_stream("arm"):
                    publish_points(points(), lambda point: client._backend.publish_arm(
                        point, (0.,)*14, (0.,)*14, 20., 2.), config.control.publish_rate_hz, frames)
                result["publication_stats"] = client.publication_stats()
                event("recovery_complete")
                time.sleep(.5)
                event("publishing_stopped")
            finally:
                result["mode_restore"] = client.set_motion_mode("STAND_DEFAULT", confirm_hardware=True)
        except Exception as exc:
            code = 2
            result["error"] = str(exc)
            result["publication_stats"] = client.publication_stats()
        finally:
            config.control.enabled = False
            try:
                event("final")
                result["final_hands"] = {side.value: asdict(hand) for side, hand in client.hand_states().items()}
            except Exception as exc:
                result["feedback_error"] = str(exc)
            time.sleep(.3)
            snap = trace.snapshot()
            snap["result"] = result
            args.trace.parent.mkdir(parents=True, exist_ok=True)
            args.trace.write_text(json.dumps(snap), encoding="utf-8")
    print(json.dumps({"trace": str(args.trace), "error": result.get("error"),
                      "published_count": len(frames)}), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
