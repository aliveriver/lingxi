"""Explicit, bounded shoulder-only partial-gravity experiment; no global unlock.

Run --dry-run against live feedback first. Physical motion additionally needs
fresh operator authorization and --confirm-hardware. Disk control stays false.
"""
import argparse
from contextlib import ExitStack
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import yaml

from lingxi_x2.backends.ros_environment import reexec_with_ros_environment
from lingxi_x2.client import X2Client
from lingxi_x2.config import load_config
from lingxi_x2.control_trace import ControlTrace
from lingxi_x2.errors import SafetyInterlockError
from lingxi_x2.gravity_capture import GravityCapture, TOPICS
from lingxi_x2.gravity_diagnostic import (LimitedGravityGuard, limited_gravity_plan,
    check_encoder_excursion, diagnostic_mode, RATE_HZ)
from lingxi_x2.gravity_diagnostic import bounded_timing_window
from lingxi_x2.models import ARM_JOINT_NAMES
from lingxi_x2.publication import publish_points

MODEL = Path("logs/official-model-audit/pc2-sdk/x2_ultra.urdf")
CALIBRATION = Path("logs/official-model-audit/pc2-factory-calibration.yml")
HASHES = {MODEL: "f84fcd22187d7f3d9730f36b06b39e49fb6a59cd8b1d26bcb5a817dfd300b40a",
          CALIBRATION: "ce405931791f04ffdc14b6ac511e43e74912f0a86370ab327d20070d6c8dc0ef"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--confirm-hardware", action="store_true")
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--bias-only", action="store_true", help="Ramp bias in/hold/out without the additional +0.01 rad move")
    args = parser.parse_args()
    reexec_with_ros_environment()
    from sensor_msgs.msg import Imu
    from aimdk_msgs.msg import JointStateArray
    for path, expected in HASHES.items():
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise SafetyInterlockError(f"Audited model/calibration hash mismatch: {path}")
    config = load_config("config/x2.yaml")
    if config.control.enabled or config.control.authority != "upper_body_mc" or config.control.publish_rate_hz != RATE_HZ:
        raise SafetyInterlockError("Requires disabled site control, upper_body_mc and 50 Hz")
    config.control.command_echo = True
    guard = LimitedGravityGuard(MODEL, yaml.safe_load(CALIBRATION.read_text()))
    result = {"kind": "limited_gravity_baseline_session", "dry_run": args.dry_run,
              "hardware_acceptance_complete": False, "model_and_mounting_verified": False,
              "joint_index": 0, "delta_rad": 0. if args.bias_only else .01, "bias_limit_rad": .002,
              "bias_only": args.bias_only, "cyclic_gc_paused_during_bounded_stream": True,
              "total_command_excursion_limit_rad": .002 if args.bias_only else .012, "gravity_compensation_enabled": not args.dry_run,
              "assumed_stiffness_nm_rad": 40., "additional_payload_mass_kg": 0.,
              "model_sha256": HASHES[MODEL], "calibration_sha256": HASHES[CALIBRATION],
              "physical_publish_count": 0, "events": [], "decisions": []}
    frames, subscriptions = [], []
    code = 0
    # Exclusive creation protects previous trials; journal survives an ordinary abort.
    with args.trace.open("x") as output, args.trace.with_suffix(".commands.jsonl").open("x") as journal:
        def record(row):
            journal.write(json.dumps(row, allow_nan=False) + "\n")
            journal.flush()
        record({"kind": "trial_metadata", **{k:v for k,v in result.items() if k not in {"events", "decisions"}}})
        with X2Client(config) as client, ControlTrace(client._backend) as trace, ExitStack() as cleanup:
            capture = GravityCapture()
            node = client._backend._node
            for key,(topic,_) in TOPICS.items():
                def receive(message, selected=key):
                    capture.receive(selected, message, received_monotonic_ns=time.monotonic_ns(),
                                    received_ros_ns=node.get_clock().now().nanoseconds)
                sub = node.create_subscription(Imu if key.endswith("imu") else JointStateArray,
                                               topic, receive, client._backend._qos)
                subscriptions.append(sub)
                cleanup.callback(node.destroy_subscription, sub)

            def gravity_snapshot():
                return capture.snapshot(captured_monotonic_ns=time.monotonic_ns(),
                                        captured_ros_ns=node.get_clock().now().nanoseconds)

            def event(phase):
                value = {"phase": phase, "monotonic_ns": time.monotonic_ns(),
                         "arm": asdict(client.arm_state()), "mc": asdict(client.mc_state())}
                result["events"].append(value)
                record({"kind": "phase", **value})

            try:
                time.sleep(3.)
                report = client.control_preflight("arm")
                result["preflight"] = report.as_dict()
                if client.status().backend != "ros2" or client.mc_state().action != "STAND_DEFAULT" or any(
                        not c.passed for c in report.checks if c.name not in {"control_enabled", "mc_mode"}):
                    raise SafetyInterlockError("Standing preflight failed")
                raw = trace.snapshot()
                rows = raw["samples"]["hal_arm"]
                if raw["errors"] or not rows or time.monotonic_ns()-rows[-1]["received_monotonic_ns"] > .1e9:
                    raise SafetyInterlockError("No fresh HAL command baseline")
                recent = [r for r in rows if rows[-1]["received_monotonic_ns"]-r["received_monotonic_ns"] <= .5e9]
                baseline = tuple(j["position"] for j in rows[-1]["joints"])
                if len(recent) < 50 or any(tuple(j["name"] for j in r["joints"]) != ARM_JOINT_NAMES
                        or any(abs(j["position"]-q)>1e-5 for j,q in zip(r["joints"],baseline)) for r in recent):
                    raise SafetyInterlockError("HAL baseline not complete and stationary")
                plan = list(limited_gravity_plan(baseline, bias_only=args.bias_only))
                guard.prepare_desired_poses(frame["desired_rad"] for frame in plan)
                result["baseline_command_rad"] = list(baseline)
                reference = tuple(j.position_rad for j in client.arm_state().joints)
                result["qualification"] = []
                # Fresh repeated samples and both path endpoints before any mode change.
                for i in range(20):
                    snapshot = gravity_snapshot()
                    desired = plan[0 if i % 2 == 0 else 349]["desired_rad"]
                    decision = guard.evaluate(snapshot, desired, now_mono=time.monotonic_ns(),
                                               now_ros=node.get_clock().now().nanoseconds)
                    result["qualification"].append({"snapshot": snapshot, "decision": decision})
                    time.sleep(.05)
                event("before_mode")
                pending = None
                previous_phase = None

                def points():
                    nonlocal pending, previous_phase
                    for index,frame in enumerate(plan):
                        if frame["phase"] != previous_phase:
                            event(frame["phase"])
                            previous_phase = frame["phase"]
                        snapshot = gravity_snapshot()
                        started = time.monotonic_ns()
                        try:
                            decision = guard.evaluate(snapshot, frame["desired_rad"], now_mono=started,
                                                      now_ros=node.get_clock().now().nanoseconds)
                        except Exception:
                            result["rejected_gravity"] = {"index": index, "phase": frame["phase"],
                                "snapshot": snapshot, "checked_monotonic_ns": started,
                                "failure_monotonic_ns": time.monotonic_ns()}
                            raise
                        pending = {"index": index, **frame, "snapshot": snapshot, "gravity": decision,
                                   "computation_started_monotonic_ns": started,
                                   "computation_finished_monotonic_ns": time.monotonic_ns()}
                        yield frame["phase"], tuple(frame["command_rad"])

                def publish(point):
                    try:
                        guard.require_fresh(pending["snapshot"], time.monotonic_ns(), node.get_clock().now().nanoseconds)
                    except Exception:
                        result["rejected_gravity"] = {"pending": pending, "failure_monotonic_ns": time.monotonic_ns()}
                        raise
                    arm = client.arm_state()
                    if not 0 <= time.monotonic_ns()-arm.timestamp.monotonic_ns <= 100_000_000:
                        raise SafetyInterlockError("Arm feedback older than 100 ms")
                    check_encoder_excursion(arm.joints, reference)
                    if frames and time.monotonic_ns()-frames[-1]["monotonic_ns"] > 50_000_000:
                        raise SafetyInterlockError("Publication/shadow interval exceeds 50 ms")
                    pending["encoder_before_rad"] = [j.position_rad for j in arm.joints]
                    if args.dry_run:
                        mc = client.mc_state()
                        if (mc.action != "STAND_DEFAULT" or mc.action_status != 100 or mc.fsm_state != 4 or mc.body_state != 1):
                            raise SafetyInterlockError("Robot left standing during shadow check")
                        receipt = {"monotonic_ns": time.monotonic_ns(), "sequence": None, "measurement": "shadow_no_publish"}
                    else:
                        receipt = client._backend.publish_arm(point, (0.,)*14, (0.,)*14, 20., 2.)
                        result["physical_publish_count"] += 1
                    pending["receipt"] = receipt
                    result["decisions"].append(pending)
                    record({"kind": "shadow_decision" if args.dry_run else "published_compensated_command", **pending})
                    return receipt

                if not args.dry_run:
                    client._publication_frames = frames
                with bounded_timing_window(), diagnostic_mode(client, result, dry_run=args.dry_run):
                    timing = publish_points(points(), publish, RATE_HZ, frames)
                    if args.dry_run:
                        timing["computed_count"] = timing.pop("published_count")
                        timing["evidence"] = "paced shadow computations; zero physical commands"
                    result["shadow_timing" if args.dry_run else "publication_stats"] = {k:v for k,v in timing.items() if k != "frames"}
                    event("recovery_complete")
                    time.sleep(.3)
                    event("publishing_stopped")
            except Exception as exc:
                result["error"] = f"{type(exc).__name__}: {exc}"
                code = 2
            finally:
                config.control.enabled = False
                if not args.dry_run:
                    result["publication_stats"] = client.publication_stats()
                try:
                    event("final")
                    result["final_hands"] = {s.value: asdict(h) for s,h in client.hand_states().items()}
                except Exception as exc:
                    result["feedback_error"] = str(exc)
                result["final_gravity_snapshot"] = gravity_snapshot()
                result["command_publisher_created"] = client._backend._upper_publisher is not None
                time.sleep(.3)
                final = trace.snapshot()
                final["result"] = result
                json.dump(final, output, allow_nan=False)
                record({"kind": "trial_finished", "error": result.get("error"),
                        "physical_publish_count": result["physical_publish_count"]})
    print(json.dumps({"trace": str(args.trace), "dry_run": args.dry_run,
                      "error": result.get("error"), "physical_publish_count": result["physical_publish_count"]}), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
