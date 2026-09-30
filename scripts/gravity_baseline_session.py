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
import sys

import yaml

from lingxi_x2.backends.ros_environment import reexec_with_ros_environment
from lingxi_x2.client import X2Client
from lingxi_x2.config import load_config
from lingxi_x2.control_trace import ControlTrace
from lingxi_x2.drive_trace import DriveTrace
from lingxi_x2.errors import SafetyInterlockError
from lingxi_x2.gravity_capture import GravityCapture, TOPICS
from lingxi_x2.gravity_diagnostic import (LimitedGravityGuard, limited_gravity_plan,
    check_encoder_excursion, diagnostic_mode, RATE_HZ, diagnostic_protocol,
    validate_model_plan, MODEL_SHA256, CALIBRATION_SHA256)
from lingxi_x2.gravity_diagnostic import bounded_timing_window, StaticSettlingError
from lingxi_x2.gravity_preparation import (PreparationCapture, PreparationMonitor,
    TOPICS as PREPARATION_TOPICS, POLICY as PREPARATION_POLICY, fixed_references,
    wait_for_preparation, validate_preparation_hands)
from lingxi_x2.gravity_trial_report import evaluate_trace
from lingxi_x2.models import ARM_JOINT_NAMES, Side
from lingxi_x2.publication import publish_points

MODEL = Path("logs/official-model-audit/pc2-sdk/x2_ultra.urdf")
CALIBRATION = Path("logs/official-model-audit/pc2-factory-calibration.yml")
HASHES = {MODEL: MODEL_SHA256, CALIBRATION: CALIBRATION_SHA256}


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--confirm-hardware", action="store_true")
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--bias-only", action="store_true", help="Ramp bias in/hold/out without the additional +0.01 rad move")
    parser.add_argument("--compensation", choices=("on", "off"), required=True,
                        help="Explicitly select capped bias or matched zero-bias control; all guards remain active")
    return parser


def review_completed_session(trace, execution_code):
    review = evaluate_trace(trace)
    trace["result"]["acceptance"] = review
    passed = review["verdict"] in {"single_trial_pass_under_engineering_criteria", "bias_only_diagnostic_complete"}
    return execution_code or (0 if passed else 3)


def main(argv=None):
    args = build_parser().parse_args(argv)
    compensation_enabled = args.compensation == "on"
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
    result = {"kind": "limited_gravity_baseline_session", "session_schema_version": 2, "dry_run": args.dry_run,
              "hardware_acceptance_complete": False, "model_and_mounting_verified": False,
              "joint_index": 0, "delta_rad": 0. if args.bias_only else .01, "bias_limit_rad": .002,
              "bias_only": args.bias_only, "cyclic_gc_paused_during_bounded_stream": True,
              "cyclic_gc_paused_during_bounded_session": True,
              "total_command_excursion_limit_rad": (.002 if args.bias_only else .012) if compensation_enabled else (0. if args.bias_only else .01),
              "gravity_compensation_enabled": compensation_enabled and not args.dry_run,
              "compensation_requested": compensation_enabled,
              "diagnostic_protocol": diagnostic_protocol(bias_only=args.bias_only, compensation_enabled=compensation_enabled),
              "assumed_stiffness_nm_rad": 40., "additional_payload_mass_kg": 0.,
              "model_sha256": HASHES[MODEL], "calibration_sha256": HASHES[CALIBRATION],
              "physical_publish_count": 0, "events": [], "decisions": [],
              "preparation": {"policy": dict(PREPARATION_POLICY), "batches": [], "events": []}}
    frames, subscriptions = [], []
    retained_preparation_rows = 0
    code = 0
    # Exclusive creation protects previous trials; journal survives an ordinary abort.
    with args.trace.open("x") as output, args.trace.with_suffix(".commands.jsonl").open("x") as journal:
        def record(row):
            journal.write(json.dumps(row, allow_nan=False) + "\n")
            journal.flush()
        record({"kind": "trial_metadata", **{k:v for k,v in result.items() if k not in {"events", "decisions"}}})
        with bounded_timing_window(), X2Client(config) as client, ControlTrace(client._backend) as trace, DriveTrace(client._backend) as drive, ExitStack() as cleanup:
            result["python_thread_switch_interval_s"] = sys.getswitchinterval()
            capture = GravityCapture()
            preparation_capture = PreparationCapture()
            node = client._backend._node
            for key,(topic,_) in TOPICS.items():
                def receive(message, selected=key):
                    mono, ros = time.monotonic_ns(), node.get_clock().now().nanoseconds
                    capture.receive(selected, message, received_monotonic_ns=mono, received_ros_ns=ros)
                    preparation_capture.receive(selected, message, mono=mono, ros=ros)
                sub = node.create_subscription(Imu if key.endswith("imu") else JointStateArray,
                                               topic, receive, client._backend._qos)
                subscriptions.append(sub)
                cleanup.callback(node.destroy_subscription, sub)

            for key in ("hands", "hal_arm", "mc"):
                topic, message_type = PREPARATION_TOPICS[key]
                def receive_extra(message, selected=key):
                    preparation_capture.receive(selected, message, mono=time.monotonic_ns(),
                                                ros=node.get_clock().now().nanoseconds)
                sub = node.create_subscription(client._backend._types[message_type], topic, receive_extra,
                    client._backend._state_qos if key == "mc" else client._backend._qos)
                cleanup.callback(node.destroy_subscription, sub)

            def journal_preparation_batch(batch):
                # Bound JSON encoder GIL occupancy while draining startup history.
                metadata = {k:v for k,v in batch.items() if k != "rows"}
                for begin in range(0, max(1, len(batch["rows"])), 64):
                    record({"kind": "preparation_samples", **metadata,
                            "row_offset": begin, "rows": batch["rows"][begin:begin+64]})
                    time.sleep(0)

            def preparation_batch(stage, *, journal_now=True):
                nonlocal retained_preparation_rows
                batch = {**preparation_capture.drain(), "stage": stage,
                         "now_ns": time.monotonic_ns(), "now_ros": node.get_clock().now().nanoseconds}
                result["preparation"]["batches"].append(batch)
                retained_preparation_rows += len(batch["rows"])
                if retained_preparation_rows > 100000 and stage != "after_restore":
                    raise SafetyInterlockError("Preparation telemetry retention limit exceeded")
                if journal_now:
                    journal_preparation_batch(batch)
                return batch

            def preparation_event(phase):
                row = {"phase": phase, "monotonic_ns": time.monotonic_ns()}
                result["preparation"]["events"].append(row)
                record({"kind": "preparation_phase", **row})

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
                if not args.dry_run:
                    drive.require_recent(now_ns=time.monotonic_ns())
                raw = trace.snapshot()
                rows = raw["samples"]["hal_arm"]
                if raw["errors"] or not rows or time.monotonic_ns()-rows[-1]["received_monotonic_ns"] > .1e9:
                    raise SafetyInterlockError("No fresh HAL command baseline")
                recent = [r for r in rows if rows[-1]["received_monotonic_ns"]-r["received_monotonic_ns"] <= .5e9]
                baseline = tuple(j["position"] for j in rows[-1]["joints"])
                if len(recent) < 50 or any(tuple(j["name"] for j in r["joints"]) != ARM_JOINT_NAMES
                        or any(abs(j["position"]-q)>1e-5 for j,q in zip(r["joints"],baseline)) for r in recent):
                    raise SafetyInterlockError("HAL baseline not complete and stationary")
                plan = list(limited_gravity_plan(baseline, bias_only=args.bias_only,
                                                compensation_enabled=compensation_enabled))
                validate_model_plan(guard.model, plan)
                guard.prepare_desired_poses(frame["desired_rad"] for frame in plan)
                result["baseline_command_rad"] = list(baseline)
                acquisition = preparation_batch("acquisition")
                # DDS discovery history is retained, but is not a qualification
                # window. Faults still latch before establishing references.
                if acquisition["error"] or any(j.get("error_code") != 0
                        for item in acquisition["rows"] if item["stream"] in {"arm_state", "waist_state", "hands"}
                        for j in item["sample"]["joints"]):
                    raise SafetyInterlockError("Fault/error during telemetry acquisition")
                # Seed from current arrivals AFTER the large startup journal.
                # Persist this small tail after checking the fresh seed, so disk
                # serialization cannot age the reference before its first check.
                acquisition_tail = preparation_batch("acquisition", journal_now=False)
                if acquisition_tail["error"] or any(j.get("error_code") != 0
                        for item in acquisition_tail["rows"] if item["stream"] in {"arm_state", "waist_state", "hands"}
                        for j in item["sample"]["joints"]):
                    raise SafetyInterlockError("Fault/error during telemetry acquisition")
                latest = {item["stream"]: item["sample"] for item in acquisition["rows"]}
                latest.update({item["stream"]: item["sample"] for item in acquisition_tail["rows"]})
                initial = {"stage": "qualification", "rows": [
                    {"stream": key, "sample": row} for key,row in latest.items()], "error": None,
                    "now_ns": time.monotonic_ns(), "now_ros": node.get_clock().now().nanoseconds}
                result["preparation"]["batches"].append(initial)
                record({"kind": "preparation_reference_samples", **initial})
                references = fixed_references(latest)
                reference = tuple(references["arm_state"])
                result["encoder_reference_rad"] = list(reference)
                result["preparation"]["references"] = references
                result["preparation"]["started_ns"] = initial["now_ns"]
                monitor = PreparationMonitor(baseline, references, started_ns=initial["now_ns"], action="STAND_DEFAULT")
                monitor.consume(initial, now_ns=initial["now_ns"], now_ros=initial["now_ros"])
                journal_preparation_batch(acquisition_tail)

                def monitor_check(stage, *, active=False, allow_gain_transition=False):
                    batch = preparation_batch(stage, journal_now=stage != "trajectory")
                    status = monitor.consume(batch, now_ns=batch["now_ns"], now_ros=batch["now_ros"], active=active,
                                             allow_gain_transition=allow_gain_transition)
                    batch["status"] = status
                    return status

                def settled_check(stage):
                    status = monitor_check(stage)
                    try:
                        guard.evaluate(gravity_snapshot(), baseline, now_mono=time.monotonic_ns(),
                                       now_ros=node.get_clock().now().nanoseconds)
                    except StaticSettlingError as exc:
                        monitor.reset(str(exc))
                        status["ready"] = False
                    return status

                wait_for_preparation(lambda: settled_check("qualification"))
                result["qualification"] = []
                # Fresh repeated samples and both path endpoints before any mode change.
                for i in range(20):
                    monitor_check("qualification")
                    snapshot = gravity_snapshot()
                    desired = plan[0 if i % 2 == 0 else 349]["desired_rad"]
                    decision = guard.evaluate(snapshot, desired, now_mono=time.monotonic_ns(),
                                               now_ros=node.get_clock().now().nanoseconds)
                    result["qualification"].append({"snapshot": snapshot, "decision": decision})
                    time.sleep(.05)
                # Reconfirm a complete window immediately before the mode request.
                wait_for_preparation(lambda: settled_check("qualification"))
                event("before_mode")
                if not args.dry_run:
                    drive.require_recent(now_ns=time.monotonic_ns())
                preparation_event("preparation_started")
                monitor.begin_preparation(result["preparation"]["events"][-1]["monotonic_ns"],
                    "STAND_DEFAULT" if args.dry_run else "UPPERBODY_REMOTE_SPLIT")

                def prepare():
                    preparation_event("mode_returned")
                    wait_for_preparation(lambda: settled_check("preparation"))
                    preparation_event("stable_confirmed")

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
                    monitor_check("trajectory", active=True, allow_gain_transition=pending["phase"] == "baseline_hold")
                    if not args.dry_run:
                        held = [q for side in Side for q in client._backend._held_hands[side]]
                        validate_preparation_hands(held)
                        # command_stream captures hands only AFTER settling. Never
                        # silently accept a different hand pose after mode entry.
                        if len(held) != 20 or any(abs(a-b) > PREPARATION_POLICY["hand_excursion_rad"]
                                for a,b in zip(held, references["hands"])):
                            raise SafetyInterlockError("Stream hand targets differ from pre-mode reference")
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
                    monitor.require_fresh(time.monotonic_ns(), node.get_clock().now().nanoseconds)
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
                    # Keep the realtime journal compact. Every raw callback and
                    # full gravity decision remains in the bounded session trace;
                    # flush raw trajectory telemetry only after leaving the mode.
                    record({"kind": "shadow_decision" if args.dry_run else (
                        "published_compensated_command" if compensation_enabled else "published_control_command"),
                        **{k:v for k,v in pending.items() if k not in {"snapshot", "gravity"}}})
                    return receipt

                if not args.dry_run:
                    client._publication_frames = frames
                fixed_hands = {Side.LEFT: tuple(references["hands"][:10]), Side.RIGHT: tuple(references["hands"][10:])}
                result["fixed_hand_targets_rad"] = {side.value: list(q) for side,q in fixed_hands.items()}
                with diagnostic_mode(client, result, dry_run=args.dry_run, prepare=prepare, hand_targets=fixed_hands):
                    timing = publish_points(points(), publish, RATE_HZ, frames, max_frame_interval_s=.05)
                    if args.dry_run:
                        timing["computed_count"] = timing.pop("published_count")
                        timing["evidence"] = "paced shadow computations; zero physical commands"
                    result["shadow_timing" if args.dry_run else "publication_stats"] = {k:v for k,v in timing.items() if k != "frames"}
                    event("recovery_complete")
                    observation_end = time.monotonic() + .3
                    while time.monotonic() < observation_end:
                        monitor_check("urs_observation", active=True)
                        time.sleep(.02)
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
                preparation_batch("after_restore")
                for batch in result["preparation"]["batches"]:
                    if batch["stage"] == "trajectory":
                        journal_preparation_batch(batch)
                final = trace.snapshot()
                result["drive_trace"] = drive.snapshot()
                final["result"] = result
                if not args.dry_run:
                    code = review_completed_session(final, code)
                json.dump(final, output, allow_nan=False)
                record({"kind": "trial_finished", "error": result.get("error"),
                        "physical_publish_count": result["physical_publish_count"]})
    print(json.dumps({"trace": str(args.trace), "dry_run": args.dry_run,
                      "error": result.get("error"), "physical_publish_count": result["physical_publish_count"]}), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
