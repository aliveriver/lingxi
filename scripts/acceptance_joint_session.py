"""Operator-authorized +0.01 rad diagnostic; on-disk control stays disabled."""
from dataclasses import asdict
import json
import sys
import argparse
from pathlib import Path
import time

from lingxi_x2.backends.ros_environment import reexec_with_ros_environment
from lingxi_x2.client import X2Client
from lingxi_x2.config import load_config
from lingxi_x2.errors import SafetyInterlockError
from lingxi_x2.control_trace import ControlTrace


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--confirm-hardware", action="store_true")
    parser.add_argument("--read-only", action="store_true")
    parser.add_argument("--trace", type=Path, required=True)
    args = parser.parse_args()
    if not args.read_only and not args.confirm_hardware:
        raise SystemExit("Requires --confirm-hardware after operator confirmation")
    if args.trace.exists():
        raise SystemExit("Trace output already exists; choose a new path")
    reexec_with_ros_environment()
    config = load_config("config/x2.yaml")
    config.control.command_echo = True
    result = {"kind": "instrumented_joint_acceptance", "delta_rad": 0.01, "joint_index": 0}
    exit_code = 0
    with X2Client(config) as client, ControlTrace(client._backend) as trace:
        try:
            time.sleep(3.0)
            client._backend._wait_for_topic("/mc/upper_body_command", timeout_s=3.0)
            state = client.mc_state(timeout_s=3.0)
            report = client.control_preflight("arm")
            result["preflight"] = report.as_dict()
            if args.read_only:
                result["kind"] = "read_only_control_trace"
                return 0
            snapshot = trace.snapshot()
            if snapshot["errors"] or any(not snapshot["samples"][kind] for kind in ("hal_arm", "arm_state", "mc")):
                raise SafetyInterlockError("Diagnostic subscriptions are not ready")
            failures = [check for check in report.checks
                        if not check.passed and check.name not in {"control_enabled", "mc_mode"}]
            if failures or (state.action, state.action_status, state.fsm_state, state.body_state) != (
                    "STAND_DEFAULT", 100, 4, 1):
                raise SafetyInterlockError(f"Standing preconditions failed: {failures}; {state}")
            # Authorization exists only in this process, after fresh read-only checks.
            config.control.enabled = True
            result["session"] = client.test_upper_body_joint_session(
                0, 0.01, settle_duration_s=1.0, move_duration_s=1.0,
                target_hold_s=1.0, recovery_duration_s=1.0, recovery_hold_s=1.0,
                stop_observation_s=0.5, confirm_hardware=True,
            )
        except Exception as exc:
            exit_code = 2
            result["error"] = str(exc)
            result["last_publication_stats"] = client.publication_stats()
        finally:
            config.control.enabled = False
            try:
                result["final_mc"] = asdict(client.mc_state())
                result["final_arm"] = asdict(client.arm_state())
                result["final_hands"] = {side.value: asdict(hand) for side, hand in client.hand_states().items()}
            except Exception as exc:
                result["final_feedback_error"] = str(exc)
            time.sleep(0.3)
            snapshot = trace.snapshot()
            snapshot["result"] = result
            args.trace.parent.mkdir(parents=True, exist_ok=True)
            args.trace.write_text(json.dumps(snapshot), encoding="utf-8")
            print(json.dumps({"kind": "trace_saved", "path": str(args.trace),
                              "counts": {kind: len(rows) for kind, rows in snapshot["samples"].items()},
                              "errors": snapshot["errors"]}), flush=True)
    print(json.dumps(result, indent=2), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
