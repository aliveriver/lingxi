"""Mock-only execution. Deliberately no ROS/live compensation entry point."""
from __future__ import annotations

import json
import math
from pathlib import Path
import time

from .backends.mock import MockBackend
from .compensation import plan_compensated_joint_session
from .errors import SafetyInterlockError
from .models import ARM_JOINT_NAMES
from .publication import publish_points


def execute_mock_compensated_session(client, urdf: str | Path, request: dict,
                                     journal: str | Path) -> dict:
    # Check actual implementation AND advertised backend before planning or
    # opening a stream. config.backend='mock' alone cannot grant access.
    if not isinstance(client._backend, MockBackend) or client.status().backend != "mock":
        raise SafetyInterlockError("Compensated execution is mock-only; hardware acceptance is pending")
    plan = plan_compensated_joint_session(urdf, request)
    rate = plan["request"]["rate_hz"]
    if rate != client.config.control.publish_rate_hz:
        raise SafetyInterlockError("Plan rate must equal configured publication rate")
    frames = plan["frames"]
    baseline = frames[0]["command_rad"]
    step_limit = client.config.control.max_arm_step_rad
    if any(max(abs(a - b) for a, b in zip(previous["command_rad"], current["command_rad"])) > step_limit
           for previous, current in zip(frames, frames[1:])):
        raise SafetyInterlockError("Plan exceeds configured per-frame step limit")

    def feedback():
        state = client.arm_state()
        age = (time.monotonic_ns() - state.timestamp.monotonic_ns) / 1e9
        if not 0 <= age <= .2:
            raise SafetyInterlockError("Mock arm feedback is stale or from a future clock")
        if tuple(j.name for j in state.joints) != ARM_JOINT_NAMES:
            raise SafetyInterlockError("Expected ordered, complete arm feedback")
        q = tuple(j.position_rad for j in state.joints)
        # Mock reports unknown fault codes as None. This is not a live guard.
        if not all(math.isfinite(v) for v in q) or any(j.fault_code not in (None, 0) for j in state.joints):
            raise SafetyInterlockError("Invalid/faulted mock arm feedback")
        return state, q

    _, initial = feedback()
    if max(abs(a - b) for a, b in zip(initial, baseline)) > 1e-6:
        raise SafetyInterlockError("Mock pose differs from fixed baseline; initialize simulator explicitly")
    client._publication_frames = []
    # Exclusive creation prevents destroying earlier diagnostic evidence.
    with Path(journal).open("x", encoding="utf-8") as output:
        def record(value):
            output.write(json.dumps(value, allow_nan=False, ensure_ascii=False) + "\n")
            output.flush()

        record({"kind": "mock_compensation_session", "plan": {k: v for k, v in plan.items() if k != "frames"}})
        index = 0

        def publish(point):
            nonlocal index
            before, measured = feedback()
            expected_previous = baseline if index == 0 else frames[index - 1]["command_rad"]
            if max(abs(a - b) for a, b in zip(measured, expected_previous)) > 1e-6:
                raise SafetyInterlockError("Mock no longer matches previous command; fixed baseline retained")
            started = time.monotonic_ns()
            receipt = client._backend.publish_arm(point, (0.,) * 14, (0.,) * 14, 20., 2.)
            finished = time.monotonic_ns()
            record({"kind": "mock_command", **frames[index],
                    "feedback_before_rad": list(measured),
                    "feedback_monotonic_ns": before.timestamp.monotonic_ns,
                    "publish_monotonic_ns": started, "publish_return_monotonic_ns": finished})
            index += 1
            return receipt or {"monotonic_ns": started, "publish_return_monotonic_ns": finished,
                               "sequence": None, "measurement": "mock_backend_api_call"}

        try:
            with client._backend.command_stream("arm"):
                stats = publish_points(((f["phase"], tuple(f["command_rad"])) for f in frames),
                                       publish, rate, client._publication_frames)
            final_state, final = feedback()
            returned = max(abs(a - b) for a, b in zip(final, baseline)) <= 1e-6
            if not returned:
                raise SafetyInterlockError("Mock did not return to the fixed baseline")
            result = {"kind": "mock_compensation_result", "hardware_validated": False,
                      "hardware_execution_allowed": False, "mock_returned_to_baseline": returned,
                      "final_positions_rad": list(final),
                      "final_feedback_monotonic_ns": final_state.timestamp.monotonic_ns,
                      "publication": {k: v for k, v in stats.items() if k != "frames"},
                      "journal": str(journal), "plan_summary": plan["summary"]}
            record(result)
            return result
        except BaseException as exc:
            record({"kind": "mock_compensation_aborted", "error": str(exc),
                    "published_count": len(client._publication_frames),
                    "hardware_validated": False, "return_to_baseline_verified": False})
            raise
