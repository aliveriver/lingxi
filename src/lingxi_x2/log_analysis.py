"""Read acceptance JSON embedded between DDS stdout diagnostics."""
from __future__ import annotations

import json
from typing import Any


def read_session(text: str) -> dict[str, Any]:
    decoder = json.JSONDecoder()
    for index, char in enumerate(text):
        if char != "{":
            continue
        try:
            value, _ = decoder.raw_decode(text[index:])
        except ValueError:
            continue
        if isinstance(value, dict) and "kind" in value:
            return value
    raise ValueError("No acceptance session JSON object found")


def analyze_session(session: dict[str, Any]) -> dict[str, Any]:
    sections: dict[str, Any] = {}

    def walk(value: Any, path: str) -> None:
        if not isinstance(value, dict):
            return
        result: dict[str, Any] = {}
        if "started_monotonic_ns" in value and "finished_monotonic_ns" in value:
            result["elapsed_s"] = (value["finished_monotonic_ns"] - value["started_monotonic_ns"]) / 1e9
        events = value.get("events", [])
        if events:
            result["phase_intervals"] = [
                {"from": a["phase"], "to": b["phase"], "elapsed_s": (b["monotonic_ns"] - a["monotonic_ns"]) / 1e9}
                for a, b in zip(events, events[1:])
            ]
        if "target_feedback_rad" in value:
            i = value["joint_index"]
            result["selected_joint_delta_rad"] = value["target_feedback_rad"][i] - value["baseline_positions_rad"][i]
            result["selected_joint_error_rad"] = value["target_feedback_rad"][i] - value["target_positions_rad"][i]
        if "published_count" in value:
            result.update({k: v for k, v in value.items() if k not in {"frames", "last_command", "last_local_echo"}})
        if result:
            sections[path] = result
        for key, child in value.items():
            if key not in {"events", "frames"}:
                walk(child, f"{path}.{key}")

    walk(session, "session")
    return {"kind": session["kind"], "sections": sections,
            "note": "Event intervals include preflight and feedback reads; absent per-frame timestamps cannot establish Hz or jitter."}
