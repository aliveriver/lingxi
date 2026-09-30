"""Bounded passive drive telemetry; raw fields, no inferred units or enums."""
from collections import deque
from contextlib import AbstractContextManager
import threading
import time
import math

from .errors import SafetyInterlockError


TOPICS = {
    "udcu": ("/aima/hal/monitor/udcu", "DcuMonitorData"),
    "joints": ("/aima/hal/joint/norealtime/udcu_joint/state", "JointNoRealTimeStateArray"),
}


class DriveTrace(AbstractContextManager):
    """Uses the existing ROS node, never a bus connection, publisher or service."""

    def __init__(self, backend, *, max_samples=20000, message_types=None, converter=None):
        if type(max_samples) is not int or not 1 <= max_samples <= 100000:
            raise ValueError("Drive trace capacity must be an integer in 1..100000 per stream")
        self.backend = backend
        self.types, self.converter = message_types, converter
        self.samples = {key: deque(maxlen=max_samples) for key in TOPICS}
        self.counts = dict.fromkeys(TOPICS, 0)
        self.dropped = dict.fromkeys(TOPICS, 0)
        self.errors = []
        self.error_count = 0
        self.lock = threading.Lock()
        self.subscriptions = []
        self.active = False
        self.unavailable_reason = None

    def __enter__(self):
        try:
            if self.types is None:
                from aimdk_msgs.msg import DcuMonitorData, JointNoRealTimeStateArray
                self.types = {"DcuMonitorData": DcuMonitorData, "JointNoRealTimeStateArray": JointNoRealTimeStateArray}
            if self.converter is None:
                from rosidl_runtime_py.convert import message_to_ordereddict
                self.converter = message_to_ordereddict
            self.active = True
            for key, (topic, typename) in TOPICS.items():
                self.subscriptions.append(self.backend._node.create_subscription(
                    self.types[typename], topic,
                    lambda message, selected=key: self.receive(selected, message), self.backend._qos))
        except Exception as exc:
            self.unavailable_reason = f"{type(exc).__name__}: {exc}"
            self.__exit__(None, None, None)
        return self

    def receive(self, key, message):
        try:
            mono = time.monotonic_ns()
            ros = self.backend._node.get_clock().now().nanoseconds
            raw = self.converter(message)
            if not isinstance(raw, dict):
                raise ValueError("Drive message converter did not return a mapping")
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            source = None if stamp is None else int(stamp.sec) * 10**9 + int(stamp.nanosec)
            row = {"received_monotonic_ns": mono, "received_ros_ns": ros,
                   "stamp_ns": source, "message": raw}
        except Exception as exc:
            with self.lock:
                if self.active:
                    self.error_count += 1
                    if len(self.errors) < 20:
                        self.errors.append(f"{key}: {type(exc).__name__}: {exc}")
            return
        with self.lock:
            if not self.active:
                return
            self.counts[key] += 1
            if len(self.samples[key]) == self.samples[key].maxlen:
                self.dropped[key] += 1
            self.samples[key].append(row)

    def __exit__(self, *args):
        with self.lock:
            self.active = False
        for sub in self.subscriptions:
            self.backend._node.destroy_subscription(sub)
        self.subscriptions.clear()

    def snapshot(self):
        with self.lock:
            return {"kind": "drive_trace_v1", "unavailable_reason": self.unavailable_reason,
                    "topics": {key: {"topic": topic, "type": typename} for key, (topic, typename) in TOPICS.items()},
                    "samples": {key: list(rows) for key, rows in self.samples.items()},
                    "counts": dict(self.counts), "dropped": dict(self.dropped),
                    "errors": list(self.errors), "error_count": self.error_count,
                    "units_verified": False, "control_mode_decoded": False, "current_limit_decoded": False,
                    "evidence": "Raw installed ROS fields; current scaling, mode/limit enums and effort meaning require vendor confirmation"}

    def require_recent(self, *, now_ns, max_age_s=1.):
        """Diagnostic data prerequisite, not a drive-state safety interpretation."""
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("Need a finite positive telemetry age bound")
        with self.lock:
            if (self.unavailable_reason or self.error_count or any(self.dropped.values())
                    or any(not rows or not 0 <= now_ns-rows[-1]["received_monotonic_ns"] <= max_age_s*1e9
                           for rows in self.samples.values())):
                raise SafetyInterlockError("Missing/stale or incomplete raw drive telemetry; physical diagnosis cannot begin")


def review_drive_window(trace, start_ns, stop_ns):
    """Describe receive-clock coverage, never interpret current or current limits."""
    if type(start_ns) is not int or type(stop_ns) is not int or not 0 < start_ns < stop_ns:
        raise ValueError("Need a positive ordered receive-clock window")
    problems, streams = [], {}
    if trace.get("unavailable_reason") or trace.get("error_count") or any(trace.get("dropped", {}).values()):
        problems.append("Drive telemetry unavailable, malformed or truncated")
    for key in TOPICS:
        rows = [r for r in trace.get("samples", {}).get(key, []) if start_ns <= r["received_monotonic_ns"] <= stop_ns]
        times = [r["received_monotonic_ns"] for r in rows]
        gaps = [b-a for a,b in zip([start_ns, *times], [*times, stop_ns])]
        if not rows or not all(b > a for a,b in zip(times, times[1:])):
            problems.append(f"{key}: no samples or unordered receive clocks")
        if max(gaps) > 1_000_000_000:
            problems.append(f"{key}: receive coverage gap exceeds the 1 s diagnostic freshness bound")
        sources = [r.get("stamp_ns") for r in rows]
        source_ordered = bool(sources) and all(type(t) is int and t > 0 for t in sources) and all(
            b > a for a,b in zip(sources, sources[1:]))
        if any(t is not None for t in sources) and not source_ordered:
            problems.append(f"{key}: invalid or unordered available source clocks")
        streams[key] = {"samples": len(rows), "max_receive_gap_s": max(gaps)/1e9,
                        "first_receive_age_s": (times[0]-start_ns)/1e9 if times else None,
                        "last_receive_age_s": (stop_ns-times[-1])/1e9 if times else None,
                        "source_clock_ordered": source_ordered}
    return {"kind": "drive_window_review_v1", "start_monotonic_ns": start_ns, "stop_monotonic_ns": stop_ns,
            "streams": streams, "problems": problems, "raw_samples_available": not problems,
            "drive_cause_identified": False,
            "evidence": "Receive-clock coverage only; no telemetry rate, physical units or drive state inferred"}
