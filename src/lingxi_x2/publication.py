"""Publication timing describes local calls, never proves MC consumption."""
from __future__ import annotations

import time
import math
from collections.abc import Callable, Iterable
from typing import Any

from .errors import SafetyInterlockError


def summarize_frames(frames: list[dict[str, Any]], rate_hz: float) -> dict[str, Any]:
    times = [frame["monotonic_ns"] for frame in frames]
    gaps = [(b - a) / 1e9 for a, b in zip(times, times[1:])]
    sequences = [frame.get("sequence") for frame in frames]
    known = bool(sequences) and all(value is not None for value in sequences)
    discontinuities = (
        sum(b != (a + 1) % (2**32) for a, b in zip(sequences, sequences[1:]))
        if known else None
    )
    return {
        "configured_rate_hz": rate_hz,
        "published_count": len(frames),
        "actual_hz": (len(gaps) / sum(gaps)) if gaps and sum(gaps) > 0 else None,
        "max_frame_interval_s": max(gaps) if gaps else None,
        "min_frame_interval_s": min(gaps) if gaps else None,
        "intervals_over_1_5_periods": sum(gap > 1.5 / rate_hz for gap in gaps),
        "sequence_discontinuities": discontinuities,
        "sequence_continuous": discontinuities == 0 if known else None,
        "evidence": "local publish calls; not MC receipt or motion verification",
        "frames": frames,
    }


def publish_points(
    points: Iterable[tuple[str, tuple[float, ...]]],
    publish: Callable[[tuple[float, ...]], Any],
    rate_hz: float,
    frames: list[dict[str, Any]],
    *,
    max_frame_interval_s: float | None = None,
    before_publish: Callable[[tuple[float, ...]], None] | None = None,
) -> dict[str, Any]:
    """Pace using monotonic time, resetting missed deadlines without catch-up bursts.

    Caller owns frames so successfully published evidence survives exceptions.
    ROS receipts timestamp the actual publisher call; other backends measure API calls.
    """
    if not math.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError("Publication rate must be finite and positive")
    if max_frame_interval_s is not None and (
        not math.isfinite(max_frame_interval_s) or max_frame_interval_s <= 0
        or max_frame_interval_s < 1.0 / rate_hz
    ):
        raise ValueError("Maximum interval must be finite and at least one period")
    period = 1.0 / rate_hz
    deadline = time.monotonic()
    def check_gap():
        if max_frame_interval_s is not None and frames:
            gap = time.monotonic_ns() - frames[-1]["monotonic_ns"]
            if gap < 0 or gap > max_frame_interval_s * 1e9:
                raise SafetyInterlockError("Publication interval exceeds configured maximum or clock regressed")
    for phase, point in points:
        time.sleep(max(0.0, deadline - time.monotonic()))
        check_gap()
        if before_publish is not None:
            before_publish(point)
        # Include validation latency; a stale check must not release another frame.
        check_gap()
        started = time.monotonic_ns()
        receipt = publish(point)
        finished = time.monotonic_ns()
        frame = dict(receipt or {
            "monotonic_ns": started,
            "publish_return_monotonic_ns": finished,
            "sequence": None,
            "measurement": "backend_api_call",
        })
        frame["phase"] = phase
        frames.append(frame)
        # Preserve the completed send as evidence before reporting an overrun,
        # including an overrun on the final frame. A blocking transport cannot
        # be interrupted by this synchronous software guard.
        check_gap()
        # Anchor to the actual call start, so a late wake-up cannot shorten the
        # next interval. If publishing itself overruns, allow a full new period.
        deadline = max(frame["monotonic_ns"] / 1e9 + period, finished / 1e9)
        if finished - started >= period * 1e9:
            deadline = finished / 1e9 + period
    # Retain the final target for one period before the caller ends the phase.
    time.sleep(max(0.0, deadline - time.monotonic()))
    return summarize_frames(frames, rate_hz)
