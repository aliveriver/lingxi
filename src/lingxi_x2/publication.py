"""Publication timing describes local calls, never proves MC consumption."""
from __future__ import annotations

import time
from collections.abc import Callable, Iterable
from typing import Any


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
) -> dict[str, Any]:
    """Pace using monotonic time, resetting missed deadlines without catch-up bursts.

    Caller owns frames so successfully published evidence survives exceptions.
    ROS receipts timestamp the actual publisher call; other backends measure API calls.
    """
    period = 1.0 / rate_hz
    deadline = time.monotonic()
    for phase, point in points:
        time.sleep(max(0.0, deadline - time.monotonic()))
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
        # Anchor to the actual call start, so a late wake-up cannot shorten the
        # next interval. If publishing itself overruns, allow a full new period.
        deadline = max(frame["monotonic_ns"] / 1e9 + period, finished / 1e9)
        if finished - started >= period * 1e9:
            deadline = finished / 1e9 + period
    # Retain the final target for one period before the caller ends the phase.
    time.sleep(max(0.0, deadline - time.monotonic()))
    return summarize_frames(frames, rate_hz)
