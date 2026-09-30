"""Typed, streaming observation replay. This module has no robot client or writer."""
from __future__ import annotations

import base64
import binascii
from collections.abc import Iterator
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

from pydantic import TypeAdapter

from .models import ARM_JOINT_NAMES, CameraName, Observation


class RecordingError(ValueError):
    """A recording is malformed or incomplete; never substitute fabricated data."""


def strict_json(text: str) -> Any:
    def reject(value):
        raise RecordingError(f"Nonfinite JSON constant: {value}")

    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise RecordingError(f"Duplicate JSON key: {key}")
            result[key] = value
        return result

    result = json.loads(text, parse_constant=reject, object_pairs_hook=unique)
    def finite(value):
        if isinstance(value, float) and not math.isfinite(value):
            raise RecordingError("Nonfinite JSON number")
        if isinstance(value, dict):
            for child in value.values(): finite(child)
        elif isinstance(value, list):
            for child in value: finite(child)
    finite(result)
    return result


@dataclass(frozen=True)
class RecordedSample:
    index: int
    observation: Observation
    omitted_images: tuple[CameraName, ...]
    camera_metadata: dict[str, dict[str, Any]]


def decode_observation(payload: dict, index: int = 0, *, require_images: bool = False) -> RecordedSample:
    if not isinstance(payload, dict) or set(payload) != {"captured_monotonic_ns", "arm", "hands", "tactile", "cameras"}:
        raise RecordingError("Unexpected observation fields")
    captured = payload["captured_monotonic_ns"]
    if type(captured) is not int or captured < 0:
        raise RecordingError("Invalid captured_monotonic_ns")
    # Validate original raw types before Pydantic can coerce bool/string/float to int.
    from .tactile_quality import validate_frame
    if not isinstance(payload['tactile'], dict) or not set(payload['tactile']) <= {'left', 'right'}:
        raise RecordingError('Invalid tactile mapping')
    for side, frame in payload['tactile'].items():
        try:
            validate_frame(frame, side)
        except ValueError as exc:
            raise RecordingError(str(exc)) from exc
    # Keep source payloads untouched; omitted images must not become empty fake frames.
    data = dict(payload)
    cameras, omitted, camera_metadata = {}, [], {}
    for name, raw in payload["cameras"].items():
        camera = CameraName(name)
        item = dict(raw)
        if item.get("camera") != name:
            raise RecordingError("Camera key/name mismatch")
        size = item.pop("size_bytes")
        if type(size) is not int or size < 0:
            raise RecordingError("Invalid camera size_bytes")
        camera_metadata[name] = {k: v for k, v in raw.items() if k != "data_base64"}
        if "data_base64" not in item:
            omitted.append(camera)
            # Validate metadata with a temporary frame, never expose it to a policy.
            item["data"] = b""
        else:
            try:
                item["data"] = base64.b64decode(item.pop("data_base64"), validate=True)
            except (ValueError, TypeError, binascii.Error) as exc:
                raise RecordingError("Invalid image base64") from exc
            if len(item["data"]) != size:
                raise RecordingError("Camera byte length mismatch")
        cameras[camera] = item
    data["cameras"] = cameras
    try:
        observation = TypeAdapter(Observation).validate_python(data)
    except (ValueError, TypeError) as exc:
        raise RecordingError(f"Invalid observation: {exc}") from exc
    stamps = []
    if observation.arm is not None:
        if tuple(j.name for j in observation.arm.joints) != ARM_JOINT_NAMES:
            raise RecordingError("Arm joints must have the complete canonical 14-axis order")
        stamps.append(observation.arm.timestamp)
    for side, hand in observation.hands.items():
        if hand.side != side or len(hand.joints) != 10:
            raise RecordingError("Hand side/axis count mismatch")
        stamps.append(hand.timestamp)
    joint_groups = ([observation.arm.joints] if observation.arm is not None else []) + [h.joints for h in observation.hands.values()]
    for group in joint_groups:
        if any(not math.isfinite(v) for j in group for v in (j.position_rad, j.velocity_rad_s, j.effort_nm)):
            raise RecordingError("Nonfinite joint feedback")
    for side, frame in observation.tactile.items():
        if frame.side != side or frame.unit != "raw_uint8":
            raise RecordingError("Invalid tactile side or units")
        stamps.append(frame.timestamp)
        for surface in (frame.palm, frame.back_of_hand, *frame.fingertips.values()):
            if (len(surface.shape) != 2 or any(v <= 0 for v in surface.shape)
                    or math.prod(surface.shape) != len(surface.values)
                    or any(not 0 <= v <= 255 for v in surface.values)):
                raise RecordingError("Invalid raw tactile dimensions/values")
    for frame in observation.cameras.values():
        if frame.width <= 0 or frame.height <= 0:
            raise RecordingError("Invalid camera dimensions")
        stamps.append(frame.timestamp)
    for stamp in stamps:
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 10**9 or not 0 <= stamp.monotonic_ns <= captured:
            raise RecordingError("Invalid sensor timestamp or receipt later than capture")
    if omitted and require_images:
        raise RecordingError(f"Recording omitted image data: {', '.join(str(c) for c in omitted)}")
    observation = Observation(observation.captured_monotonic_ns, observation.arm, observation.hands,
                              observation.tactile, {k: v for k, v in observation.cameras.items() if k not in omitted})
    return RecordedSample(index, observation, tuple(omitted), camera_metadata)


class RecordingReader:
    """Bounded per-line memory; strict v1 input, original timestamps preserved.

    No trailing corrupt frame is silently discarded. A truncated file remains
    evidence of an interrupted recording and must be repaired into a separate file.
    """

    def __init__(self, path: str | Path, *, require_images: bool = False, max_line_bytes: int = 32 * 1024 * 1024):
        self.path = Path(path)
        self.require_images = require_images
        if type(max_line_bytes) is not int or max_line_bytes < 1024:
            raise ValueError("max_line_bytes must be an integer >= 1024")
        self.max_line_bytes = max_line_bytes
        self.metadata: dict = {}

    def __iter__(self) -> Iterator[RecordedSample]:
        previous = None
        with self.path.open("rb") as stream:
            line_number = 0
            while True:
                raw = stream.readline(self.max_line_bytes + 1)
                if not raw:
                    if line_number == 0:
                        raise RecordingError("Empty recording")
                    break
                line_number += 1
                try:
                    if len(raw) > self.max_line_bytes:
                        raise RecordingError("Recording line exceeds configured byte limit")
                    if not raw.endswith(b"\n"):
                        raise RecordingError("Unterminated recording line; file may be truncated")
                    payload = strict_json(raw.decode("utf-8"))
                    if line_number == 1:
                        if (not isinstance(payload, dict) or payload.get("record_type") != "metadata"
                                or payload.get("format") != "lingxi-x2-jsonl-v1"
                                or not isinstance(payload.get("status"), dict)):
                            raise RecordingError("Missing/unsupported recording metadata")
                        self.metadata = payload
                        continue
                    sample = decode_observation(payload, line_number - 2, require_images=self.require_images)
                    captured = sample.observation.captured_monotonic_ns
                    if previous is not None and captured <= previous:
                        raise RecordingError("Capture timestamps must strictly increase within an episode")
                    previous = captured
                    yield sample
                except (ValueError, TypeError, KeyError, AttributeError, RecursionError) as exc:
                    raise RecordingError(f"{self.path}: line {line_number}: {exc}") from exc

    def playback(self, speed: float = 1.) -> Iterator[RecordedSample]:
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("Replay speed must be finite and positive")
        previous = None
        for sample in self:
            stamp = sample.observation.captured_monotonic_ns
            if previous is not None:
                time.sleep((stamp - previous) / 1e9 / speed)
            yield sample
            # Relative pacing deliberately slows for expensive consumers, without
            # catch-up bursts. Source timestamps are never rewritten to 'now'.
            previous = stamp


def inspect_recording(path: str | Path) -> dict:
    from .tactile_quality import review_tactile
    from dataclasses import asdict
    reader = RecordingReader(path)
    count, first, last, omitted = 0, None, None, set()
    tactile_peak, tactile_nonzero = 0, 0
    quality_counts = {side: {k: 0 for k in ('fresh', 'stale', 'missing', 'invalid', 'time_error')}
                      for side in ('left', 'right')}
    repeated = dict.fromkeys(('left', 'right'), 0)
    previous_tactile = {}
    ceiling_cells = 0
    for sample in reader:
        stamp = sample.observation.captured_monotonic_ns
        if first is None: first = stamp
        last = stamp
        count += 1
        omitted.update(c.value for c in sample.omitted_images)
        quality = review_tactile({s.value: asdict(f) for s, f in sample.observation.tactile.items()}, stamp)
        for side, row in quality['sides'].items():
            quality_counts[side][row['status']] += 1
            ceiling_cells += row['raw_ceiling_cells'] or 0
        for side, frame in sample.observation.tactile.items():
            receipt = frame.timestamp.monotonic_ns
            if previous_tactile.get(side) == receipt: repeated[side.value] += 1
            previous_tactile[side] = receipt
        for frame in sample.observation.tactile.values():
            for surface in (frame.palm, frame.back_of_hand, *frame.fingertips.values()):
                tactile_peak = max(tactile_peak, max(surface.values, default=0))
                tactile_nonzero += sum(value != 0 for value in surface.values)
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""): digest.update(block)
    return {"kind": "recording_inspection", "sha256": digest.hexdigest(), "samples": count,
            "duration_s": (last - first) / 1e9 if count > 1 else 0., "metadata": reader.metadata,
            "omitted_image_streams": sorted(omitted), "tactile_peak_raw_uint8": tactile_peak,
            "tactile_nonzero_cell_samples": tactile_nonzero,
            "tactile_quality_counts": quality_counts, "tactile_raw_ceiling_cell_samples": ceiling_cells,
            "tactile_repeated_receipt_samples": repeated,
            "tactile_quality_note": "Age at original capture; missing is not zero. Repeated receipt is not a sensor-drop count; 255 is a raw byte ceiling, not calibrated saturation.",
            "motion_verified": False, "tactile_contact_verified": False}
