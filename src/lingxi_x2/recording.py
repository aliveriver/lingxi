from __future__ import annotations

from dataclasses import asdict
import base64
import json
import math
from pathlib import Path
import time
import threading
from typing import Any

from .client import X2Client
from .models import CameraName, Observation


def observation_to_dict(observation: Observation, include_image_data: bool = True) -> dict[str, Any]:
    payload = asdict(observation)
    payload["hands"] = {side.value: value for side, value in payload["hands"].items()}
    payload["tactile"] = {side.value: value for side, value in payload["tactile"].items()}
    cameras: dict[str, Any] = {}
    for camera, frame in observation.cameras.items():
        item = asdict(frame)
        data = item.pop("data")
        if include_image_data:
            item["data_base64"] = base64.b64encode(data).decode("ascii")
        item["size_bytes"] = len(data)
        cameras[camera.value] = item
    payload["cameras"] = cameras
    return payload


class JsonlRecorder:
    """Streaming, flush-on-each-sample recorder suitable for abrupt experiment stops."""

    def __init__(self, client: X2Client, output: str | Path):
        self.client = client
        self.output = Path(output)

    def run(
        self,
        duration_s: float,
        rate_hz: float,
        cameras: tuple[CameraName | str, ...] = (),
        include_tactile: bool = True,
        include_image_data: bool = True,
        stop_event: threading.Event | None = None,
    ) -> int:
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError("duration_s must be finite and positive")
        if not math.isfinite(rate_hz) or not 0 < rate_hz <= 200:
            raise ValueError("rate_hz must be finite and in (0, 200]")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        stop = stop_event or threading.Event()
        end = time.monotonic() + duration_s
        count = 0
        with self.output.open("x", encoding="utf-8") as stream:
            metadata = {
                "record_type": "metadata",
                "format": "lingxi-x2-jsonl-v1",
                "created_unix_ns": time.time_ns(),
                "status": self.client.status().as_dict(),
                "recording_options": {"rate_hz": rate_hz, "duration_s": duration_s,
                                      "cameras": [str(c) for c in cameras], "include_tactile": include_tactile,
                                      "include_image_data": include_image_data},
                "evidence": "observations only; not motion or tactile contact acceptance",
            }
            stream.write(json.dumps(metadata, ensure_ascii=False) + "\n")
            stream.flush()
            deadline = time.monotonic()
            while not stop.is_set() and time.monotonic() < end:
                observation = self.client.observe(cameras, include_tactile, timeout_s=1.0)
                if stop.is_set():
                    break
                stream.write(json.dumps(observation_to_dict(observation, include_image_data), ensure_ascii=False, allow_nan=False) + "\n")
                stream.flush()
                count += 1
                now = time.monotonic()
                deadline += 1.0 / rate_hz
                if deadline <= now:
                    deadline = now + 1.0 / rate_hz
                stop.wait(max(0.0, min(deadline, end) - now))
        return count
