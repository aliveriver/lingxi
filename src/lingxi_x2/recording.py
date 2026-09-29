from __future__ import annotations

from dataclasses import asdict
import base64
import json
from pathlib import Path
import time
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
    ) -> int:
        if duration_s <= 0:
            raise ValueError("duration_s must be positive")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        end = time.monotonic() + duration_s
        count = 0
        with self.output.open("x", encoding="utf-8") as stream:
            metadata = {
                "record_type": "metadata",
                "format": "lingxi-x2-jsonl-v1",
                "created_unix_ns": time.time_ns(),
                "status": self.client.status().as_dict(),
            }
            stream.write(json.dumps(metadata, ensure_ascii=False) + "\n")
            stream.flush()
            for observation in self.client.stream_observations(rate_hz, cameras, include_tactile):
                stream.write(json.dumps(observation_to_dict(observation, include_image_data), ensure_ascii=False) + "\n")
                stream.flush()
                count += 1
                if time.monotonic() >= end:
                    break
        return count

