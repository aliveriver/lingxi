from __future__ import annotations

import json
from pathlib import Path

from lingxi_x2 import X2Client
from lingxi_x2.config import PlatformConfig
from lingxi_x2.models import CameraName
from lingxi_x2.recording import JsonlRecorder


def test_jsonl_recorder_writes_metadata_and_samples(tmp_path: Path) -> None:
    output = tmp_path / "episode.jsonl"
    with X2Client(PlatformConfig(backend="mock")) as client:
        count = JsonlRecorder(client, output).run(
            duration_s=0.03,
            rate_hz=100,
            cameras=(CameraName.RGBD_FRONT_RGB,),
            include_tactile=True,
            include_image_data=False,
        )
    lines = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
    assert count >= 1
    assert lines[0]["format"] == "lingxi-x2-jsonl-v1"
    assert lines[1]["cameras"]["rgbd_front_rgb"]["size_bytes"] > 0
    assert "data_base64" not in lines[1]["cameras"]["rgbd_front_rgb"]


def test_recorder_refuses_overwrite(tmp_path: Path) -> None:
    output = tmp_path / "episode.jsonl"
    output.write_text("existing", encoding="utf-8")
    with X2Client(PlatformConfig(backend="mock")) as client:
        try:
            JsonlRecorder(client, output).run(0.01, 10)
        except FileExistsError:
            pass
        else:
            raise AssertionError("recorder overwrote an existing episode")

