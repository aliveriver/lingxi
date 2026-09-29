from dataclasses import replace
import json

from PIL import Image
import pytest

from lingxi_x2 import X2Client
from lingxi_x2.backends.ros2 import Ros2Backend
from lingxi_x2.config import PlatformConfig
from lingxi_x2.errors import DataTimeoutError
from lingxi_x2.tactile import save_tactile_map


def test_export_preserves_raw_values_and_rejects_stale_ros_data(tmp_path):
    with X2Client(PlatformConfig(backend="mock")) as client:
        frames = client.tactile_frames()
    result = save_tactile_map(frames, tmp_path / "touch.png")
    assert Image.open(result["image"]).size == (1120, 420)
    metadata = json.loads((tmp_path / "touch.json").read_text())
    assert metadata["units"] == "raw_uint8"
    assert metadata["physical_orientation_verified"] is False
    for side, frame in frames.items():
        assert metadata["frames"][side.value]["palm"]["values"] == list(frame.palm.values)
    backend = Ros2Backend.__new__(Ros2Backend)
    backend.config = PlatformConfig()
    import threading
    backend._condition = threading.Condition()
    backend._message_supports_tactile = lambda: True
    backend._tactile = {side: replace(frame, timestamp=replace(frame.timestamp, monotonic_ns=0)) for side, frame in frames.items()}
    with pytest.raises(DataTimeoutError, match="stale"):
        backend.tactile_frames()
