from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path

import pytest

from lingxi_x2 import X2Client, PlatformConfig
from lingxi_x2.cli import main
from lingxi_x2.models import CameraName
from lingxi_x2.recording import observation_to_dict, JsonlRecorder
from lingxi_x2.replay import RecordingReader, RecordingError, decode_observation, inspect_recording


@pytest.fixture
def observation():
    with X2Client(PlatformConfig(backend="mock")) as client:
        return client.observe((CameraName.RGBD_FRONT_RGB,), True)


def write_recording(path, payloads):
    metadata = {"record_type": "metadata", "format": "lingxi-x2-jsonl-v1", "status": {"backend": "mock"}}
    path.write_text('\n'.join(json.dumps(v) for v in [metadata, *payloads]) + '\n')


def test_roundtrip_preserves_bytes_clocks_and_tactile(observation, tmp_path):
    path = tmp_path / "recording.jsonl"
    write_recording(path, [observation_to_dict(observation)])
    sample, = RecordingReader(path)
    assert sample.observation == observation
    assert not sample.omitted_images
    report = inspect_recording(path)
    assert report["samples"] == 1 and len(report["sha256"]) == 64
    assert report["tactile_peak_raw_uint8"] > 0 and not report["tactile_contact_verified"]


def test_omitted_images_not_fabricated(observation, tmp_path):
    path = tmp_path / "recording.jsonl"
    write_recording(path, [observation_to_dict(observation, False)])
    sample, = RecordingReader(path)
    assert not sample.observation.cameras
    assert sample.omitted_images == (CameraName.RGBD_FRONT_RGB,)
    assert sample.camera_metadata["rgbd_front_rgb"]["size_bytes"] > 0
    with pytest.raises(RecordingError, match="omitted image"):
        list(RecordingReader(path, require_images=True))


@pytest.mark.parametrize("issue", ["missing_axis", "wrong_order", "side", "tactile_size", "tactile_range",
                                   "base64", "image_size", "camera_name", "future_stamp", "nan"])
def test_bad_frames_rejected(observation, tmp_path, issue):
    data = observation_to_dict(observation)
    if issue == "missing_axis": data["arm"]["joints"] = data["arm"]["joints"][:-1]
    elif issue == "wrong_order": data["arm"]["joints"] = data["arm"]["joints"][::-1]
    elif issue == "side": data["hands"]["left"]["side"] = "right"
    elif issue == "tactile_size": data["tactile"]["left"]["palm"]["shape"] = [1, 1]
    elif issue == "tactile_range": data["tactile"]["left"]["palm"]["values"] = [999] * 25
    elif issue == "base64": data["cameras"]["rgbd_front_rgb"]["data_base64"] = "bad!"
    elif issue == "image_size": data["cameras"]["rgbd_front_rgb"]["size_bytes"] += 1
    elif issue == "camera_name": data["cameras"]["rgbd_front_rgb"]["camera"] = "head_rear"
    elif issue == "future_stamp": data["arm"]["timestamp"]["monotonic_ns"] = data["captured_monotonic_ns"] + 1
    elif issue == "nan": data["arm"]["joints"][0]["position_rad"] = float("nan")
    path = tmp_path / "bad.jsonl"; write_recording(path, [data])
    with pytest.raises(RecordingError, match="line 2"):
        list(RecordingReader(path))


def test_truncation_duplicate_keys_and_timestamp_reversal(observation, tmp_path):
    path = tmp_path / "bad.jsonl"
    data = observation_to_dict(observation)
    write_recording(path, [data, data])
    with pytest.raises(RecordingError, match="strictly increase"): list(RecordingReader(path))
    write_recording(path, [data]); path.write_bytes(path.read_bytes()[:-8])
    with pytest.raises(RecordingError, match="truncated"): list(RecordingReader(path))
    path.write_text('{"record_type":"metadata","record_type":"metadata"}\n')
    with pytest.raises(RecordingError, match="Duplicate"): list(RecordingReader(path))


def test_pacing_preserves_source_time(observation, tmp_path, monkeypatch):
    path = tmp_path / "episode.jsonl"
    second = replace(observation, captured_monotonic_ns=observation.captured_monotonic_ns + 200_000_000)
    write_recording(path, [observation_to_dict(observation), observation_to_dict(second)])
    sleeps = []
    monkeypatch.setattr("lingxi_x2.replay.time.sleep", sleeps.append)
    samples = list(RecordingReader(path).playback(speed=2.))
    assert sleeps == [.1]
    assert samples[1].observation.captured_monotonic_ns == second.captured_monotonic_ns


def test_cli_offline_and_stream_bounds(observation, tmp_path, monkeypatch, capsys):
    path = tmp_path / "episode.jsonl"; write_recording(path, [observation_to_dict(observation)])
    def forbidden(*args, **kwargs): pytest.fail("Replay must not initialize a client or ROS")
    monkeypatch.setattr("lingxi_x2.cli.X2Client", forbidden)
    monkeypatch.setattr("lingxi_x2.cli.reexec_with_ros_environment", forbidden)
    assert main(["inspect-recording", str(path)]) == 0
    assert json.loads(capsys.readouterr().out)["samples"] == 1
    assert main(["replay", str(path)]) == 0
    assert json.loads(capsys.readouterr().out)["execution"] == "none"
    with pytest.raises(RecordingError, match="byte limit"):
        list(RecordingReader(path, max_line_bytes=1024))


@pytest.mark.parametrize("duration,rate", [(float("nan"), 10), (1, 0), (1, float("nan")), (float("inf"), 10)])
def test_invalid_recording_timing_leaves_no_file(tmp_path, duration, rate):
    path = tmp_path / "bad.jsonl"
    with pytest.raises(ValueError): JsonlRecorder(None, path).run(duration, rate)
    assert not path.exists()
