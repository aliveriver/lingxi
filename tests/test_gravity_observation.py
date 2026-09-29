from copy import deepcopy
import json
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from lingxi_x2.cli import main
from lingxi_x2.gravity_capture import GravityCapture, gravity_sample, capture_readonly
from lingxi_x2.gravity_observation import analyze_gravity_observation, WAIST_NAMES
from lingxi_x2.models import ARM_JOINT_NAMES


def snapshot():
    common = {"stamp_ns": 9_990_000_000, "received_ros_ns": 9_995_000_000,
              "received_monotonic_ns": 995_000_000}
    def joints(names):
        return {**common, "joints": [{"name": n, "position": 0., "velocity": 0., "error_code": 0} for n in names]}
    imu = {**common, "frame_id": "imu_frame", "orientation_xyzw": [0., 0., 0., 1.],
           "orientation_covariance": [0.] * 9}
    return {"kind": "gravity_readonly_snapshot", "schema_version": 1,
            "captured_monotonic_ns": 1_000_000_000, "captured_ros_ns": 10_000_000_000,
            "errors": [], "error_count": 0, "samples": {
                "chest_imu": deepcopy(imu), "pelvis_imu": deepcopy(imu),
                "arm_state": joints(ARM_JOINT_NAMES), "waist_state": joints(WAIST_NAMES)}}


def mounting(body="torso_link"):
    return {"imu_key": "chest_imu" if body == "torso_link" else "pelvis_imu", "body": body,
            "expected_frame_id": "imu_frame", "rotation_body_from_imu": np.eye(3).tolist()}


@pytest.fixture
def waist_urdf(tmp_path):
    path = tmp_path / "waist.urdf"
    children = ["waist_yaw_link", "waist_pitch_link", "torso_link"]
    parents = ["pelvis", *children[:-1]]
    axes = ["0 0 1", "0 1 0", "1 0 0"]
    path.write_text('<robot name="synthetic">' + ''.join(f'<link name="{n}"/>' for n in ["pelvis", *children]) + ''.join(
        f'<joint name="{n}" type="revolute"><parent link="{p}"/><child link="{c}"/>'
        f'<axis xyz="{a}"/><limit lower="-2" upper="2"/></joint>'
        for n, p, c, a in zip(WAIST_NAMES, parents, children, axes)) + '</robot>')
    return path


def test_torso_orientation_and_mounting_direction():
    data = snapshot()
    angle = .3
    data["samples"]["chest_imu"]["orientation_xyzw"] = [0., math.sin(angle/2), 0., math.cos(angle/2)]
    result = analyze_gravity_observation(data, mounting())
    np.testing.assert_allclose(result["gravity_torso_m_s2"], [9.81*math.sin(angle), 0., -9.81*math.cos(angle)], atol=1e-12)
    assert result["usable_for_offline_estimate"] and not result["hardware_validated"]
    # IMU yaw installation is a basis change; ensure we use R_body_imu,
    # not its inverse. A +90deg mount rotates IMU +X gravity into body +Y.
    mount = mounting()
    mount["rotation_body_from_imu"] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    result = analyze_gravity_observation(data, mount)
    np.testing.assert_allclose(result["gravity_torso_m_s2"], [0., 9.81*math.sin(angle), -9.81*math.cos(angle)], atol=1e-12)


def test_pelvis_waist_rotation(waist_urdf):
    data = snapshot()
    data["samples"]["waist_state"]["joints"][1]["position"] = .4
    result = analyze_gravity_observation(data, mounting("pelvis"), urdf=waist_urdf)
    assert result["usable_for_offline_estimate"]
    np.testing.assert_allclose(result["gravity_torso_m_s2"], [9.81*math.sin(.4), 0., -9.81*math.cos(.4)])
    assert len(result["waist_model_sha256"]) == 64


@pytest.mark.parametrize("issue", ["stale_receive", "stale_source", "future", "missing_arm", "skew", "clock_jump",
                                   "zero_quat", "nan_quat", "no_orientation", "frame", "duplicate_joint",
                                   "fault", "nan_velocity", "reflection", "error_count", "zero_stamp"])
def test_invalid_observation_never_returns_gravity(issue):
    data, mount = snapshot(), mounting()
    imu, arm = data["samples"]["chest_imu"], data["samples"]["arm_state"]
    if issue == "stale_receive": imu["received_monotonic_ns"] -= 300_000_000
    elif issue == "stale_source": imu["stamp_ns"] -= 300_000_000
    elif issue == "future": imu["stamp_ns"] += 20_000_000
    elif issue == "missing_arm": del data["samples"]["arm_state"]
    elif issue == "skew": imu["stamp_ns"] -= 60_000_000
    elif issue == "clock_jump": data["captured_ros_ns"] += 100_000_000
    elif issue == "zero_quat": imu["orientation_xyzw"] = [0] * 4
    elif issue == "nan_quat": imu["orientation_xyzw"][0] = float("nan")
    elif issue == "no_orientation": imu["orientation_covariance"][0] = -1
    elif issue == "frame": imu["frame_id"] = "different"
    elif issue == "duplicate_joint": arm["joints"].append(arm["joints"][0])
    elif issue == "fault": arm["joints"][0]["error_code"] = 1
    elif issue == "nan_velocity": arm["joints"][0]["velocity"] = float("nan")
    elif issue == "reflection": mount["rotation_body_from_imu"][0][0] = -1
    elif issue == "error_count": data["error_count"] = 1
    elif issue == "zero_stamp": imu["stamp_ns"] = 0
    result = analyze_gravity_observation(data, mount)
    assert not result["usable_for_offline_estimate"] and result["errors"]
    assert "gravity_torso_m_s2" not in result


@pytest.mark.parametrize("issue", ["missing", "incomplete", "out_of_limit", "bad_chain", "no_urdf", "stale"])
def test_waist_is_required_and_validated(waist_urdf, issue):
    data = snapshot()
    if issue == "missing": del data["samples"]["waist_state"]
    elif issue == "incomplete": data["samples"]["waist_state"]["joints"].pop()
    elif issue == "out_of_limit": data["samples"]["waist_state"]["joints"][0]["position"] = 3.
    elif issue == "bad_chain": waist_urdf.write_text(waist_urdf.read_text().replace('link="pelvis"', 'link="wrong"'))
    elif issue == "stale": data["samples"]["waist_state"]["stamp_ns"] -= 300_000_000
    result = analyze_gravity_observation(data, mounting("pelvis"), urdf=None if issue == "no_urdf" else waist_urdf)
    assert not result["usable_for_offline_estimate"]
    assert "gravity_torso_m_s2" not in result


def imu_message(stamp=1):
    return NS(header=NS(stamp=NS(sec=stamp, nanosec=0), frame_id="imu"),
              orientation=NS(x=0., y=0., z=0., w=1.), orientation_covariance=[0.] * 9,
              linear_acceleration=NS(x=0., y=0., z=9.81))


def test_capture_invalidates_stale_and_bounds_errors():
    c = GravityCapture()
    c.receive("chest_imu", imu_message(), received_monotonic_ns=10, received_ros_ns=1_000_000_010)
    c.receive("chest_imu", imu_message(), received_monotonic_ns=20, received_ros_ns=1_000_000_020)
    assert "chest_imu" not in c.samples
    for _ in range(30):
        c.receive("chest_imu", NS(), received_monotonic_ns=30, received_ros_ns=1_000_000_030)
    report = c.snapshot(captured_monotonic_ns=40, captured_ros_ns=1_000_000_040)
    assert report["error_count"] == 31 and len(report["errors"]) == 20
    assert report["counts"]["chest_imu"] == 32


def test_capture_raw_fields_and_snapshot_copy():
    c = GravityCapture()
    c.receive("pelvis_imu", imu_message(), received_monotonic_ns=10, received_ros_ns=1_000_000_010)
    snap = c.snapshot(captured_monotonic_ns=20, captured_ros_ns=1_000_000_020)
    snap["samples"]["pelvis_imu"]["orientation_xyzw"][3] = 0
    assert c.samples["pelvis_imu"]["orientation_xyzw"][3] == 1
    assert c.samples["pelvis_imu"]["linear_acceleration_m_s2"] == [0, 0, 9.81]
    assert c.samples["pelvis_imu"]["stamp_ns"] == 1_000_000_000


def test_nonfinite_telemetry_and_invalid_duration():
    message = imu_message()
    message.orientation.x = float("nan")
    with pytest.raises(ValueError):
        gravity_sample("chest_imu", message, 1, 1)
    with pytest.raises(ValueError):
        capture_readonly(float("nan"))  # validation precedes ROS imports


def test_offline_cli_never_connects(tmp_path, monkeypatch, capsys):
    def forbidden(*args, **kwargs): pytest.fail("offline command initialized ROS/client")
    monkeypatch.setattr("lingxi_x2.cli.X2Client", forbidden)
    monkeypatch.setattr("lingxi_x2.cli.reexec_with_ros_environment", forbidden)
    data, mount = tmp_path / "data.json", tmp_path / "mount.json"
    data.write_text(json.dumps(snapshot())); mount.write_text(json.dumps(mounting()))
    assert main(["analyze-gravity-state", str(data), "--mounting", str(mount)]) == 0
    assert json.loads(capsys.readouterr().out)["usable_for_offline_estimate"]
    data.write_text('{}')
    assert main(["analyze-gravity-state", str(data), "--mounting", str(mount)]) == 2
