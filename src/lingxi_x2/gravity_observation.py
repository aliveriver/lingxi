"""Strict offline checks for a recorded IMU/waist/arm gravity observation.

Timestamps are compared only within their clock domain. Mount transforms and
sensor body are explicit caller declarations, never inferred from topic names.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .gravity import StaticArmModel, _origin, _rotation, _vector
from .models import ARM_JOINT_NAMES

WAIST_NAMES = ("waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint")


def _integer(value, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{label} must be a positive integer")
    return value


def _rotation_matrix(value) -> np.ndarray:
    result = np.asarray(value, float)
    if (result.shape != (3, 3) or not np.isfinite(result).all()
            or not np.allclose(result.T @ result, np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(result), 1., atol=1e-8, rtol=0)):
        raise ValueError("rotation_body_from_imu must be a proper rotation matrix")
    return result


def _quaternion(value) -> np.ndarray:
    x, y, z, w = _vector(value, 4, "orientation_xyzw")
    norm = np.linalg.norm([x, y, z, w])
    if not .99 <= norm <= 1.01:
        raise ValueError("IMU quaternion norm outside 0.99–1.01")
    x, y, z, w = np.array([x, y, z, w]) / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _joint_positions(sample: dict, names: tuple[str, ...]) -> dict[str, float]:
    rows = sample["joints"]
    positions = {}
    for row in rows:
        name = row["name"]
        if name in positions:
            raise ValueError(f"Duplicate joint feedback: {name}")
        position, velocity = _vector([row["position"], row["velocity"]], 2, name)
        if row.get("error_code") != 0:
            raise ValueError(f"Nonzero or missing joint fault: {name}")
        positions[name] = float(position)
    if set(positions) != set(names):
        raise ValueError("Joint feedback must contain the exact complete named chain")
    return positions


def _waist_rotation(path: str | Path, positions: dict[str, float]) -> tuple[np.ndarray, str]:
    raw = Path(path).read_bytes()
    root = ET.fromstring(raw)
    if root.tag != "robot":
        raise ValueError("Expected URDF robot root")
    links = StaticArmModel._named(root, "link")
    if "pelvis" not in links:
        raise ValueError("Missing pelvis base link")
    joints = {}
    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        if name in joints:
            raise ValueError(f"Duplicate URDF joint: {name}")
        joints[name] = joint
    parent, rot = "pelvis", np.eye(3)
    children = []
    for name in WAIST_NAMES:
        joint = joints[name]
        if (joint.get("type") != "revolute" or joint.find("mimic") is not None
                or joint.find("parent").attrib["link"] != parent):
            raise ValueError(f"Unsupported pelvis-to-torso chain at {name}")
        _, origin_rot = _origin(joint)
        axis = _vector(joint.find("axis").attrib["xyz"].split(), 3, "waist axis")
        norm = np.linalg.norm(axis)
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError("Invalid waist axis")
        limit = joint.find("limit")
        low, high = _vector([limit.attrib["lower"], limit.attrib["upper"]], 2, "waist limits")
        if low >= high or not low <= positions[name] <= high:
            raise ValueError(f"Waist position/limits invalid: {name}={positions[name]:.9f} rad, "
                             f"model range=[{low:.9f}, {high:.9f}]")
        rot = rot @ origin_rot @ _rotation(axis / norm, positions[name])
        parent = joint.find("child").attrib["link"]
        if parent not in links:
            raise ValueError(f"Missing waist child link: {parent}")
        children.append(parent)
    if parent != "torso_link" or len(set(children)) != 3 or "pelvis" in children:
        raise ValueError("Expected acyclic three-axis waist chain ending at torso_link")
    for name, joint in joints.items():
        child = joint.find("child")
        if name not in WAIST_NAMES and child is not None and child.get("link") in children:
            raise ValueError("Multiple parents in waist chain")
    return rot, hashlib.sha256(raw).hexdigest()


def analyze_gravity_observation(snapshot: dict, mounting: dict, *, urdf: str | Path | None = None) -> dict:
    """Analyze one latest-sample snapshot; unavailable inputs yield no gravity.

    Fixed diagnostic gates: 200 ms age, 50 ms cross-stream skew. A result marked
    usable is only arithmetically usable at the recorded instant; mounting and
    actual hardware compatibility remain unverified. No accelerometer fallback.
    """
    result = {"kind": "offline_gravity_observation", "schema_version": 1,
              "hardware_validated": False, "command_output": False,
              "usable_for_offline_estimate": False, "errors": [],
              "thresholds": {"max_age_s": .2, "max_skew_s": .05},
              "limitations": ["Sensor body and mounting are caller declarations, not verified calibration.",
                              "IMU header stamps may be driver receipt times, not sensor acquisition times; transport delay is not measured here.",
                              "ROS clocks across producers must be synchronized; stamps alone cannot prove synchronization.",
                              "This is a recorded instant, not ongoing feedback freshness or motion authorization."]}
    try:
        if snapshot.get("kind") != "gravity_readonly_snapshot" or snapshot.get("schema_version") != 1:
            raise ValueError("Unsupported gravity snapshot schema")
        if snapshot.get("errors") or snapshot.get("error_count", 0):
            raise ValueError("Capture contains parsing errors")
        if set(mounting) != {"imu_key", "body", "expected_frame_id", "rotation_body_from_imu"}:
            raise ValueError("Explicit imu_key, body, expected_frame_id and rotation_body_from_imu required")
        key, body = mounting["imu_key"], mounting["body"]
        if key not in ("chest_imu", "pelvis_imu") or body not in ("torso_link", "pelvis"):
            raise ValueError("Unknown IMU key or declared body")
        frame = mounting["expected_frame_id"]
        if not isinstance(frame, str) or not frame.strip():
            raise ValueError("Nonempty expected_frame_id required")
        mount = _rotation_matrix(mounting["rotation_body_from_imu"])
        now_mono = _integer(snapshot["captured_monotonic_ns"], "capture monotonic time")
        now_ros = _integer(snapshot["captured_ros_ns"], "capture ROS time")
        selected = [key, "arm_state"] + (["waist_state"] if body == "pelvis" else [])
        samples = {kind: snapshot["samples"][kind] for kind in selected}
        times = {"receive": [], "source": []}
        timing = {}
        for kind, sample in samples.items():
            received = _integer(sample["received_monotonic_ns"], f"{kind} receive time")
            received_ros = _integer(sample["received_ros_ns"], f"{kind} receive ROS time")
            stamp = _integer(sample["stamp_ns"], f"{kind} source time")
            mono_age, ros_age = now_mono - received, now_ros - received_ros
            source_age = now_ros - stamp
            if not (0 <= mono_age <= 200_000_000 and 0 <= ros_age <= 200_000_000
                    and 0 <= source_age <= 200_000_000 and 0 <= received_ros - stamp <= 200_000_000):
                raise ValueError(f"{kind}: stale or future receive/source timestamp")
            if abs(mono_age - ros_age) > 50_000_000:
                raise ValueError(f"{kind}: ROS/monotonic elapsed time mismatch")
            times["receive"].append(received)
            times["source"].append(stamp)
            timing[kind] = {"receive_age_s": mono_age / 1e9, "source_age_s": source_age / 1e9}
        for domain, values in times.items():
            if max(values) - min(values) > 50_000_000:
                raise ValueError(f"{domain}: cross-stream skew exceeds 50 ms")
        imu = samples[key]
        if imu["frame_id"] != frame:
            raise ValueError("IMU frame_id differs from declared mounting")
        cov = _vector(imu["orientation_covariance"], 9, "orientation covariance")
        if np.any(cov[[0, 4, 8]] < 0):
            raise ValueError("IMU orientation unavailable or invalid covariance")
        world_from_imu = _quaternion(imu["orientation_xyzw"])
        # R_world_body = R_world_imu * R_body_imu.T.
        gravity_body = mount @ world_from_imu.T @ np.array([0., 0., -9.81])
        positions = _joint_positions(samples["arm_state"], ARM_JOINT_NAMES)
        model_hash = None
        if body == "pelvis":
            if urdf is None:
                raise ValueError("Pelvis IMU requires explicit waist URDF")
            waist = _joint_positions(samples["waist_state"], WAIST_NAMES)
            pelvis_from_torso, model_hash = _waist_rotation(urdf, waist)
            gravity_torso = pelvis_from_torso.T @ gravity_body
            result["rotation_pelvis_from_torso"] = pelvis_from_torso.tolist()
        else:
            gravity_torso = gravity_body
        result.update(usable_for_offline_estimate=True, gravity_torso_m_s2=gravity_torso.tolist(),
                      positions_rad=positions, gravity_frame="torso_link", timing=timing,
                      declared_mounting=mounting, waist_model_sha256=model_hash,
                      covariance_is_unknown=bool(np.all(cov == 0)))
    except (KeyError, ValueError, TypeError, AttributeError, OSError, ET.ParseError) as exc:
        result["errors"].append(str(exc))
    return result
