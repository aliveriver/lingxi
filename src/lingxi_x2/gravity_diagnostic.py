"""Narrow first live diagnostic: left shoulder only, +0.002 rad bias cap.

Model/mounting remain hypotheses. A command is admissible only when all six
mount hypotheses give the same positive clipped bias. This is a bounded
partial-compensation experiment, not calibration or full gravity control.
"""
from __future__ import annotations

import math
import gc
from contextlib import contextmanager
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

from .compensation import quintic
from .errors import SafetyInterlockError
from .gravity import StaticArmModel, _origin
from .gravity_observation import _waist_rotation, _rotation_matrix, WAIST_NAMES
from .models import ARM_JOINT_NAMES
from .safety import validate_arm_target

BIAS_LIMIT_RAD = .002
DELTA_RAD = .01
RATE_HZ = 50.
MAX_BIAS_RATE = .002
MAX_COMMAND_RATE = .012
MAX_EXCURSION = .012


def limited_gravity_plan(baseline, *, bias_only=False):
    baseline = validate_arm_target(baseline)
    previous_bias, previous_command = 0., baseline
    phases = (("baseline_hold", 1.), ("bias_in", 2.), ("compensated_baseline_hold", 1.),
              ("target_ramp", 2.), ("target_hold", 1.), ("recovery_ramp", 2.),
              ("compensated_recovery_hold", 1.), ("bias_out", 2.), ("recovery_hold", 1.))
    if bias_only:
        phases = tuple(p for p in phases if p[0] in {
            "baseline_hold", "bias_in", "compensated_baseline_hold", "bias_out", "recovery_hold"})
    for phase, duration in phases:
        count = round(duration * RATE_HZ)
        for step in range(1, count + 1):
            s = quintic(step / count)
            motion = s if phase == "target_ramp" else 1-s if phase == "recovery_ramp" else float(phase == "target_hold")
            alpha = s if phase == "bias_in" else 1-s if phase == "bias_out" else float(phase in {
                "compensated_baseline_hold", "target_ramp", "target_hold", "recovery_ramp", "compensated_recovery_hold"})
            desired = list(baseline)
            desired[0] += DELTA_RAD * motion
            command = list(desired)
            bias = alpha * BIAS_LIMIT_RAD
            command[0] += bias
            command = validate_arm_target(command)
            if (abs(command[0]-baseline[0]) > MAX_EXCURSION + 1e-12
                    or abs(bias-previous_bias)*RATE_HZ > MAX_BIAS_RATE + 1e-12
                    or max(abs(a-b) for a,b in zip(command,previous_command))*RATE_HZ > MAX_COMMAND_RATE + 1e-12):
                raise SafetyInterlockError("Limited gravity plan exceeds hard diagnostic envelope")
            yield {"phase": phase, "desired_rad": desired, "bias_fraction": alpha,
                   "applied_bias_rad": bias, "command_rad": list(command)}
            previous_bias, previous_command = bias, command


def _angle(a, b):
    cosine = sum(x*y for x,y in zip(a,b)) / (_norm(a)*_norm(b))
    return math.acos(max(-1., min(1., cosine)))


def _norm(v):
    return math.sqrt(sum(x*x for x in v))


def _vec(v, size):
    if len(v) != size or any(isinstance(x, bool) or not math.isfinite(x) for x in v):
        raise SafetyInterlockError("Incomplete/nonfinite gravity input")
    return tuple(float(x) for x in v)


def _mv(m, v):
    return tuple(sum(x*y for x,y in zip(row,v)) for row in m)


def _transpose(m):
    return tuple(zip(*m))


def _mm(a, b):
    return tuple(tuple(sum(x*y for x,y in zip(row,col)) for col in zip(*b)) for row in a)


def _axis_rotation(axis, angle):
    x,y,z = axis
    c,s = math.cos(angle),math.sin(angle)
    d = 1-c
    return ((c+x*x*d, x*y*d-z*s, x*z*d+y*s),
            (y*x*d+z*s, c+y*y*d, y*z*d-x*s),
            (z*x*d-y*s, z*y*d+x*s, c+z*z*d))


def _joint_values(sample, names):
    values = {}
    for row in sample["joints"]:
        name = row["name"]
        if name in values or row.get("error_code") != 0:
            raise SafetyInterlockError("Duplicate/faulted gravity joint feedback")
        position, _ = _vec((row["position"],row["velocity"]),2)
        values[name] = position
    if set(values) != set(names):
        raise SafetyInterlockError("Incomplete gravity joint chain")
    return values


class LimitedGravityGuard:
    def __init__(self, urdf, factory_calibration):
        self.urdf = urdf
        self.model = StaticArmModel(urdf)
        # Validate the entire waist chain once with the existing strict reader.
        _, waist_hash = _waist_rotation(urdf, dict.fromkeys(WAIST_NAMES, 0.))
        if waist_hash != self.model.sha256:
            raise SafetyInterlockError("Model changed while initializing")
        joints = StaticArmModel._named(ET.fromstring(Path(urdf).read_bytes()), "joint")
        self.waist_chain = []
        for name in WAIST_NAMES:
            joint = joints[name]
            _, rotation = _origin(joint)
            axis = tuple(float(x) for x in joint.find("axis").attrib["xyz"].split())
            norm = _norm(axis)
            limits = joint.find("limit").attrib
            self.waist_chain.append((name, rotation.tolist(), tuple(x/norm for x in axis),
                                     float(limits["lower"]),float(limits["upper"])))
        self.model_file_stat = self._file_stat()
        self.mounts = {}
        for key, body, factory_key in (("chest_imu", "torso_link", "chest_imu"),
                                       ("pelvis_imu", "pelvis", "waist_imu")):
            rotation = _rotation_matrix(np.asarray(factory_calibration[factory_key]["R_baselink_sensor"]).reshape(3,3))
            for variant, matrix in (("identity", np.eye(3)), ("factory", rotation), ("transpose", rotation.T)):
                self.mounts[f"{key}_{variant}"] = {"imu_key": key, "body": body,
                    "expected_frame_id": "base_link", "rotation_body_from_imu": matrix.tolist()}
        self.reference_gravity = None
        self._coefficients = {}

    def _file_stat(self):
        value = Path(self.urdf).stat()
        return value.st_ino, value.st_size, value.st_mtime_ns

    def prepare_desired_poses(self, poses):
        """Cache only fixed URDF geometry, never IMU or other live samples."""
        for pose in poses:
            q = validate_arm_target(pose)
            if q not in self._coefficients:
                if len(self._coefficients) >= 1024:
                    raise SafetyInterlockError("Too many diagnostic poses")
                self._coefficients[q] = tuple(self.model.evaluate("left", q[:7], axis,
                    payload_mass_kg=0., payload_com_wrist_m=[0.,0.,0.])["gravity_torque_nm"][0] for axis in np.eye(3))

    @staticmethod
    def require_fresh(snapshot, now_mono, now_ros):
        # Repeat immediately before publication, AFTER all computation/sleep.
        if snapshot.get("errors") or snapshot.get("error_count", 0):
            raise SafetyInterlockError("Gravity capture reported invalid telemetry")
        if snapshot.get("kind") != "gravity_readonly_snapshot" or snapshot.get("schema_version") != 1:
            raise SafetyInterlockError("Unexpected gravity snapshot schema")
        receive, source = [], []
        for key in ("chest_imu", "pelvis_imu", "waist_state", "arm_state"):
            sample = snapshot["samples"][key]
            ages = (now_mono-sample["received_monotonic_ns"], now_ros-sample["received_ros_ns"],
                    now_ros-sample["stamp_ns"])
            if any(type(t) is not int or not 0 <= t <= 100_000_000 for t in ages):
                raise SafetyInterlockError(f"{key}: missing, future or older than 100 ms; "
                                          f"receive_mono/source_receive_ros/source_age_ns={ages}")
            if (sample["stamp_ns"] <= 0 or not 0 <= sample["received_ros_ns"]-sample["stamp_ns"] <= 100_000_000
                    or abs(ages[0]-ages[1]) > 50_000_000):
                raise SafetyInterlockError("Inconsistent source/receive clocks")
            receive.append(sample["received_monotonic_ns"])
            source.append(sample["stamp_ns"])
        if any(max(times)-min(times)>50_000_000 for times in (receive,source)):
            raise SafetyInterlockError("Gravity stream skew exceeds 50 ms")

    def evaluate(self, snapshot, desired, *, now_mono, now_ros):
        self.require_fresh(snapshot, now_mono, now_ros)
        if self._file_stat() != self.model_file_stat:
            raise SafetyInterlockError("Waist model changed after guard initialization")
        q = validate_arm_target(desired)
        for i, joint in enumerate(j for side in ("left", "right") for j in self.model.arms[side]):
            if not joint.lower <= q[i] <= joint.upper:
                raise SafetyInterlockError("Desired pose outside installed model limits")
        _joint_values(snapshot["samples"]["arm_state"], ARM_JOINT_NAMES)
        waist = _joint_values(snapshot["samples"]["waist_state"], WAIST_NAMES)
        waist_rotation = ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))
        for name, origin, axis, lower, upper in self.waist_chain:
            if not lower <= waist[name] <= upper:
                raise SafetyInterlockError("Waist feedback outside model limits")
            waist_rotation = _mm(_mm(waist_rotation,origin),_axis_rotation(axis,waist[name]))
        sensor_checks, imu_gravity = {}, {}
        for key in ("chest_imu", "pelvis_imu"):
            sample = snapshot["samples"][key]
            if sample["frame_id"] != "base_link":
                raise SafetyInterlockError("Unexpected IMU frame")
            covariance = _vec(sample["orientation_covariance"],9)
            if any(covariance[i] < 0 for i in (0,4,8)):
                raise SafetyInterlockError("IMU orientation unavailable")
            accel = _vec(sample["linear_acceleration_m_s2"], 3)
            if not 9 <= _norm(accel) <= 10.5:
                raise SafetyInterlockError("IMU acceleration inconsistent with static standing")
            quaternion = _vec(sample["orientation_xyzw"],4)
            norm = _norm(quaternion)
            if not .99 <= norm <= 1.01:
                raise SafetyInterlockError("Invalid IMU quaternion norm")
            x,y,z,w = (v/norm for v in quaternion)
            g = (-9.81*2*(x*z-y*w), -9.81*2*(y*z+x*w), -9.81*(1-2*(x*x+y*y)))
            imu_gravity[key] = g
            angle = _angle(g, tuple(-x for x in accel))
            if angle > math.radians(3):
                raise SafetyInterlockError("IMU quaternion/acceleration directions disagree")
            sensor_checks[key] = {"acceleration_norm": _norm(accel),
                                  "quaternion_acceleration_angle_rad": angle}
        # Static gravity torque is linear in g. Three basis evaluations replace
        # six repeated kinematic evaluations without approximating the model.
        self.prepare_desired_poses([q])
        coefficients = self._coefficients[q]
        hypotheses = {}
        for label, mounting in self.mounts.items():
            g = _mv(mounting["rotation_body_from_imu"], imu_gravity[mounting["imu_key"]])
            if mounting["body"] == "pelvis":
                g = _mv(_transpose(waist_rotation), g)
            if _angle(g, [0., 0., -9.81]) > math.radians(10):
                raise SafetyInterlockError("Torso tilt exceeds 10 degrees")
            tau = sum(a*b for a,b in zip(coefficients,g))
            raw = tau / 40.
            clipped = max(-BIAS_LIMIT_RAD,min(BIAS_LIMIT_RAD,raw))
            # Do not select a preferred uncertain mount, average opposite signs,
            # or relax this gate when the model predicts insufficient gravity.
            if not math.isfinite(raw) or abs(clipped-BIAS_LIMIT_RAD) > 1e-12:
                raise SafetyInterlockError("Mount hypotheses do not all support the +0.002 rad clipped bias")
            hypotheses[label] = {"gravity_torso_m_s2": list(g), "torque_nm": tau,
                                 "unbounded_bias_rad": raw, "clipped_bias_rad": clipped}
        gravities = {label: h["gravity_torso_m_s2"] for label,h in hypotheses.items()}
        directions = [tuple(x/_norm(g) for x in g) for g in gravities.values()]
        if min(sum(x*y for x,y in zip(a,b)) for a in directions for b in directions) < math.cos(math.radians(5)):
            raise SafetyInterlockError("Chest/pelvis/mount gravity hypotheses differ by more than 5 degrees")
        if self.reference_gravity is not None and any(_angle(g,self.reference_gravity[label]) > math.radians(1)
                                                     for label,g in gravities.items()):
            raise SafetyInterlockError("Torso gravity changed by more than 1 degree since qualification")
        if self.reference_gravity is None:
            self.reference_gravity = gravities
        return {"hypotheses": hypotheses, "sensor_checks": sensor_checks,
                "selected_clipped_bias_rad": BIAS_LIMIT_RAD,
                "assumed_stiffness_nm_rad": 40., "model_sha256": self.model.sha256,
                "additional_payload_mass_kg": 0., "model_and_mounting_verified": False}


def check_encoder_excursion(joints, reference):
    if tuple(j.name for j in joints) != ARM_JOINT_NAMES or len(reference) != 14:
        raise SafetyInterlockError("Incomplete or reordered arm feedback")
    if any(j.fault_code != 0 or not math.isfinite(j.position_rad) for j in joints):
        raise SafetyInterlockError("Invalid/faulted live arm feedback")
    for i,(joint,start) in enumerate(zip(joints,reference)):
        if abs(joint.position_rad-start) > (.025 if i == 0 else .005):
            raise SafetyInterlockError(f"Encoder excursion exceeds diagnostic envelope: {joint.name}")


@contextmanager
def diagnostic_mode(client, result, *, dry_run):
    """Single trial only; restore even if mode entry or streaming fails."""
    if client.status().backend != "ros2" or client.config.control.enabled:
        raise SafetyInterlockError("Requires ROS with disk control disabled")
    if dry_run:
        yield
        return
    client.config.control.enabled = True
    try:
        result["mode_entry"] = client.set_motion_mode("UPPERBODY_REMOTE_SPLIT", confirm_hardware=True)
        with client._backend.command_stream("arm"):
            yield
    finally:
        try:
            result["mode_restore"] = client.set_motion_mode("STAND_DEFAULT", confirm_hardware=True)
        finally:
            client.config.control.enabled = False


@contextmanager
def bounded_timing_window():
    """Avoid cyclic-GC pauses only inside this bounded diagnostic stream.

    Reference counting remains active. Trace buffers and the frame plan are
    bounded. Restore the caller's GC state on every exit, including exceptions.
    """
    enabled = gc.isenabled()
    if enabled:
        gc.collect()
        gc.disable()
    try:
        yield
    finally:
        if enabled:
            gc.enable()
