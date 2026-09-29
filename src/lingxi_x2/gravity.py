"""Offline static gravity analysis. No ROS, client, command or transport dependency.

Only the explicit seven-link serial arms rooted at torso_link are supported.
Additional descendants are rejected rather than silently dropping their mass.
Payload COMs are measured from wrist_roll_link, independent of a display TCP.
Seven arm links do not prove that their inertials exclude end-effector parts.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .models import ARM_JOINT_NAMES


def _vector(value, size: int, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {size} finite numbers")
    return result


def _scalar(value, label: str, *, positive: bool = False) -> float:
    result = float(value)
    if not np.isfinite(result) or (result <= 0 if positive else result < 0):
        raise ValueError(f"{label} must be finite and {'positive' if positive else 'nonnegative'}")
    return result


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def _origin(element: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    origin = element.find("origin")
    attrs = {} if origin is None else origin.attrib
    xyz = _vector(attrs.get("xyz", "0 0 0").split(), 3, "origin xyz")
    rpy = _vector(attrs.get("rpy", "0 0 0").split(), 3, "origin rpy")
    axes = np.eye(3)
    return xyz, _rotation(axes[2], rpy[2]) @ _rotation(axes[1], rpy[1]) @ _rotation(axes[0], rpy[0])


@dataclass(frozen=True)
class _LinkJoint:
    name: str
    child: str
    origin: np.ndarray
    rotation: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    effort: float
    mass: float
    com: np.ndarray


class StaticArmModel:
    """Read URDF data only; never import or execute code from a model checkout."""

    def __init__(self, urdf: str | Path):
        raw = Path(urdf).read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        root = ET.fromstring(raw)
        if root.tag != "robot":
            raise ValueError("Expected a URDF robot root")
        self.robot_name = root.get("name", "")
        links = self._named(root, "link")
        joints = self._named(root, "joint")
        if "torso_link" not in links:
            raise ValueError("Missing torso_link base")
        self.arms: dict[str, tuple[_LinkJoint, ...]] = {}
        for side, names in (("left", ARM_JOINT_NAMES[:7]), ("right", ARM_JOINT_NAMES[7:])):
            parent = "torso_link"
            chain = []
            for name in names:
                if name not in joints:
                    raise ValueError(f"Missing joint: {name}")
                joint = joints[name]
                p, c, limit, axis = (joint.find(tag) for tag in ("parent", "child", "limit", "axis"))
                if (joint.get("type") != "revolute" or joint.find("mimic") is not None
                        or p is None or c is None or p.get("link") != parent
                        or limit is None or axis is None):
                    raise ValueError(f"Unsupported serial arm joint: {name}")
                child = c.attrib["link"]
                if child not in links:
                    raise ValueError(f"Missing link: {child}")
                inertial = links[child].find("inertial")
                if inertial is None or inertial.find("mass") is None:
                    raise ValueError(f"Missing mass/COM inertial: {child}")
                mass = _scalar(inertial.find("mass").attrib["value"], f"{child} mass")
                com, _ = _origin(inertial)
                xyz, rotation = _origin(joint)
                direction = _vector(axis.attrib["xyz"].split(), 3, f"{name} axis")
                norm = np.linalg.norm(direction)
                if not np.isfinite(norm) or norm < 1e-12:
                    raise ValueError(f"Invalid axis: {name}")
                low, high = _vector([limit.attrib["lower"], limit.attrib["upper"]], 2, "limits")
                if low >= high:
                    raise ValueError(f"Invalid limits: {name}")
                effort = _scalar(limit.attrib["effort"], f"{name} effort", positive=True)
                chain.append(_LinkJoint(name, child, xyz, rotation, direction / norm,
                                        low, high, effort, mass, com))
                parent = child
            # Fixed tools must also be accounted for. Do not accept a hand tree
            # and then add a second payload, or silently discard its inertials.
            children = {item.child for item in chain}
            if len(children) != 7 or "torso_link" in children:
                raise ValueError(f"Invalid/cyclic child links: {side}")
            for name, joint in joints.items():
                p, c = joint.find("parent"), joint.find("child")
                if p is not None and p.get("link") in children and name not in names:
                    raise ValueError(f"Unmodelled arm descendant: {name}; use an explicit bare-arm model")
                if c is not None and c.get("link") in children and name not in names:
                    raise ValueError(f"Multiple parents for arm link: {name}")
            self.arms[side] = tuple(chain)

    @staticmethod
    def _named(root: ET.Element, tag: str) -> dict[str, ET.Element]:
        result = {}
        for element in root.findall(tag):
            name = element.attrib["name"]
            if name in result:
                raise ValueError(f"Duplicate {tag}: {name}")
            result[name] = element
        return result

    def evaluate(self, side: str, q, gravity_torso_m_s2, *, payload_mass_kg: float,
                 payload_com_wrist_m) -> dict:
        if side not in self.arms:
            raise ValueError("side must be left or right")
        chain = self.arms[side]
        angles = _vector(q, 7, "q")
        gravity = _vector(gravity_torso_m_s2, 3, "gravity_torso_m_s2")
        mass = _scalar(payload_mass_kg, "payload_mass_kg")
        payload_com = _vector(payload_com_wrist_m, 3, "payload_com_wrist_m")
        if any(not item.lower <= angle <= item.upper for item, angle in zip(chain, angles)):
            raise ValueError(f"{side} q outside URDF limits; no implicit clipping")
        origins, axes, coms = [], [], []
        pos, rot = np.zeros(3), np.eye(3)
        for item, angle in zip(chain, angles):
            pos = pos + rot @ item.origin
            rot = rot @ item.rotation
            origins.append(pos.copy())
            axes.append(rot @ item.axis)
            rot = rot @ _rotation(item.axis, angle)
            coms.append(pos + rot @ item.com)
        coms.append(pos + rot @ payload_com)
        masses = [item.mass for item in chain] + [mass]
        # Compensation torque = dU/dq, U = -sum(m * g dot COM).
        tau = [-sum(masses[j] * np.dot(gravity, np.cross(axes[i], coms[j] - origins[i]))
                    for j in range(i, 8)) for i in range(7)]
        potential = -sum(m * np.dot(gravity, com) for m, com in zip(masses, coms))
        if not np.isfinite([*tau, potential]).all():
            raise ValueError("Nonfinite dynamics result")
        return {"joint_names": [item.name for item in chain],
                "positions_rad": angles.tolist(), "gravity_torque_nm": list(map(float, tau)),
                "potential_energy_j": float(potential),
                "arm_mass_kg": sum(item.mass for item in chain),
                "payload_mass_kg": mass, "payload_com_wrist_m": payload_com.tolist(),
                "urdf_effort_limits_nm": [item.effort for item in chain],
                "urdf_position_margin_rad": [float(min(qi - j.lower, j.upper - qi))
                                             for qi, j in zip(angles, chain)]}


def gravity_report(urdf: str | Path, request: dict) -> dict:
    """Report a declared offline scenario, never compensated command targets.

    tau/k is an explanatory equivalent, not an identified actuator model.
    No default gravity, pose, load or stiffness is inferred from the robot.
    """
    if not isinstance(request, dict) or set(request) != {
        "positions_rad", "gravity_torso_m_s2", "payloads", "assumed_stiffness_nm_rad"
    }:
        raise ValueError("Expected positions_rad, gravity_torso_m_s2, payloads, assumed_stiffness_nm_rad")
    positions = request["positions_rad"]
    if not isinstance(positions, dict) or set(positions) != set(ARM_JOINT_NAMES):
        raise ValueError("positions_rad must name all 14 arm joints exactly once")
    if not isinstance(request["payloads"], dict) or set(request["payloads"]) != {"left", "right"}:
        raise ValueError("Explicit left and right payloads required (zero excludes tools/hands)")
    gravity = _vector(request["gravity_torso_m_s2"], 3, "gravity_torso_m_s2")
    if not 9.0 <= np.linalg.norm(gravity) <= 10.5:
        raise ValueError("Expected Earth gravity in torso frame, magnitude 9.0–10.5 m/s²")
    stiffness = _scalar(request["assumed_stiffness_nm_rad"], "assumed stiffness", positive=True)
    model = StaticArmModel(urdf)
    arms = {}
    for side, chain in model.arms.items():
        payload = request["payloads"][side]
        if not isinstance(payload, dict) or set(payload) != {"mass_kg", "com_wrist_m"}:
            raise ValueError(f"{side} payload requires mass_kg and com_wrist_m")
        arm = model.evaluate(side, [positions[item.name] for item in chain], gravity,
                             payload_mass_kg=payload["mass_kg"], payload_com_wrist_m=payload["com_wrist_m"])
        equivalent = np.asarray(arm["gravity_torque_nm"]) / stiffness
        if not np.isfinite(equivalent).all():
            raise ValueError("Nonfinite torque/stiffness equivalent")
        arm["torque_over_assumed_stiffness_rad"] = equivalent.tolist()
        arm["equivalent_exceeds_002_rad"] = (np.abs(equivalent) > .02).tolist()
        arms[side] = arm
    return {"kind": "offline_gravity_report", "schema_version": 1,
            "hardware_validated": False, "command_output": False,
            "model": {"path": str(urdf), "sha256": model.sha256, "robot_name": model.robot_name},
            "gravity_frame": "torso_link", "gravity_torso_m_s2": gravity.tolist(),
            "assumed_stiffness_nm_rad": stiffness, "arms": arms,
            "limitations": ["Model/firmware, IMU frame, joint zeros and payload require machine-specific verification.",
                            "Zero payload adds no extra mass. Verify distal-link inertial contents before adding a hand/tool; avoid double counting.",
                            "A TCP offset is not a payload COM.",
                            "URDF limits are model metadata, not approved hardware limits.",
                            "Torque/stiffness equivalents are diagnostic values, not motion targets or authorization.",
                            "Static gravity excludes friction, drive dynamics, contacts and whole-body balance."]}
