"""Compare the supported seven-link arm URDF parameters, without hardware claims."""
from dataclasses import fields
from pathlib import Path

import numpy as np

from .gravity import StaticArmModel


def compare_arm_models(reference: str | Path, candidate: str | Path) -> dict:
    """Exact numerical comparison after parsing rotations and normalizing axes.

    This is intentionally limited to static arm geometry, mass, COM and joint
    limits. It does not establish firmware, IMU, payload or actuator compatibility.
    """
    models = [StaticArmModel(path) for path in (reference, candidate)]
    differences = []
    groups = {"child": "kinematics", "origin": "kinematics", "rotation": "kinematics",
              "axis": "kinematics", "mass": "static_dynamics", "com": "static_dynamics",
              "lower": "limits", "upper": "limits", "effort": "limits"}
    for side in ("left", "right"):
        for a, b in zip(models[0].arms[side], models[1].arms[side]):
            for field in fields(a):
                if field.name == "name":
                    continue
                av, bv = getattr(a, field.name), getattr(b, field.name)
                if not np.array_equal(av, bv):
                    differences.append({"joint": a.name, "field": field.name,
                                        "group": groups[field.name],
                                        "reference": av.tolist() if isinstance(av, np.ndarray) else av,
                                        "candidate": bv.tolist() if isinstance(bv, np.ndarray) else bv})
    return {"kind": "offline_arm_model_comparison", "schema_version": 1,
            "hardware_validated": False, "command_output": False,
            "reference": {"path": str(reference), "sha256": models[0].sha256},
            "candidate": {"path": str(candidate), "sha256": models[1].sha256},
            "identical_files": models[0].sha256 == models[1].sha256,
            "compared_parameters_equal": not differences,
            "groups_equal": {group: not any(d["group"] == group for d in differences)
                             for group in sorted(set(groups.values()))},
            "differences": differences,
            "scope": "14 arm joints: child link, origin, rotation, normalized axis, mass, COM, position/effort limits",
            "not_compared": ["waist/base/IMU chain", "inertia tensors", "velocity limits",
                             "MJCF and actuator configuration", "hands/tools/payload", "physical joint zeros",
                             "firmware and control behavior"]}
