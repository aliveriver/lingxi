"""Offline geometry evidence from frozen official ZIPs; never outputs a mount.

Run with uv run python; add --plot with uv run --with matplotlib python.
Mesh bounds use URDF metres (all selected mesh scales/origins are identity).
An old-to-current palm mesh transform is NOT a wrist-to-palm transform.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import zipfile

import numpy as np


ARCHIVES = {
    "robot": ("logs/model-adaptation-followup-20260929/legacy-official-models.zip",
              "aeb0155a372111a7421397883f8570ca6b291908ee6e39214381b2a3652f48c2"),
    "left": ("logs/official-model-refresh-20260929/hand-left.zip",
             "8764a84d911dd5d46ece309bb3712e71c23fedffd366e353fc737f74f7e7efdb"),
    "right": ("logs/official-model-refresh-20260929/hand-right.zip",
              "f4b48bbc28085370300496eac59a32f3bac497cdb9dda2447156c727c41fef34"),
}


def binary_stl(raw):
    if len(raw) < 84:
        raise ValueError("Truncated STL")
    count = struct.unpack_from("<I", raw, 80)[0]
    if count == 0 or len(raw) != 84 + 50 * count:
        raise ValueError("Expected exact binary STL length and nonempty mesh")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    vertices = np.frombuffer(raw, dtype=dtype, offset=84)["vertices"].astype(float)
    if not np.isfinite(vertices).all():
        raise ValueError("Nonfinite STL vertex")
    return vertices


def fit_corresponding_vertices(source, target):
    a, b = source.reshape(-1, 3), target.reshape(-1, 3)
    if a.shape != b.shape:
        raise ValueError("Mesh vertex correspondence requires equal shapes")
    u, _, vt = np.linalg.svd((a - a.mean(0)).T @ (b - b.mean(0)))
    rotation = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
    translation = b.mean(0) - rotation @ a.mean(0)
    residuals = np.linalg.norm(a @ rotation.T + translation - b, axis=1)
    return rotation, translation, {
        "definition": "p_current_palm = R @ p_old_palm + t; NOT wrist mounting",
        "method": "proper rigid least-squares fit using original STL triangle/vertex order",
        "corresponding_vertex_count": len(a), "rotation_current_from_old": rotation.tolist(),
        "translation_current_from_old_m": translation.tolist(),
        "max_vertex_residual_m": float(residuals.max()),
        "rms_vertex_residual_m": float(np.sqrt(np.mean(residuals ** 2))),
    }


def audit(root):
    meshes, summaries, exports, sources = {}, {}, {}, {}
    for key, (relative, expected) in ARCHIVES.items():
        raw = (root / relative).read_bytes()
        if hashlib.sha256(raw).hexdigest() != expected:
            raise ValueError(f"Archive hash mismatch: {relative}")
        sources[key] = {"path": relative, "sha256": expected}
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            for member in archive.namelist():
                if not member.lower().endswith(".stl") or not any(
                    term in member.lower() for term in ("palm", "wrist_roll")
                ):
                    continue
                data = archive.read(member)
                vertices = binary_stl(data)
                meshes[member] = vertices
                low, high = vertices.min(axis=(0, 1)), vertices.max(axis=(0, 1))
                summaries[member] = {"archive": key, "sha256": hashlib.sha256(data).hexdigest(),
                                     "triangles": len(vertices), "bounds_m": [low.tolist(), high.tolist()],
                                     "extents_m": (high - low).tolist()}
            if key == "robot":
                continue
            csv_name = next(n for n in archive.namelist() if n.endswith(".csv"))
            urdf_name = next(n for n in archive.namelist() if n.endswith(".urdf"))
            csv_bytes, urdf_bytes = archive.read(csv_name), archive.read(urdf_name)
            rows = list(csv.DictReader(io.StringIO(csv_bytes.decode("utf-8-sig"))))
            model = ET.fromstring(urdf_bytes)
            palm = "l_palm" if key == "left" else "R_palm"
            row = next(r for r in rows if r["Link Name"] == palm)
            inertial = model.find(f"link[@name='{palm}']/inertial")
            children = []
            for child in rows:
                if child["Parent"] == palm:
                    joint = model.find(f"joint[@name='{child['Joint Name']}']")
                    children.append({"joint": child["Joint Name"],
                                     "csv_origin_m": [float(child["Joint Origin " + a]) for a in "XYZ"],
                                     "urdf_origin_m": [float(x) for x in joint.find("origin").get("xyz").split()]})
            exports[key] = {
                "csv_member": csv_name, "csv_sha256": hashlib.sha256(csv_bytes).hexdigest(),
                "urdf_member": urdf_name, "urdf_sha256": hashlib.sha256(urdf_bytes).hexdigest(),
                "csv_total_mass_kg": sum(float(r["Mass"]) for r in rows),
                "urdf_total_mass_kg": sum(float(m.get("value")) for m in model.findall("link/inertial/mass")),
                "csv_palm_mass_kg": float(row["Mass"]),
                "urdf_palm_mass_kg": float(inertial.find("mass").get("value")),
                "csv_palm_com_m": [float(row["Center of Mass " + a]) for a in "XYZ"],
                "urdf_palm_com_m": [float(x) for x in inertial.find("origin").get("xyz").split()],
                "root_joint_origins": children,
            }
    rotation, translation, fit = fit_corresponding_vertices(
        meshes["OmniHand2025left/meshes/l_palm_old_frame_backup.STL"],
        meshes["OmniHand2025left/meshes/l_palm.STL"])
    left = exports["left"]
    transformed_com = rotation @ np.array(left["csv_palm_com_m"]) + translation
    fit["transformed_csv_palm_com_m"] = transformed_com.tolist()
    fit["csv_to_urdf_palm_com_residual_m"] = float(np.linalg.norm(transformed_com - left["urdf_palm_com_m"]))
    fit["csv_child_origin_residuals_m"] = {
        r["joint"]: float(np.linalg.norm(rotation @ np.array(r["csv_origin_m"]) + translation - r["urdf_origin_m"]))
        for r in left["root_joint_origins"]}
    report = {
        "kind": "offline_wrist_palm_geometry_audit", "schema_version": 1,
        "robot_connected": False, "hardware_validated": False, "execution_authorized": False,
        "mounting_transform_established": False, "combined_model_ready": False,
        "user_confirmed_mechanical_connection": "O10 palm directly fixed to robot wrist",
        "sources": sources, "mesh_inventory": summaries, "csv_vs_urdf": exports,
        "left_old_to_current_palm_mesh_fit": fit,
        "limitations": [
            "Mesh shape does not establish inertial component membership, mass or density.",
            "CSV exports are not authoritative replacements for the current URDF.",
            "Mesh bounds and fitted old palm frame do not identify installed wrist-to-palm transforms.",
            "No cross-model assembly, command, HAL correction or live model replacement is generated.",
        ],
    }
    return report, meshes


def plot_projections(meshes, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    selected = [
        ("v1.3 LEFT wrist_roll mesh", "X2_URDF-v1.3.0/meshes/left_wrist_roll_link.STL"),
        ("v1.3 RIGHT wrist_roll mesh", "X2_URDF-v1.3.0/meshes/right_wrist_roll_link.STL"),
        ("O10 LEFT palm mesh", "OmniHand2025left/meshes/l_palm.STL"),
        ("O10 RIGHT palm mesh", "OmniHand2025right/meshes/R_palm.STL"),
    ]
    fig, axes = plt.subplots(4, 3, figsize=(12, 14), constrained_layout=True)
    for row, (title, name) in enumerate(selected):
        triangles = meshes[name] * 1000
        for col, (a, b) in enumerate(((0, 1), (0, 2), (1, 2))):
            ax = axes[row, col]
            # Every triangle is projected; no inferred assembly or frame alignment.
            ax.add_collection(PolyCollection(triangles[:, :, [a, b]], facecolors="#9eb5ca",
                                             edgecolors="#354d65", linewidths=.04, rasterized=True))
            ax.autoscale_view()
            ax.plot(0, 0, "+", color="#c73523", markersize=9, markeredgewidth=1.8)
            ax.set_aspect("equal", adjustable="datalim")
            ax.set_xlabel("XYZ"[a] + " (mm)")
            ax.set_ylabel("XYZ"[b] + " (mm)")
            ax.set_title(title, fontsize=10)
            ax.grid(alpha=.2)
    fig.suptitle("Official meshes in their OWN local frames\nRed + = local origin; NOT an assembled robot or calibrated mounting", fontsize=13)
    with destination.open("xb") as stream:
        fig.savefig(stream, format="png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory; refuses reuse")
    parser.add_argument("--plot", action="store_true", help="Requires optional matplotlib; no project dependency change")
    args = parser.parse_args()
    report, meshes = audit(args.root)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    with (args.output_dir / "geometry-audit.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    if args.plot:
        plot_projections(meshes, args.output_dir / "mesh-projections.png")
    print(json.dumps({"output_dir": str(args.output_dir), "mounting_transform_established": False,
                      "palm_frame_fit_max_residual_m": report["left_old_to_current_palm_mesh_fit"]["max_vertex_residual_m"]}))


if __name__ == "__main__":
    main()
