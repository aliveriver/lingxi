"""Offline mesh components and planar surfaces, never a mounting calibration.

Uses frozen public archives from audit_wrist_palm_geometry. No ROS or SDK.
Shared vertices define components; a component is not necessarily a CAD part.
Surface areas are sums of triangles, not union/contact areas.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

from audit_wrist_palm_geometry import ARCHIVES, binary_stl


def components(vertices, decimals=None):
    points = vertices.reshape(-1, 3)
    if decimals is not None:
        points = np.round(points, decimals)
    unique, indices = np.unique(points, axis=0, return_inverse=True)
    faces = indices.reshape(-1, 3)
    parents = np.arange(len(unique))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for face in faces:
        for other in face[1:]:
            parents[find(other)] = find(face[0])
    labels = np.array([find(face[0]) for face in faces])
    groups = [np.flatnonzero(labels == label) for label in np.unique(labels)]
    return sorted(groups, key=lambda group: (-len(group), int(group[0])))


def planar_z_surfaces(vertices):
    areas = np.linalg.norm(np.cross(vertices[:, 1] - vertices[:, 0],
                                    vertices[:, 2] - vertices[:, 0]), axis=1) / 2
    mask = (np.ptp(vertices[:, :, 2], axis=1) <= 1e-8) & (areas > 1e-12)
    # Explicit binning at 0.1 micrometre; no physical accuracy implied.
    z = np.round(vertices[mask, :, 2].mean(axis=1), 7)
    rows = []
    for level in np.unique(z):
        chosen = vertices[mask][z == level]
        rows.append({"z_m": float(level), "summed_triangle_area_m2": float(areas[mask][z == level].sum()),
                     "triangles": len(chosen),
                     "xy_bounds_m": [chosen[:, :, :2].min((0, 1)).tolist(), chosen[:, :, :2].max((0, 1)).tolist()]})
    return sorted(rows, key=lambda row: -row["summed_triangle_area_m2"])


def topology(vertices):
    _, indices = np.unique(vertices.reshape(-1, 3), axis=0, return_inverse=True)
    faces = indices.reshape(-1, 3)
    directed = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    edges, inverse, counts = np.unique(np.sort(directed, axis=1), axis=0, return_inverse=True, return_counts=True)
    signs = np.where(directed[:, 0] < directed[:, 1], 1, -1)
    balance = np.bincount(inverse, weights=signs, minlength=len(edges))
    degenerate = np.linalg.norm(np.cross(vertices[:, 1] - vertices[:, 0],
                                        vertices[:, 2] - vertices[:, 0]), axis=1) <= 1e-14
    return {"boundary_edges": int(np.sum(counts == 1)),
            "nonmanifold_edges": int(np.sum(counts > 2)),
            "two_face_edges_with_inconsistent_winding": int(np.sum((counts == 2) & (balance != 0))),
            "degenerate_triangles": int(degenerate.sum()),
            "mass_inferred": False,
            "note": "Topology counts alone do not test self-intersection, CAD membership or density."}


def audit(root):
    archives = {}
    for key, (relative, expected) in ARCHIVES.items():
        raw = (root / relative).read_bytes()
        if hashlib.sha256(raw).hexdigest() != expected:
            raise ValueError(f"Archive hash mismatch: {relative}")
        archives[key] = raw
    wrists, palms, plot_meshes = {}, {}, {}
    with zipfile.ZipFile(io.BytesIO(archives["robot"])) as archive:
        urdf = ET.fromstring(archive.read("X2_URDF-v1.3.0/x2_ultra.urdf"))
        for side in ("left", "right"):
            member = f"X2_URDF-v1.3.0/meshes/{side}_wrist_roll_link.STL"
            raw = archive.read(member)
            vertices = binary_stl(raw)
            groups = components(vertices)
            rows = []
            for index, group in enumerate(groups):
                mesh = vertices[group]
                plot_meshes[f"{side} C{index}"] = mesh
                rows.append({"component": index, "triangles": len(group),
                             "bounds_m": [mesh.min((0, 1)).tolist(), mesh.max((0, 1)).tolist()],
                             "topology": topology(mesh), "z_planes_by_triangle_area": planar_z_surfaces(mesh)[:12]})
            inertial = urdf.find(f"link[@name='{side}_wrist_roll_link']/inertial")
            wrists[side] = {"member": member, "sha256": hashlib.sha256(raw).hexdigest(),
                            "grouping": "exact shared vertex positions; no CAD part labels inferred",
                            "component_sizes_by_rounding_decimal_places": {
                                str(d): [len(g) for g in components(vertices, d)] for d in (None, 8, 7, 6)},
                            "components": rows,
                            "aggregate_inertial_only": {"mass_kg": float(inertial.find("mass").get("value")),
                                                        "origin": inertial.find("origin").attrib,
                                                        "tensor_kg_m2": inertial.find("inertia").attrib},
                            "component_inertials": None}
    for side, prefix, package in (("left", "l", "OmniHand2025left"), ("right", "R", "OmniHand2025right")):
        with zipfile.ZipFile(io.BytesIO(archives[side])) as archive:
            member = f"{package}/meshes/{prefix}_palm.STL"
            raw = archive.read(member)
            vertices = binary_stl(raw)
            palms[side] = {"member": member, "sha256": hashlib.sha256(raw).hexdigest(),
                           "base_z_planes_by_triangle_area": [r for r in planar_z_surfaces(vertices) if abs(r["z_m"]) < .012],
                           "selected_mount_plane": None}
    return {"kind": "offline_wrist_component_surface_audit", "schema_version": 1,
            "robot_connected": False, "execution_authorized": False, "hardware_validated": False,
            "wrist_from_palm_transform": {"left": None, "right": None},
            "combined_model_ready": False,
            "sources": {key: {"path": p, "sha256": h} for key, (p, h) in ARCHIVES.items()},
            "wrists": wrists, "palms": palms,
            "missing_constraints": ["Which model surfaces actually mate after removing the hand cover/old end effector",
                                    "Corresponding feature centres and hole/connector orientation on the retained wrist and palm",
                                    "Installed left/right part revisions",
                                    "Component masses, COMs and inertia tensors for retained/replaced parts"],
            "limitations": ["One plane pair constrains separation and normal alignment, leaving two in-plane translations and yaw.",
                            "A single aggregate mass/COM/inertia does not uniquely determine component inertials.",
                            "No selected mounting plane, guessed transform, mass subtraction or URDF is emitted."]}, plot_meshes


def plot(meshes, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    fig, axes = plt.subplots(2, 3, figsize=(13, 8), constrained_layout=True)
    for row, side in enumerate(("left", "right")):
        for col in range(3):
            ax = axes[row, col]
            for i, color in enumerate(("#b3cbe2", "#ebbd81", "#a8c5a0")):
                tri = meshes[f"{side} C{i}"] * 1000
                axes_pair = ((0, 2), (1, 2), (0, 1))[col]
                ax.add_collection(PolyCollection(tri[:, :, list(axes_pair)], facecolors=color,
                                                 edgecolors="#444444", linewidths=.05, rasterized=True,
                                                 label=f"C{i}"))
            ax.autoscale_view()
            ax.plot(0, 0, "+", color="red", markersize=10)
            ax.set_aspect("equal", adjustable="datalim")
            ax.set_xlabel("XYZ"[axes_pair[0]] + " (mm)")
            ax.set_ylabel("XYZ"[axes_pair[1]] + " (mm)")
            ax.set_title(f"v1.3 {side}: exact shared-vertex components")
            ax.legend(loc="upper right")
    fig.suptitle("Component colours are mesh connectivity, NOT verified removable parts\nRed +: wrist_roll_link origin; no O10 mounting inferred")
    with destination.open("xb") as stream:
        fig.savefig(stream, format="png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    report, meshes = audit(args.root)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    with (args.output_dir / "component-audit.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    if args.plot:
        plot(meshes, args.output_dir / "wrist-components.png")
    print(json.dumps({"output_dir": str(args.output_dir), "combined_model_ready": False,
                      "components_per_wrist": {side: len(r["components"]) for side, r in report["wrists"].items()}}))


if __name__ == "__main__":
    main()
