"""Independent synthetic mesh checks; no ignored evidence or robot runtime needed."""
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


@pytest.fixture
def audit_module(monkeypatch):
    scripts = Path(__file__).resolve().parents[1] / "scripts"
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location("wrist_component_audit_test", scripts / "audit_wrist_components.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def tetrahedron():
    points = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    return points[[[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]]]


def test_disconnected_solids_remain_separate(audit_module, tetrahedron):
    mesh = np.concatenate([tetrahedron, tetrahedron + 3.])
    groups = audit_module.components(mesh)
    assert [len(g) for g in groups] == [4, 4]
    assert sorted(np.concatenate(groups).tolist()) == list(range(8))


def test_shared_vertex_means_connectivity_not_cad_identity(audit_module, tetrahedron):
    # Two different solids touch at exactly one vertex, so connectivity cannot
    # be treated as a reliable component bill of materials.
    mesh = np.concatenate([tetrahedron, -tetrahedron])
    assert len(audit_module.components(mesh)) == 1


def test_rounding_effect_is_explicit(audit_module):
    mesh = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]],
                     [[0., 0., 1e-8], [-1., 0., 0.], [0., -1., 0.]]])
    assert len(audit_module.components(mesh)) == 2
    assert len(audit_module.components(mesh, 7)) == 1


def test_closed_oriented_tetrahedron(audit_module, tetrahedron):
    result = audit_module.topology(tetrahedron)
    assert result["boundary_edges"] == 0
    assert result["nonmanifold_edges"] == 0
    assert result["two_face_edges_with_inconsistent_winding"] == 0
    assert result["degenerate_triangles"] == 0
    assert result["mass_inferred"] is False


def test_missing_face_has_three_boundary_edges(audit_module, tetrahedron):
    assert audit_module.topology(tetrahedron[:-1])["boundary_edges"] == 3


def test_duplicate_face_has_three_nonmanifold_edges(audit_module, tetrahedron):
    mesh = np.concatenate([tetrahedron, tetrahedron[:1]])
    assert audit_module.topology(mesh)["nonmanifold_edges"] == 3


def test_reversed_face_has_three_winding_conflicts(audit_module, tetrahedron):
    tetrahedron[0] = tetrahedron[0, ::-1]
    assert audit_module.topology(tetrahedron)["two_face_edges_with_inconsistent_winding"] == 3


def test_zero_area_triangle_reported(audit_module):
    result = audit_module.topology(np.array([[[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]]]))
    assert result["degenerate_triangles"] == 1


def test_plane_area_and_height_known_independently(audit_module):
    # 2 x 3 rectangle at z=5, triangulated, plus a non-horizontal face.
    mesh = np.array([[[0., 0., 5.], [2., 0., 5.], [0., 3., 5.]],
                     [[2., 0., 5.], [2., 3., 5.], [0., 3., 5.]],
                     [[0., 0., 5.], [2., 0., 5.], [0., 0., 6.]]])
    rows = audit_module.planar_z_surfaces(mesh)
    assert len(rows) == 1
    assert rows[0]["z_m"] == 5.
    assert rows[0]["summed_triangle_area_m2"] == 6.
    assert rows[0]["triangles"] == 2
    assert rows[0]["xy_bounds_m"] == [[0., 0.], [2., 3.]]


def test_overlapping_triangles_not_mislabelled_union_area(audit_module):
    face = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
    row = audit_module.planar_z_surfaces(np.concatenate([face, face]))[0]
    assert row["summed_triangle_area_m2"] == 1.  # Union would be 0.5.


def test_source_hash_mismatch_prevents_archive_parse(audit_module, tmp_path, monkeypatch):
    path = tmp_path / "source.zip"
    path.write_bytes(b"not an archive")
    monkeypatch.setattr(audit_module, "ARCHIVES", {"robot": ("source.zip", "0" * 64)})
    with pytest.raises(ValueError, match="Archive hash mismatch"):
        audit_module.audit(tmp_path)


def test_existing_output_directory_not_overwritten(audit_module, tmp_path, monkeypatch):
    output = tmp_path / "existing"
    output.mkdir()
    marker = output / "keep.txt"
    marker.write_text("old evidence")
    monkeypatch.setattr(audit_module, "audit", lambda root: ({}, {}))
    monkeypatch.setattr(sys, "argv", ["audit_wrist_components", "--output-dir", str(output)])
    with pytest.raises(FileExistsError):
        audit_module.main()
    assert list(output.iterdir()) == [marker]
    assert marker.read_text() == "old evidence"
