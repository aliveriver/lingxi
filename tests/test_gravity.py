import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from lingxi_x2.cli import main
from lingxi_x2.gravity import StaticArmModel, gravity_report
from lingxi_x2.models import ARM_JOINT_NAMES


@pytest.fixture
def urdf(tmp_path):
    # Synthetic pendulum on each side: only final link has mass, all rotations
    # about +Y. Its COM is +X from the last joint, independently known U(q).
    root = ET.Element("robot", name="synthetic_test_only")
    ET.SubElement(root, "link", name="torso_link")
    for names in (ARM_JOINT_NAMES[:7], ARM_JOINT_NAMES[7:]):
        parent = "torso_link"
        for index, name in enumerate(names):
            child = name.replace("_joint", "_link")
            link = ET.SubElement(root, "link", name=child)
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "mass", value="2" if index == 6 else "0")
            ET.SubElement(inertial, "origin", xyz="1 0 0")
            joint = ET.SubElement(root, "joint", name=name, type="revolute")
            ET.SubElement(joint, "parent", link=parent)
            ET.SubElement(joint, "child", link=child)
            ET.SubElement(joint, "axis", xyz="0 1 0")
            ET.SubElement(joint, "limit", lower="-3", upper="3", effort="50")
            parent = child
    path = tmp_path / "pendulum.urdf"
    ET.ElementTree(root).write(path)
    return path


def request():
    return {"positions_rad": dict.fromkeys(ARM_JOINT_NAMES, 0.),
            "gravity_torso_m_s2": [0, 0, -9.81],
            "assumed_stiffness_nm_rad": 40.,
            "payloads": {side: {"mass_kg": 0., "com_wrist_m": [0, 0, 0]}
                         for side in ("left", "right")}}


def test_analytic_pendulum_sign_and_payload(urdf):
    model = StaticArmModel(urdf)
    q = [.2, 0, 0, 0, 0, 0, 0]
    result = model.evaluate("left", q, [0, 0, -9.81], payload_mass_kg=3, payload_com_wrist_m=[2, 0, 0])
    np.testing.assert_allclose(result["gravity_torque_nm"], [-8 * 9.81 * math.cos(.2)] * 7)
    assert result["potential_energy_j"] == pytest.approx(-8 * 9.81 * math.sin(.2))


def test_general_frames_against_energy_derivative(urdf):
    tree = ET.parse(urdf)
    for i, joint in enumerate(tree.getroot().findall("joint")):
        ET.SubElement(joint, "origin", xyz=f"{i * .01} .03 -.02", rpy=".1 -.2 .3")
        joint.find("axis").set("xyz", "1 2 3")
    tree.write(urdf)
    model = StaticArmModel(urdf)
    rng = np.random.default_rng(92)
    for side in ("left", "right"):
        for _ in range(4):
            q = rng.uniform(-.5, .5, 7)
            def calc(angles):
                return model.evaluate(side, angles, [1, 2, -9.5], payload_mass_kg=.6,
                                      payload_com_wrist_m=[.01, -.02, .12])
            derivative = []
            for i in range(7):
                step = np.eye(7)[i] * 1e-6
                derivative.append((calc(q + step)["potential_energy_j"] - calc(q - step)["potential_energy_j"]) / 2e-6)
            np.testing.assert_allclose(calc(q)["gravity_torque_nm"], derivative, atol=1e-7)


def test_report_is_offline_diagnostic(urdf, monkeypatch, tmp_path, capsys):
    def forbidden(*args, **kwargs):
        pytest.fail("offline report must not initialize a client or ROS")
    monkeypatch.setattr("lingxi_x2.cli.X2Client", forbidden)
    monkeypatch.setattr("lingxi_x2.cli.reexec_with_ros_environment", forbidden)
    input_path = tmp_path / "input.json"
    input_path.write_text(json.dumps(request()))
    assert main(["--config", "nonexistent.yaml", "gravity-report", "--urdf", str(urdf), "--input", str(input_path)]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["hardware_validated"] is False and report["command_output"] is False
    assert len(report["model"]["sha256"]) == 64
    assert all(report["arms"]["left"]["equivalent_exceeds_002_rad"])


@pytest.mark.parametrize("field,value", [
    ("gravity_torso_m_s2", [0, 0, 0]), ("gravity_torso_m_s2", [0, float("nan"), -9.81]),
    ("gravity_torso_m_s2", [0, -9.81]), ("assumed_stiffness_nm_rad", 0),
    ("assumed_stiffness_nm_rad", float("inf")), ("positions_rad", {}), ("payloads", {}),
])
def test_bad_report_inputs(urdf, field, value):
    data = request()
    data[field] = value
    with pytest.raises(ValueError):
        gravity_report(urdf, data)


@pytest.mark.parametrize("change", ["nan_pose", "limit", "negative_payload", "nan_com"])
def test_pose_and_load_rejected(urdf, change):
    data = request()
    if change in {"nan_pose", "limit"}:
        data["positions_rad"][ARM_JOINT_NAMES[0]] = float("nan") if change == "nan_pose" else 4.
    elif change == "negative_payload":
        data["payloads"]["left"]["mass_kg"] = -.5
    else:
        data["payloads"]["right"]["com_wrist_m"] = [0, 0, float("nan")]
    with pytest.raises(ValueError):
        gravity_report(urdf, data)


@pytest.mark.parametrize("change", ["duplicate", "missing_mass", "wrong_parent", "extra_tool", "zero_axis", "mimic"])
def test_unsupported_models_rejected(urdf, change):
    tree = ET.parse(urdf)
    root = tree.getroot()
    joint = root.find("joint")
    if change == "duplicate":
        root.append(joint)
    elif change == "missing_mass":
        inertial = root.findall("link")[1].find("inertial")
        inertial.remove(inertial.find("mass"))
    elif change == "wrong_parent":
        joint.find("parent").set("link", "pelvis")
    elif change == "extra_tool":
        tool = ET.SubElement(root, "joint", name="hand", type="fixed")
        ET.SubElement(tool, "parent", link="left_wrist_roll_link")
        ET.SubElement(tool, "child", link="tool")
    elif change == "mimic":
        ET.SubElement(joint, "mimic", joint="other")
    else:
        joint.find("axis").set("xyz", "0 0 0")
    tree.write(urdf)
    with pytest.raises(ValueError):
        StaticArmModel(urdf)


def test_duplicate_json_rejected(urdf, tmp_path, capsys):
    path = tmp_path / "duplicate.json"
    path.write_text('{"positions_rad": {}, "positions_rad": {}}')
    assert main(["gravity-report", "--urdf", str(urdf), "--input", str(path)]) == 2
    assert "Duplicate JSON key" in capsys.readouterr().err


def test_model_comparison_distinguishes_file_and_parameter_changes(urdf, tmp_path):
    from lingxi_x2.model_audit import compare_arm_models
    candidate = tmp_path / "candidate.urdf"
    candidate.write_bytes(urdf.read_bytes() + b'\n')
    result = compare_arm_models(urdf, candidate)
    assert result["compared_parameters_equal"] and not result["identical_files"]
    tree = ET.parse(candidate)
    tree.getroot().findall("link")[-1].find("inertial/mass").set("value", "3")
    tree.getroot().find("joint/limit").set("upper", "2")
    tree.write(candidate)
    result = compare_arm_models(urdf, candidate)
    assert result["groups_equal"] == {"kinematics": True, "static_dynamics": False, "limits": False}
    assert len(result["differences"]) == 2
    assert not result["hardware_validated"]


def test_model_comparison_detects_axis_sign_and_com(urdf, tmp_path):
    from lingxi_x2.model_audit import compare_arm_models
    candidate = tmp_path / "candidate.urdf"
    tree = ET.parse(urdf)
    tree.getroot().find("joint/axis").set("xyz", "0 -1 0")
    tree.getroot().findall("link")[-1].find("inertial/origin").set("xyz", "2 0 0")
    tree.write(candidate)
    result = compare_arm_models(urdf, candidate)
    assert result["groups_equal"] == {"kinematics": False, "static_dynamics": False, "limits": True}


def test_gravity_difference_bound_analytic_and_reachable(urdf):
    from lingxi_x2.gravity_following import gravity_difference_bound
    model=StaticArmModel(urdf)
    a=[0.]*7;b=[.2,0.,0.,0.,0.,0.,0.]
    result=gravity_difference_bound(model,'left',a,b,gravity_magnitude_m_s2=9.81,
                                    payload_mass_kg=3.,payload_com_wrist_m=[2.,0.,0.])
    # For this pendulum mass*lever=8 kg*m; rotated unit lever difference is 2sin(theta/2).
    expected=8*9.81*2*math.sin(.1)
    np.testing.assert_allclose(result['max_abs_delta_gravity_torque_nm'],[expected]*7)
    coefficients=np.asarray(result['delta_torque_per_gravity_component_kg_m'])
    g=9.81*coefficients[0]/np.linalg.norm(coefficients[0])
    def calc(q,gravity):return np.asarray(model.evaluate('left',q,gravity,payload_mass_kg=3.,payload_com_wrist_m=[2.,0.,0.])['gravity_torque_nm'])
    np.testing.assert_allclose(calc(b,g)-calc(a,g),[expected]*7)
    rng=np.random.default_rng(11)
    for _ in range(50):
        g=rng.normal(size=3);g=9.81*g/np.linalg.norm(g)
        assert np.all(np.abs(calc(b,g)-calc(a,g))<=expected+1e-10)
    assert not result['command_output']


def test_gravity_difference_zero_pose_change_and_bad_gravity(urdf):
    from lingxi_x2.gravity_following import gravity_difference_bound
    model=StaticArmModel(urdf)
    kwargs={'payload_mass_kg':0.,'payload_com_wrist_m':[0.,0.,0.]}
    r=gravity_difference_bound(model,'right',[0.]*7,[0.]*7,gravity_magnitude_m_s2=9.81,**kwargs)
    assert r['max_abs_delta_gravity_torque_nm']==[0.]*7
    for g in (0.,float('nan'),float('inf')):
        with pytest.raises(ValueError):gravity_difference_bound(model,'left',[0.]*7,[0.]*7,gravity_magnitude_m_s2=g,**kwargs)
