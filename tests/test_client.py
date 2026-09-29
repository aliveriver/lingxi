from __future__ import annotations

import pytest

from lingxi_x2 import ArmCommand, HandCommand, X2Client
from lingxi_x2.config import ControlConfig, PlatformConfig
from lingxi_x2.errors import SafetyInterlockError
from lingxi_x2.models import CameraName, Side


@pytest.fixture
def client():
    config = PlatformConfig(
        backend="mock",
        control=ControlConfig(enabled=True, authority="upper_body_mc"),
    )
    with X2Client(config) as value:
        yield value


def test_mock_observation_is_structured(client: X2Client) -> None:
    observation = client.observe((CameraName.RGBD_FRONT_RGB,), include_tactile=True)
    assert len(observation.arm.joints) == 14
    assert len(observation.hands[Side.LEFT].joints) == 10
    assert observation.tactile[Side.RIGHT].palm.shape == (5, 5)
    assert observation.cameras[CameraName.RGBD_FRONT_RGB].width == 640


def test_control_requires_explicit_confirmation(client: X2Client) -> None:
    with pytest.raises(SafetyInterlockError, match="confirm_hardware"):
        client.move_hand(HandCommand.from_positions("left", [0.0] * 10))


def test_read_only_preflight_reports_disabled_without_writing() -> None:
    with X2Client(PlatformConfig(backend="mock")) as read_only:
        report = read_only.control_preflight("arm")
        assert report.ready is False
        assert report.checks[0].name == "control_enabled"
        with pytest.raises(SafetyInterlockError, match="control.enabled=false"):
            read_only.move_arm(ArmCommand.from_positions([0.0] * 14, 0.01), confirm_hardware=True)


def test_mock_arm_and_hand_trajectory(client: X2Client) -> None:
    arm_target = [-0.1, 0.1, 0.0, -0.2, 0.0, 0.0, 0.0] * 2
    client.move_arm(ArmCommand.from_positions(arm_target, 0.01), confirm_hardware=True)
    client.move_hand(HandCommand.from_positions("right", [0.2] * 10, 0.01), confirm_hardware=True)
    assert [joint.position_rad for joint in client.arm_state().joints] == pytest.approx(arm_target)
    assert [joint.position_rad for joint in client.hand_states()[Side.RIGHT].joints] == pytest.approx([0.2] * 10)


def test_arm_limits_are_enforced(client: X2Client) -> None:
    target = [0.0] * 14
    target[3] = 0.1
    with pytest.raises(SafetyInterlockError, match="outside"):
        client.move_arm(ArmCommand.from_positions(target, 0.01), confirm_hardware=True)


def test_single_joint_acceptance_is_bounded_and_recovers(client: X2Client) -> None:
    result = client.test_arm_joint(
        0,
        0.01,
        move_duration_s=0.01,
        target_hold_s=0.01,
        recovery_duration_s=0.01,
        recovery_hold_s=0.01,
        stop_observation_s=0.0,
        confirm_hardware=True,
    )
    assert result["target_positions_rad"][0] == pytest.approx(0.01)
    assert result["recovery_feedback_rad"] == pytest.approx([0.0] * 14)
    assert [event["phase"] for event in result["events"]] == [
        "baseline",
        "target_command",
        "target_feedback",
        "recovery_command",
        "recovery_feedback",
        "publishing_stopped",
    ]
    with pytest.raises(SafetyInterlockError, match="0.01-0.02"):
        client.test_arm_joint(0, 0.03, confirm_hardware=True)


def test_mode_change_returns_before_and_after_state(client: X2Client) -> None:
    result = client.set_motion_mode("DAMPING_DEFAULT", confirm_hardware=True)
    assert result["mc_before"]["action"] == "UPPERBODY_REMOTE_SPLIT"
    assert result["mc_after"]["action"] == "DAMPING_DEFAULT"


def test_upper_body_mode_requires_stable_standing_default(client: X2Client) -> None:
    backend = client._backend
    backend._motion_mode = "PASSIVE_DEFAULT"
    with pytest.raises(SafetyInterlockError, match="requires STAND_DEFAULT/RUNNING"):
        client.set_motion_mode("UPPERBODY_REMOTE_SPLIT", confirm_hardware=True)

    backend._motion_mode = "STAND_DEFAULT"
    result = client.set_motion_mode("UPPERBODY_REMOTE_SPLIT", confirm_hardware=True)
    assert result["mc_after"]["action"] == "UPPERBODY_REMOTE_SPLIT"


def test_hold_session_restores_standing_mode(client: X2Client) -> None:
    client._backend._motion_mode = "STAND_DEFAULT"
    result = client.test_upper_body_hold_session(
        0.01,
        stop_observation_s=0.0,
        confirm_hardware=True,
    )
    assert result["hold"]["kind"] == "upper_body_hold"
    assert result["hold"]["max_abs_tracking_error_rad"] == pytest.approx(0.0)
    assert result["mode_restore"]["mc_after"]["action"] == "STAND_DEFAULT"
    assert client.mc_state().action == "STAND_DEFAULT"


def test_joint_session_runs_bounded_test_and_restores_standing(client: X2Client) -> None:
    client._backend._motion_mode = "STAND_DEFAULT"
    result = client.test_upper_body_joint_session(
        0,
        0.01,
        settle_duration_s=0.01,
        move_duration_s=0.01,
        target_hold_s=0.01,
        recovery_duration_s=0.01,
        recovery_hold_s=0.01,
        stop_observation_s=0.0,
        confirm_hardware=True,
    )
    assert result["qualification_hold"]["max_abs_tracking_error_rad"] == pytest.approx(0.0)
    assert result["joint_test"]["delta_rad"] == pytest.approx(0.01)
    assert result["joint_test"]["recovery_feedback_rad"] == pytest.approx([0.0] * 14)
    assert result["mode_restore"]["mc_after"]["action"] == "STAND_DEFAULT"


def test_trajectory_and_dwell_share_one_preflight(client, monkeypatch):
    calls = []
    original = client._backend.control_preflight
    def preflight(subsystem):
        calls.append(subsystem)
        return original(subsystem)
    monkeypatch.setattr(client._backend, "control_preflight", preflight)
    stats = client.move_arm(ArmCommand.from_positions([0.0] * 14, .04),
                            confirm_hardware=True, hold_duration_s=.04)
    assert calls == ["arm"]
    assert stats["published_count"] == 4
    assert [f["phase"] for f in stats["frames"]] == ["trajectory", "trajectory", "dwell", "dwell"]


@pytest.mark.parametrize("duration", [-1, 0, float("nan"), float("inf")])
def test_invalid_session_duration_never_changes_mode(client, duration):
    client._backend._motion_mode = "STAND_DEFAULT"
    with pytest.raises(SafetyInterlockError):
        client.test_upper_body_joint_session(0, .01, move_duration_s=duration, confirm_hardware=True)
    assert client.mc_state().action == "STAND_DEFAULT"


@pytest.mark.parametrize("method", ["test_arm_joint", "test_upper_body_joint_session"])
def test_legacy_hardware_acceptance_is_blocked_before_any_io(method):
    from lingxi_x2.backends.ros2 import Ros2Backend

    backend = Ros2Backend.__new__(Ros2Backend)
    client = X2Client(PlatformConfig(backend="ros2"), backend=backend)
    with pytest.raises(SafetyInterlockError, match="Measured-feedback baseline"):
        getattr(client, method)(0, .01, confirm_hardware=True)


@pytest.mark.parametrize("overrides", [
    {"stiffness_nm_rad": 60.0}, {"damping_nm_s_rad": 3.0},
    {"efforts_nm": (0.1,) * 14}, {"velocities_rad_s": (0.1,) * 14},
])
def test_upper_body_rejects_fields_that_mc_cannot_receive(client, overrides, monkeypatch):
    monkeypatch.setattr(client._backend, "publish_arm", lambda *args: pytest.fail("must not publish"))
    with pytest.raises(SafetyInterlockError, match="position targets only"):
        client.move_arm(ArmCommand((0.0,) * 14, duration_s=.02, **overrides), confirm_hardware=True)
