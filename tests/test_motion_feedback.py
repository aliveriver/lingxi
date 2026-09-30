from dataclasses import replace

import pytest

from lingxi_x2 import ArmCommand, X2Client
from lingxi_x2.backends.mock import MockBackend
from lingxi_x2.config import ControlConfig, PlatformConfig
from lingxi_x2.errors import DataTimeoutError
from lingxi_x2.models import ARM_JOINT_NAMES, ArmState, JointSample, Timestamp
from lingxi_x2.motion_feedback import arm_tracking_report


def state():
    return ArmState(Timestamp(1, 0, 110), tuple(
        JointSample(n, 0., 0., 0., fault_code=0) for n in ARM_JOINT_NAMES), 0)


def report(arm):
    return arm_tracking_report((.01,) + (0.,) * 13, arm, now_ns=120,
                               not_before_ns=100, max_age_ns=50)


def test_low_following_is_reported_against_desired_not_encoder_baseline():
    arm = state()
    arm = replace(arm, joints=(replace(arm.joints[0], position_rad=.00134), *arm.joints[1:]))
    result = report(arm)
    assert result["status"] == "observed"
    assert result["error_rad"][0] == pytest.approx(-.00866)
    assert result["max_abs_error_rad"] == pytest.approx(.00866)
    assert not result["motion_verified"]


@pytest.mark.parametrize("damage,reason", [
    ("old", "no_feedback_received_after_final_publish"),
    ("stale", "stale_or_future_feedback"),
    ("future", "stale_or_future_feedback"),
    ("missing", "incomplete_or_reordered_feedback"),
    ("reordered", "incomplete_or_reordered_feedback"),
    ("fault", "faulted_or_unknown_feedback_state"),
    ("unknown", "faulted_or_unknown_feedback_state"),
    ("domain", "faulted_or_unknown_feedback_state"),
    ("nan", "nonfinite_feedback"),
])
def test_invalid_feedback_never_reports_zero_error(damage, reason):
    arm = state()
    if damage in {"old", "stale", "future"}:
        arm = replace(arm, timestamp=replace(arm.timestamp, monotonic_ns={"old": 100, "stale": 1, "future": 121}[damage]))
    if damage == "missing":
        arm = replace(arm, joints=arm.joints[:-1])
    if damage == "reordered":
        arm = replace(arm, joints=arm.joints[::-1])
    if damage in {"fault", "unknown", "nan"}:
        change = {"position_rad": float("nan")} if damage == "nan" else {"fault_code": None if damage == "unknown" else 1}
        arm = replace(arm, joints=(*arm.joints[:-1], replace(arm.joints[-1], **change)))
    if damage == "domain":
        arm = replace(arm, domain_state=None)
    result = report(arm)
    assert result["status"] == "unavailable" and result["reason"] == reason
    assert "max_abs_error_rad" not in result
    assert not result["motion_verified"]


def test_all_axes_include_nonmoving_arm_error():
    arm = state()
    arm = replace(arm, joints=(*arm.joints[:-1], replace(arm.joints[-1], position_rad=.03)))
    assert report(arm)["max_abs_error_rad"] == .03


class LowFollowingBackend(MockBackend):
    def publish_arm(self, positions_rad, *args):
        self._arm[:] = [q * .134 for q in positions_rad]


def test_move_arm_returns_actual_error_and_does_not_correct_or_retry():
    config = PlatformConfig(backend="mock", control=ControlConfig(enabled=True, authority="upper_body_mc"))
    with X2Client(config, backend=LowFollowingBackend(config)) as client:
        result = client.move_arm(ArmCommand.from_positions([.01] + [0.] * 13, .02), confirm_hardware=True)
    assert result["published_count"] == 1
    assert result["tracking"]["max_abs_error_rad"] == pytest.approx(.00866)
    assert result["tracking"]["desired_rad"][0] == .01
    assert not result["tracking"]["motion_verified"]


def test_feedback_loss_after_publication_preserves_publication_evidence(monkeypatch):
    config = PlatformConfig(backend="mock", control=ControlConfig(enabled=True, authority="upper_body_mc"))
    backend = MockBackend(config)
    original = backend.publish_arm
    def publish(*args):
        original(*args)
        def missing(*args, **kwargs):
            raise DataTimeoutError("lost final feedback")
        monkeypatch.setattr(backend, "arm_state", missing)
    monkeypatch.setattr(backend, "publish_arm", publish)
    with X2Client(config, backend=backend) as client:
        result = client.move_arm(ArmCommand.from_positions([0.] * 14, .02), confirm_hardware=True)
    assert result["published_count"] == 1
    assert result["tracking"]["status"] == "unavailable"
    assert not result["tracking"]["motion_verified"]
