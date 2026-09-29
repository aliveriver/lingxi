from dataclasses import replace
import json

import pytest

from lingxi_x2 import X2Client, PlatformConfig
from lingxi_x2.models import ArmCommand
from lingxi_x2.experiments import PolicyAction
from lingxi_x2.shadow import ShadowExperimentRunner, JointActionAdapter


def observations():
    with X2Client(PlatformConfig(backend="mock")) as client:
        obs = client.observe((), True)
    return [obs, replace(obs, captured_monotonic_ns=obs.captured_monotonic_ns + 1)]


class Policy:
    def __init__(self, action=None): self.action = action or PolicyAction()
    def reset(self): self.calls = 0
    def act(self, observation): self.calls += 1; return self.action


def test_shadow_records_proposals_without_execution(tmp_path):
    policy = Policy(PolicyAction(arm=ArmCommand((0.,)*14)))
    output = tmp_path / "shadow.jsonl"
    result = ShadowExperimentRunner(policy).run(observations(), output, max_steps=10, source="mock")
    assert result["proposals"] == 2 and result["executed_actions"] == 0
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    assert rows[0]["source"] == "mock" and rows[0]["execution"] == "none"
    assert [r["record_type"] for r in rows] == ["metadata", "observation", "proposal", "observation", "proposal", "summary"]
    assert rows[2]["executed"] is False
    with pytest.raises(FileExistsError): ShadowExperimentRunner(policy).run(observations(), output, max_steps=1)


def test_error_preserves_input_and_failure(tmp_path):
    policy = Policy(PolicyAction(arm=ArmCommand((float("nan"),)*14)))
    path = tmp_path / "error.jsonl"
    with pytest.raises(ValueError): ShadowExperimentRunner(policy).run(observations(), path, max_steps=10)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert [r["record_type"] for r in rows] == ["metadata", "observation", "error"]
    assert policy.calls == 1


def test_deadline_stops_before_next_observation(tmp_path, monkeypatch):
    times = iter([0, 2_000_000_000])
    obs = observations()
    monkeypatch.setattr("lingxi_x2.shadow.time.monotonic_ns", lambda: next(times))
    policy = Policy()
    result = ShadowExperimentRunner(policy).run(obs, tmp_path / "late.jsonl", max_steps=2, inference_deadline_s=.5)
    assert result["reason"] == "inference_deadline_missed" and policy.calls == 1


def test_stop_has_no_motion_targets(tmp_path):
    policy = Policy(PolicyAction(stop=True))
    result = ShadowExperimentRunner(policy).run(observations(), tmp_path / "stop.jsonl", max_steps=10)
    assert result["reason"] == "policy_stop" and policy.calls == 1
    bad = Policy(PolicyAction(arm=ArmCommand((0.,)*14), stop=True))
    with pytest.raises(ValueError): ShadowExperimentRunner(bad).run(observations(), tmp_path / "bad.jsonl", max_steps=1)


@pytest.mark.parametrize("issue", ["degrees", "delta", "chunk", "length", "effort", "nan", "stop_number"])
def test_adapter_rejects_ambiguous_model_actions(issue):
    raw = {"units": "rad", "representation": "absolute_joint_positions", "duration_s": 1., "arm_positions_rad": [0.]*14}
    if issue == "degrees": raw["units"] = "deg"
    elif issue == "delta": raw["representation"] = "delta"
    elif issue == "chunk": raw["arm_positions_rad"] = [[0.]*14]*3
    elif issue == "length": raw["arm_positions_rad"] = [0.]*7
    elif issue == "effort": raw["effort_nm"] = [0.]*14
    elif issue == "nan": raw["duration_s"] = float("nan")
    elif issue == "stop_number": raw = {"stop": 1}
    with pytest.raises((ValueError, TypeError)):
        JointActionAdapter(lambda _: raw).act(observations()[0])


def test_adapter_two_hands_and_reset():
    calls = []
    adapter = JointActionAdapter(lambda _: {"units": "rad", "representation": "absolute_joint_positions", "duration_s": .5,
        "hand_positions_rad": {"left": [0.]*10, "right": [0.]*10}}, reset=lambda: calls.append("reset"))
    adapter.reset(); action = adapter.act(observations()[0])
    assert calls == ["reset"] and len(action.hands) == 2 and action.arm is None
