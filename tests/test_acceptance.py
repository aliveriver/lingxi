import pytest

from lingxi_x2.acceptance import fixed_baseline_plan
from lingxi_x2.errors import SafetyInterlockError


@pytest.mark.parametrize("delta", [.01, .02, -.01, -.02])
def test_fixed_command_baseline_only_moves_selected_joint_and_returns(delta):
    baseline = (.4, 0., 0., -1.2, 0., 0., 0.) * 2
    plan = list(fixed_baseline_plan(baseline, 0, delta))
    assert len(plan) == 250
    assert all(point[1:] == baseline[1:] for _, point in plan)
    assert plan[49][1] == baseline
    assert plan[149][1][0] == pytest.approx(.4 + delta)
    assert plan[-1][1] == baseline
    assert max(abs(b[1][0]-a[1][0]) for a, b in zip(plan, plan[1:])) <= abs(delta) / 50 + 1e-10
    assert {name: sum(p == name for p, _ in plan) for name, _ in plan} == {
        "baseline_hold": 50, "target_ramp": 50, "target_hold": 50,
        "recovery_ramp": 50, "recovery_hold": 50}


@pytest.mark.parametrize("delta", [.03, .2, -.2, float("nan"), float("inf")])
def test_fixed_baseline_rejects_unbounded_delta(delta):
    with pytest.raises(SafetyInterlockError):
        list(fixed_baseline_plan([0.]*14, 0, delta))
