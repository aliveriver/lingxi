import pytest
import math

from lingxi_x2.acceptance import fixed_baseline_plan
from lingxi_x2.errors import SafetyInterlockError


@pytest.mark.parametrize("delta", [.01, .02, -.01, -.02, .03, -.03, .2, -.2, .5, -.5, 1., -1.])
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


@pytest.mark.parametrize("delta", [0., math.nextafter(.01, 0.), -math.nextafter(.01, 0.),
    math.nextafter(1., math.inf), -math.nextafter(1., math.inf),
    float("nan"), float("inf"), -float("inf")])
def test_fixed_baseline_rejects_unbounded_delta(delta):
    with pytest.raises(SafetyInterlockError):
        list(fixed_baseline_plan([0.]*14, 0, delta))


def test_admitted_delta_does_not_override_absolute_joint_limits():
    baseline = [.4, 0., 0., -1.2, 0., 0., 0.] * 2
    baseline[0] = 1.9
    with pytest.raises(SafetyInterlockError, match='outside'):
        list(fixed_baseline_plan(baseline, 0, .5))


@pytest.mark.parametrize('delta', [.01, -.01])
def test_two_second_quintic_is_smooth_bounded_and_returns_fixed_baseline(delta):
    baseline = (.4, 0., 0., -1.2, 0., 0., 0.) * 2
    plan = list(fixed_baseline_plan(baseline, 0, delta, profile='quintic', ramp_duration_s=2.))
    assert len(plan) == 350
    assert all(q[1:] == baseline[1:] for _, q in plan)
    assert plan[49][1] == plan[-1][1] == baseline
    assert plan[149][1][0] == pytest.approx(baseline[0]+delta)
    # Peak rate remains BELOW the former 1 s linear diagnostic's |delta|/s.
    gaps = [abs(b[1][0]-a[1][0]) for a,b in zip(plan,plan[1:])]
    assert max(gaps) * 50 < abs(delta)
    assert abs(plan[50][1][0]-baseline[0]) < abs(delta)*1e-5
    assert abs(plan[298][1][0]-baseline[0]) < abs(delta)*1e-5


@pytest.mark.parametrize('options', [{'profile':'other'}, {'ramp_duration_s':.5}, {'ramp_duration_s':float('nan')}])
def test_invalid_fixed_baseline_profile_rejected(options):
    with pytest.raises(SafetyInterlockError):
        list(fixed_baseline_plan([0.]*14,0,.01,**options))
