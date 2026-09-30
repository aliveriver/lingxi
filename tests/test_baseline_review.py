import json
import pytest

from lingxi_x2.baseline_review import qualify_trace, compare_qualified_files
from test_acceptance_report import synthetic_trace
from test_gravity_trial_report import synthetic


def with_gains(trace):
    trace = json.loads(json.dumps(trace))
    for row in trace['samples']['hal_arm']:
        for j in row['joints']: j.update(stiffness=40., damping=2.)
    return trace


def test_early_gain_transition_is_visible_but_terminal_baseline_can_qualify():
    trace = with_gains(synthetic_trace(gain=.1))
    for row in trace['samples']['hal_arm'][:3]:
        for j in row['joints']: j.update(stiffness=50., damping=3.)
    r = qualify_trace(trace)
    assert r['qualified_for_pair'] and r['original_verdict'] == 'fail'
    assert [v['all_axes_expected_40_2'] for v in r['hal_gain_changes']] == [False, True]
    assert not r['hardware_validated'] and not r['execution_authorized']


@pytest.mark.parametrize('damage', ['missing_gain','tail_gain','later_gain','other_axis_gain',
                                  'nan_gain','bool_gain','tail_drift','tail_spike','target_drift','dropped'])
def test_incomplete_or_unstable_baseline_rejected(damage):
    trace = with_gains(synthetic_trace())
    hal = trace['samples']['hal_arm']
    if damage == 'missing_gain': del hal[70]['joints'][0]['stiffness']
    if damage == 'tail_gain': hal[70]['joints'][0]['stiffness'] = 50.
    if damage == 'later_gain': hal[250]['joints'][0]['damping'] = 3.
    if damage == 'other_axis_gain': hal[70]['joints'][13]['stiffness'] = 50.
    if damage == 'nan_gain': hal[70]['joints'][0]['damping'] = float('nan')
    if damage == 'bool_gain': hal[70]['joints'][0]['damping'] = True
    if damage == 'tail_drift':
        for row in trace['samples']['arm_state'][60:80]: row['joints'][13]['position'] += .001
    if damage == 'tail_spike': trace['samples']['arm_state'][70]['joints'][0]['position'] += .001
    if damage == 'target_drift': hal[70]['joints'][0]['position'] += .0005
    if damage == 'dropped': trace['dropped']['arm_state'] = 1
    r = qualify_trace(trace)
    assert not r['qualified_for_pair']
    assert any(not c['passed'] for c in r['checks'])


def test_one_observed_step_drift_allowed_but_reported():
    trace = with_gains(synthetic_trace())
    for row in trace['samples']['arm_state'][60:80]: row['joints'][0]['position'] += .00019168853759765625
    r = qualify_trace(trace)
    assert r['qualified_for_pair']
    assert abs(r['encoder_tail']['median_drift_rad'][0]) > 0


def test_upper_sampling_does_not_require_feedback_rate():
    trace = with_gains(synthetic_trace())
    trace['samples']['upper'] = trace['samples']['upper'][::2]
    for i, row in enumerate(trace['samples']['upper']): row['sequence'] = i
    assert qualify_trace(trace)['qualified_for_pair']


@pytest.mark.parametrize('bad', [False, True])
def test_pair_gate_withholds_improvements_when_baseline_fails(tmp_path, bad):
    off = with_gains(synthetic(enabled=False))
    on = with_gains(synthetic(enabled=True, clock_offset=30_000_000_000))
    if bad: on['samples']['hal_arm'][90]['joints'][0]['stiffness'] = 50.
    paths = [tmp_path/'off.json', tmp_path/'on.json']
    for p, trace in zip(paths, [off, on]): p.write_text(json.dumps(trace))
    r = compare_qualified_files(*paths)
    assert r['legacy_pair_verdict'] == 'paired_numeric_comparison'
    assert (r['verdict'] == 'not_comparable') is bad
    assert ('qualified_numeric_comparison' in r) is not bad
    assert not r['repeatability_verified'] and not r['execution_authorized']
