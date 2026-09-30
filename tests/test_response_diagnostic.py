import json
import subprocess
import sys

import pytest

from lingxi_x2.response_diagnostic import analyze_trace, analyze_files, crossing_interval, value_summary
from test_acceptance_report import synthetic_trace
from test_gravity_trial_report import synthetic


def test_distinct_levels_are_not_number_of_transitions_or_encoder_spec():
    s = value_summary([0., .25, .25, .75, .25])
    assert s['distinct_levels'] == 3
    assert s['adjacent_level_gap_range'] == [.25, .5]
    assert s['unchanged_adjacent_sample_fraction'] == .25
    assert value_summary([1., 1.])['adjacent_level_gap_range'] is None


@pytest.mark.parametrize('values', [[], [float('nan')], [float('inf')], [True]])
def test_invalid_observations_rejected(values):
    with pytest.raises(ValueError):
        value_summary(values)


def test_crossing_excludes_entry_transient_and_brackets_observation():
    rows = [{'received_monotonic_ns': t*1_000_000_000, 'q': q}
            for t, q in [(1, 5.), (2, 0.), (3, .1), (4, .6), (5, .7)]]
    c = crossing_interval(rows, 0., 1, .5, 2_000_000_000, 5_000_000_000, lambda r: r['q'])
    assert c == {'previous_below_threshold_s': 1., 'first_at_or_above_threshold_s': 2.,
                 'left_censored': False, 'right_censored': False}
    c = crossing_interval(rows, 0., -1, .5, 2_000_000_000, 5_000_000_000, lambda r: r['q'])
    assert c['right_censored'] and c['first_at_or_above_threshold_s'] is None
    c = crossing_interval(rows, 0., 1, .5, 1_000_000_000, 5_000_000_000, lambda r: r['q'])
    assert c['left_censored'] and c['previous_below_threshold_s'] is None


def test_failed_motion_still_described_without_becoming_acceptance():
    r = analyze_trace(synthetic_trace(gain=.1, residual=.003))
    assert r['source_review']['verdict'] == 'fail'
    assert r['metrics']['encoder_increment_rad'] == pytest.approx(.002)
    assert r['metrics']['urs_return_residual_rad'] == pytest.approx(.003)


def test_dwell_drift_and_gain_transition_are_kept_separate():
    trace = synthetic_trace(gain=.1)
    # Input fixtures share some dictionaries; JSON round trip matches disk semantics.
    trace = json.loads(json.dumps(trace))
    for row in trace['samples']['arm_state']:
        t = row['received_monotonic_ns']/1e9
        if 3 <= t < 3.2:
            row['joints'][0]['position'] -= .001
    for row in trace['samples']['hal_arm']:
        early = row['received_monotonic_ns'] < 1_030_000_000
        row['joints'][0].update(stiffness=50. if early else 40., damping=3. if early else 2.)
    r = analyze_trace(trace)
    assert r['phases']['target_hold']['last_minus_first_median_rad'] == pytest.approx(.001)
    assert len(r['hal_gain_observations']) == 2
    assert r['hal_gain_observations'][1]['receive_s_from_baseline_event'] == .03
    assert r['phases']['target_hold']['hal_stiffness_damping_pairs'] == [(40., 2.)]


def test_bias_only_never_renamed_desired_motion():
    r = analyze_trace(synthetic(bias_only=True))
    assert r['desired_excursion_rad'] == 0.
    assert r['sent_excursion_rad'] == .002
    assert r['first_threshold_crossing']['streams']['arm_state']['right_censored']


def test_off_bias_only_has_no_nonzero_command_excursion_or_ratio():
    r = analyze_trace(synthetic(bias_only=True, enabled=False))
    assert r['sent_excursion_rad'] == 0.
    assert r['metrics']['encoder_increment_over_sent_excursion'] is None


def test_bad_quality_refuses_numeric_report():
    trace = synthetic_trace()
    trace['dropped']['arm_state'] = 1
    with pytest.raises(ValueError, match='quality insufficient'):
        analyze_trace(trace)


def test_bad_ramp_target_association_remains_visible():
    trace = json.loads(json.dumps(synthetic_trace()))
    trace['samples']['hal_arm'][150]['joints'][0]['position'] = .4
    r = analyze_trace(trace)
    assert r['hal_target_association']['unmatched_count'] > 0


def test_source_hash_duplicate_rejection_and_no_robot_initialization(tmp_path):
    path = tmp_path/'synthetic.json'
    raw = json.dumps(synthetic_trace()).encode()
    path.write_bytes(raw)
    r = analyze_files([path])
    import hashlib
    assert r['sessions'][0]['source_sha256'] == hashlib.sha256(raw).hexdigest()
    assert not r['robot_connected'] and not r['execution_authorized'] and not r['repeatability_accepted']
    with pytest.raises(ValueError, match='Duplicate'):
        analyze_files([path, path])
    # Package __init__ exports the client class; forbid construction and ROS.
    code = '''
import sys
class Block:
    def find_spec(self, fullname, *args):
        if fullname == 'rclpy' or fullname.startswith('lingxi_x2.backends.ros2'):
            raise RuntimeError('Robot import forbidden')
sys.meta_path.insert(0, Block())
from lingxi_x2.client import X2Client
def forbidden(*args, **kwargs):
    raise RuntimeError('Client initialization forbidden')
X2Client.__init__ = forbidden
from lingxi_x2.response_diagnostic import analyze_files
analyze_files([sys.argv[1]])
'''
    result = subprocess.run([sys.executable, '-c', code, str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_cli_existing_output_preserved_even_with_missing_input(tmp_path):
    path = tmp_path/'old.json'
    path.write_bytes(b'historical evidence')
    result = subprocess.run([sys.executable, 'scripts/analyze_historical_response.py',
                             '--output', str(path), str(tmp_path/'missing.json')], capture_output=True, text=True)
    assert result.returncode != 0
    assert 'Output already exists' in result.stderr
    assert path.read_bytes() == b'historical evidence'
