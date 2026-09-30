from copy import deepcopy
import hashlib
import json
import math

import pytest

from lingxi_x2.gravity_diagnostic import MODEL_SHA256, CALIBRATION_SHA256
from lingxi_x2.gravity_trial_report import evaluate_trace, evaluate_file, compare_files, preview_pair
from lingxi_x2.models import ARM_JOINT_NAMES
from test_gravity import urdf


def synthetic(*, enabled=True, bias_only=False, gain=1., residual=0., offset=0., clock_offset=0):
    """Independent analytic schedule and source streams, no planner invocation.

    Encoder follows desired motion with configurable gain, not biased command.
    These are test-only fixtures, never physical evidence or friction simulation.
    """
    baseline = [.4, 0., 0., -1.2, 0., 0., 0.]*2
    phases = [('baseline_hold', 50), ('bias_in', 100), ('compensated_baseline_hold', 50)]
    if not bias_only:
        phases += [('target_ramp', 100), ('target_hold', 50), ('recovery_ramp', 100), ('compensated_recovery_hold', 50)]
    phases += [('bias_out', 100), ('recovery_hold', 50)]
    count = sum(n for _, n in phases)
    start = 1_000_000_000+clock_offset
    end = start+count*20_000_000
    final = end+400_000_000
    def mc(action, t):
        return {'action': action, 'action_status': 100, 'fsm_state': 4, 'body_state': 1,
                'timestamp': {'monotonic_ns': t}}
    stand = mc('STAND_DEFAULT', final)
    events = [{'phase': 'before_mode', 'monotonic_ns': start-100_000_000}]
    samples = {k: [] for k in ('upper', 'hal_arm', 'arm_state', 'mc')}
    decisions = []
    index = 0
    for phase, n in phases:
        events.append({'phase': phase, 'monotonic_ns': start+index*20_000_000})
        for step in range(1, n+1):
            u = step/n
            s = 10*u**3-15*u**4+6*u**5
            movement = .01*(s if phase == 'target_ramp' else 1-s if phase == 'recovery_ramp' else float(phase == 'target_hold'))
            alpha = s if phase == 'bias_in' else 1-s if phase == 'bias_out' else float(phase in {
                'compensated_baseline_hold', 'target_ramp', 'target_hold', 'recovery_ramp', 'compensated_recovery_hold'})
            bias = .002*alpha if enabled else 0.
            desired = baseline.copy(); desired[0] += movement
            command = desired.copy(); command[0] += bias
            encoder = baseline.copy(); encoder[0] += gain*movement+offset+(residual if phase in {'compensated_recovery_hold', 'bias_out', 'recovery_hold'} else 0.)
            t = start+index*20_000_000
            stamp = {'received_monotonic_ns': t, 'stamp_ns': t+100_000_000_000}
            samples['upper'].append({**stamp, 'sequence': index, 'arm_pos': command, 'hand_pos': [0.]*20, 'hand_sub_mode': 2})
            # Independent HAL/encoder streams at 100 Hz, upper at 50 Hz.
            for half in range(2):
                hstamp = {k: v+half*10_000_000 for k, v in stamp.items()}
                for kind, vector in (('hal_arm', command), ('arm_state', encoder)):
                    samples[kind].append({**hstamp, 'joints': [
                        {'name': name, 'position': value, 'velocity': 0., 'effort': 0., 'error_code': 0}
                        for name, value in zip(ARM_JOINT_NAMES, vector)]})
            if index % 5 == 0:
                samples['mc'].append({**stamp, 'action': 'UPPERBODY_REMOTE_SPLIT', 'action_status': 100, 'fsm': 4, 'body': 1})
            snap = {'kind': 'gravity_readonly_snapshot', 'schema_version': 1,
                    'captured_monotonic_ns': t, 'captured_ros_ns': t+100_000_000_000, 'errors': [], 'error_count': 0,
                    'samples': {key: {**stamp, 'received_ros_ns': stamp['stamp_ns']} for key in
                                ('chest_imu', 'pelvis_imu', 'waist_state', 'arm_state')}}
            hypotheses = {f'{sensor}_{mount}': {'torque_nm': .4, 'unbounded_bias_rad': .01,
                         'clipped_bias_rad': .002, 'gravity_torso_m_s2': [0., 0., -9.81]}
                         for sensor in ('chest_imu', 'pelvis_imu') for mount in ('identity', 'factory', 'transpose')}
            decisions.append({'index': index, 'phase': phase, 'desired_rad': desired, 'command_rad': command,
                              'applied_bias_rad': bias, 'bias_fraction': alpha, 'snapshot': snap,
                              'gravity': {'hypotheses': hypotheses}, 'receipt': {'monotonic_ns': t, 'sequence': index}})
            index += 1
    events += [{'phase': 'recovery_complete', 'monotonic_ns': end},
               {'phase': 'publishing_stopped', 'monotonic_ns': end+300_000_000},
               {'phase': 'final', 'monotonic_ns': final, 'mc': stand,
                'arm': {'timestamp': {'monotonic_ns': final}, 'joints': [
                    {'name': name, 'fault_code': 0} for name in ARM_JOINT_NAMES]}}]
    return {'kind': 'control_path_trace', 'errors': [], 'dropped': dict.fromkeys(samples, 0), 'samples': samples,
            'result': {'kind': 'limited_gravity_baseline_session', 'bias_only': bias_only, 'dry_run': False,
                       'gravity_compensation_enabled': enabled, 'compensation_requested': enabled,
                       'joint_index': 0, 'delta_rad': 0. if bias_only else .01, 'bias_limit_rad': .002,
                       'assumed_stiffness_nm_rad': 40., 'additional_payload_mass_kg': 0.,
                       'model_sha256': MODEL_SHA256, 'calibration_sha256': CALIBRATION_SHA256,
                       'baseline_command_rad': baseline, 'events': events, 'decisions': decisions,
                       'physical_publish_count': count, 'final_gravity_snapshot': {'errors': [], 'error_count': 0},
                       'mode_restore': {'mc_after': stand}, 'final_hands': {side: {
                           'timestamp': {'monotonic_ns': final}, 'joints': [{'fault_code': 0}]*10} for side in ('left', 'right')}}}


def save(tmp_path, name, trace):
    path = tmp_path/name
    path.write_text(json.dumps(trace))
    return path


@pytest.mark.parametrize('enabled', [False, True])
def test_full_motion_verdict_uses_desired_not_biased_target(enabled):
    report = evaluate_trace(synthetic(enabled=enabled))
    assert report['verdict'] == 'single_trial_pass_under_engineering_criteria', report['quality_checks']
    assert report['metrics']['tracking_fraction'] == pytest.approx(1.)
    assert report['metrics']['target_absolute_desired_error_rad'] == pytest.approx(0.)
    assert not report['hardware_acceptance_complete'] and not report['execution_authorized']


def test_bias_only_is_never_a_position_acceptance_pass():
    report = evaluate_trace(synthetic(bias_only=True))
    assert report['verdict'] == 'bias_only_diagnostic_complete'
    assert 'tracking_fraction' not in report['metrics']
    assert not report['hardware_acceptance_complete']


@pytest.mark.parametrize('gain,residual,offset', [(.13, 0., 0.), (1., .002, 0.), (1., 0., -.006)])
def test_bad_encoder_motion_fails_even_when_commands_and_restore_succeed(gain, residual, offset):
    report = evaluate_trace(synthetic(gain=gain, residual=residual, offset=offset))
    assert report['verdict'] == 'fail'
    assert all(c['passed'] for c in report['quality_checks'])


@pytest.mark.parametrize('damage', ['missing_window', 'missing_phase', 'duplicate_phase', 'missing_stream',
    'clock', 'sequence', 'drop', 'fault', 'nan', 'joint_order', 'hands', 'restore', 'final_fault',
    'decision', 'receipt', 'stale', 'skew', 'hypothesis', 'count', 'model', 'dry_run', 'hal_window',
    'hal_other', 'timing', 'gravity_capture', 'upper_missing', 'inf_bias'])
def test_corrupt_evidence_is_inconclusive_without_crashing(damage):
    t = synthetic()
    r = t['result']
    if damage == 'missing_window': t['samples']['arm_state'] = t['samples']['arm_state'][:200]
    elif damage == 'missing_phase': r['events'].pop(2)
    elif damage == 'duplicate_phase': r['events'][2]['phase'] = 'baseline_hold'
    elif damage == 'missing_stream': t['samples']['mc'] = []
    elif damage == 'clock': t['samples']['arm_state'][200]['stamp_ns'] = 1
    elif damage == 'sequence': t['samples']['upper'][100]['sequence'] = 12
    elif damage == 'drop': t['dropped']['hal_arm'] = 1
    elif damage == 'fault': t['samples']['arm_state'][200]['joints'][0]['error_code'] = 2
    elif damage == 'nan': t['samples']['arm_state'][200]['joints'][0]['position'] = float('nan')
    elif damage == 'joint_order': t['samples']['hal_arm'][100]['joints'].reverse()
    elif damage == 'hands': t['samples']['upper'][100]['hand_pos'][0] = .01
    elif damage == 'restore': r['mode_restore'] = {}
    elif damage == 'final_fault': r['final_hands']['left']['joints'][0]['fault_code'] = 1
    elif damage == 'decision': r['decisions'][200]['desired_rad'][0] += .001
    elif damage == 'receipt': r['decisions'][200]['receipt']['monotonic_ns'] += 3_000_000_000
    elif damage == 'stale': r['decisions'][200]['snapshot']['samples']['chest_imu']['stamp_ns'] -= 101_000_000
    elif damage == 'skew': r['decisions'][200]['snapshot']['samples']['chest_imu']['stamp_ns'] -= 60_000_000
    elif damage == 'hypothesis': r['decisions'][200]['gravity']['hypotheses'].pop('chest_imu_factory')
    elif damage == 'count': r['physical_publish_count'] = 649
    elif damage == 'model': r['model_sha256'] = 'x'*64
    elif damage == 'dry_run': r['dry_run'] = True
    elif damage == 'hal_window': t['samples']['hal_arm'][690]['joints'][0]['position'] -= .001
    elif damage == 'hal_other': t['samples']['hal_arm'][100]['joints'][8]['position'] += .001
    elif damage == 'timing': r['decisions'][210]['receipt']['monotonic_ns'] += 35_000_000
    elif damage == 'gravity_capture': r['final_gravity_snapshot']['error_count'] = 1
    elif damage == 'upper_missing': t['samples']['upper'].pop(100)
    elif damage == 'inf_bias': r['decisions'][0]['applied_bias_rad'] = float('inf')
    report = evaluate_trace(t)
    assert report['verdict'] == 'inconclusive'
    assert any(not c['passed'] for c in report['quality_checks'])


def test_returns_remain_in_urs_and_other_axis_checks_use_all_samples():
    trace = synthetic(residual=.002)
    final_sample = deepcopy(trace['samples']['arm_state'][0])
    final_sample['received_monotonic_ns'] += 14_000_000_000
    final_sample['stamp_ns'] += 14_000_000_000
    trace['samples']['arm_state'].append(final_sample)
    report = evaluate_trace(trace)
    assert report['metrics']['final_return_residual_rad'] == pytest.approx(.002)
    assert report['metrics']['compensated_return_residual_rad'] == pytest.approx(.002)
    trace = synthetic()
    trace['samples']['arm_state'][101]['joints'][8]['position'] += .006
    report = evaluate_trace(trace)
    assert report['verdict'] == 'fail'
    assert next(c for c in report['motion_checks'] if c['name'] == 'nonselected_axis_drift')['passed'] is False


def test_pair_reports_improvement_separately_from_acceptance(tmp_path):
    off = save(tmp_path, 'off.json', synthetic(enabled=False, gain=.13))
    on = save(tmp_path, 'on.json', synthetic(gain=.3, clock_offset=20_000_000_000))
    r = compare_files(off, on)
    assert r['verdict'] == 'paired_numeric_comparison', r['problems']
    assert [s['verdict'] for s in r['sessions']] == ['fail', 'fail']
    assert r['improvements']['increment_following_error_rad']['comparison'] == 'lower_beyond_resolution_floor'
    assert not r['compensation_effectiveness_proven'] and not r['repeatability_verified']
    assert not r['execution_authorized']


def test_single_encoder_step_difference_is_unresolved(tmp_path):
    off = save(tmp_path, 'off.json', synthetic(enabled=False, gain=.13))
    on = save(tmp_path, 'on.json', synthetic(gain=.14916885375976563, clock_offset=20_000_000_000))
    r = compare_files(off, on)
    assert r['improvements']['increment_following_error_rad']['comparison'] == 'unresolved_at_resolution_floor'


@pytest.mark.parametrize('damage', ['wrong_order', 'bias_only', 'baseline', 'hands', 'gravity', 'initial', 'overlap'])
def test_unmatched_trials_are_not_compared(tmp_path, damage):
    control = synthetic(enabled=False)
    treatment = synthetic(clock_offset=20_000_000_000)
    if damage == 'wrong_order': control, treatment = treatment, control
    elif damage == 'bias_only': treatment = synthetic(bias_only=True, clock_offset=20_000_000_000)
    elif damage == 'baseline': treatment['result']['baseline_command_rad'][0] += .001
    elif damage == 'hands':
        for row in treatment['samples']['upper']: row['hand_pos'][0] = .001
    elif damage == 'gravity':
        for d in treatment['result']['decisions']:
            for h in d['gravity']['hypotheses'].values(): h['gravity_torso_m_s2'] = [9.81*math.sin(.03), 0., -9.81*math.cos(.03)]
    elif damage == 'initial': treatment = synthetic(offset=.002, clock_offset=20_000_000_000)
    elif damage == 'overlap': treatment = synthetic()
    off = save(tmp_path, 'off.json', control); on = save(tmp_path, 'on.json', treatment)
    result = compare_files(off, on)
    assert result['verdict'] == 'not_comparable' and result['problems']
    assert 'improvements' not in result


def test_duplicate_source_and_strict_json(tmp_path):
    p = save(tmp_path, 'trace.json', synthetic())
    assert compare_files(p, p)['verdict'] == 'not_comparable'
    assert evaluate_file(p)['source_sha256'] == hashlib.sha256(p.read_bytes()).hexdigest()
    for raw in ('{"kind":1,"kind":2}', '{"value":NaN}', '{"value":1e999}'):
        p.write_text(raw)
        with pytest.raises(ValueError): evaluate_file(p)


def test_offline_cli_no_client_ros_and_exclusive_output(tmp_path, monkeypatch, capsys, urdf):
    from lingxi_x2.cli import main
    def forbidden(*args, **kwargs): pytest.fail('Offline command touched client/ROS')
    monkeypatch.setattr('lingxi_x2.cli.X2Client', forbidden)
    monkeypatch.setattr('lingxi_x2.cli.reexec_with_ros_environment', forbidden)
    trace = save(tmp_path, 'trace.json', synthetic(bias_only=True))
    output = tmp_path/'review.json'
    assert main(['analyze-gravity-trial', str(trace), '--output', str(output)]) == 0
    saved = output.read_bytes()
    assert main(['analyze-gravity-trial', str(trace), '--output', str(output)]) == 2
    assert output.read_bytes() == saved
    assert main(['compare-gravity-trials', str(trace), str(trace), '--output', str(tmp_path/'pair.json')]) == 3
    baseline = save(tmp_path, 'baseline.json', [.4, 0., 0., -1.2, 0., 0., 0.]*2)
    # Production rejects a different model, even for preview. Test-only patch
    # then exercises the no-ROS branch without relying on ignored local assets.
    assert main(['plan-gravity-comparison', '--urdf', str(urdf), '--baseline', str(baseline), '--output', str(tmp_path/'preview.json')]) == 2
    monkeypatch.setattr('lingxi_x2.gravity_trial_report.MODEL_SHA256', hashlib.sha256(urdf.read_bytes()).hexdigest())
    assert main(['plan-gravity-comparison', '--urdf', str(urdf), '--baseline', str(baseline), '--output', str(tmp_path/'preview.json')]) == 0
    preview = json.loads((tmp_path/'preview.json').read_text())
    assert len(preview['plans']['off']['frames']) == len(preview['plans']['on']['frames']) == 650
    assert not preview['execution_authorized']


def test_hal_wrong_ramp_is_rejected_even_if_envelope_and_holds_are_correct():
    trace = synthetic()
    # During the target ramp, hold a legal but wrong target; all dwells survive.
    for row in trace['samples']['hal_arm'][470:490]:
        row['joints'][0]['position'] = .402
    report = evaluate_trace(trace)
    assert report['verdict'] == 'inconclusive'
    assert next(c for c in report['quality_checks'] if c['name'] == 'hal_matches_nearby_upper_targets')['passed'] is False


def test_matched_nominal_schedule_with_different_actual_timing_is_rejected(tmp_path):
    control = synthetic(enabled=False)
    treatment = synthetic(clock_offset=20_000_000_000)
    # Dilate monotonic/source clocks equally. Each trial remains under all
    # per-frame limits, but one takes 20% longer and cannot be a matched pair.
    def stretch(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key.endswith('_ns') and type(child) is int:
                    value[key] = round(child*1.2)
                else: stretch(child)
        elif isinstance(value, list):
            for child in value: stretch(child)
    treatment = json.loads(json.dumps(treatment))  # File input has no shared dict aliases.
    stretch(treatment)
    assert evaluate_trace(treatment)['verdict'] == 'single_trial_pass_under_engineering_criteria'
    off = save(tmp_path, 'off.json', control); on = save(tmp_path, 'on.json', treatment)
    report = compare_files(off, on)
    assert report['verdict'] == 'not_comparable'
    assert any('durations' in p for p in report['problems'])
