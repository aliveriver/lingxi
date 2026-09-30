"""Additional offline baseline gate; preserves frozen historical reviewers.

Engineering evidence criteria only. Never extends a trajectory or waits for
settling on hardware. Missing fields fail closed for a new paired comparison.
"""
import hashlib
import math
from pathlib import Path
from statistics import median

from .acceptance_report import evaluate_trace as fixed_review
from .gravity_trial_report import evaluate_trace as gravity_review, compare_files
from .models import ARM_JOINT_NAMES
from .replay import strict_json

CRITERIA = {'tail_duration_s': .4, 'half_window_s': .2, 'max_receive_gap_s': .05,
            'min_feedback_samples_per_half': 10, 'min_upper_samples_per_half': 5,
            'max_encoder_median_drift_rad': .000193,
            'max_encoder_tail_spread_rad': .000385, 'target_tolerance_rad': 1e-5,
            'expected_hal_stiffness': 40., 'expected_hal_damping': 2.}


def qualify_trace(trace):
    checks = []
    report = {'kind': 'offline_baseline_qualification_v1', 'qualified_for_pair': False,
              'criteria': dict(CRITERIA), 'criteria_authority': 'conservative project review heuristics; not manufacturer acceptance',
              'checks': checks, 'execution_authorized': False, 'hardware_validated': False,
              'scope': 'recorded terminal baseline and subsequent HAL gains; not asymptotic settling or current site qualification'}
    def check(name, passed, detail=None):
        checks.append({'name': name, 'passed': bool(passed), 'detail': detail})
    try:
        r = trace['result']
        if r['kind'] == 'fixed_command_baseline_session':
            review = fixed_review(trace)
            next_phase = 'target_ramp'
        elif r['kind'] == 'limited_gravity_baseline_session':
            review = gravity_review(trace)
            next_phase = 'bias_in'
        else:
            raise ValueError('Unsupported trace')
        report['original_verdict'] = review['verdict']
        check('original_source_quality', bool(review['quality_checks']) and all(c['passed'] for c in review['quality_checks']))
        if not checks[-1]['passed']: return report
        times = {e['phase']: e['monotonic_ns'] for e in r['events']}
        end, stop = times[next_phase], times['recovery_complete']
        start = end-400_000_000
        check('terminal_baseline_available', start >= times['baseline_hold'])
        baseline = r['baseline_command_rad']
        for stream in ('upper', 'hal_arm', 'arm_state'):
            rows = [row for row in trace['samples'][stream] if start <= row['received_monotonic_ns'] < end]
            clocks = [start, *[row['received_monotonic_ns'] for row in rows], end]
            check(f'{stream}_tail_coverage', max(b-a for a, b in zip(clocks, clocks[1:])) <= 50_000_000)
            halves = [[row for row in rows if lo <= row['received_monotonic_ns'] < hi]
                      for lo, hi in ((start, start+200_000_000), (start+200_000_000, end))]
            minimum = CRITERIA['min_upper_samples_per_half'] if stream == 'upper' else CRITERIA['min_feedback_samples_per_half']
            check(f'{stream}_half_sample_counts', all(len(h) >= minimum for h in halves),
                  {'counts': [len(h) for h in halves], 'minimum': minimum})
            if stream in ('upper', 'hal_arm'):
                # Entire initial hold must retain the fixed target, including entry.
                held = [row for row in trace['samples'][stream]
                        if times['baseline_hold'] <= row['received_monotonic_ns'] < end]
                vectors = [row['arm_pos'] if stream == 'upper' else [j['position'] for j in row['joints']] for row in held]
                check(f'{stream}_baseline_target_fixed', bool(vectors) and all(
                    max(abs(a-b) for a, b in zip(q, baseline)) <= 1e-5 for q in vectors))
            elif all(halves):
                medians = [[median(row['joints'][i]['position'] for row in half) for i in range(14)] for half in halves]
                drift = [b-a for a, b in zip(*medians)]
                spread = [max(row['joints'][i]['position'] for row in rows)-min(row['joints'][i]['position'] for row in rows) for i in range(14)]
                report['encoder_tail'] = {'half_medians_rad': medians, 'median_drift_rad': drift, 'spread_rad': spread,
                                          'start_monotonic_ns': start, 'stop_monotonic_ns': end}
                check('all_axis_encoder_median_drift', max(map(abs, drift)) <= CRITERIA['max_encoder_median_drift_rad'])
                check('all_axis_encoder_tail_spread', max(spread) <= CRITERIA['max_encoder_tail_spread_rad'])
        active = [row for row in trace['samples']['hal_arm'] if times['baseline_hold'] <= row['received_monotonic_ns'] < stop]
        changes, previous = [], None
        invalid = 0
        for row in active:
            gains = [(j.get('stiffness'), j.get('damping')) for j in row['joints']]
            valid = (tuple(j['name'] for j in row['joints']) == ARM_JOINT_NAMES and all(
                type(v) in (int, float) and math.isfinite(v) for pair in gains for v in pair))
            expected = valid and all(abs(k-40.) <= 1e-9 and abs(d-2.) <= 1e-9 for k, d in gains)
            if row['received_monotonic_ns'] >= start and not expected: invalid += 1
            if gains != previous:
                changes.append({'receive_s_from_baseline_event': (row['received_monotonic_ns']-times['baseline_hold'])/1e9,
                                'all_axis_gains_finite': valid, 'all_axes_expected_40_2': expected})
                previous = gains
        report['hal_gain_changes'] = changes
        check('expected_gains_from_terminal_baseline_through_recovery', bool(active) and invalid == 0,
              {'invalid_or_unexpected_samples': invalid})
        report['qualified_for_pair'] = all(c['passed'] for c in checks)
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        check('structure', False, str(exc))
    return report


def qualify_file(path):
    raw = Path(path).read_bytes()
    return {'source_path': str(path), 'source_sha256': hashlib.sha256(raw).hexdigest(),
            **qualify_trace(strict_json(raw.decode()))}


def compare_qualified_files(control, treatment):
    pair = compare_files(control, treatment)
    qualifications = [qualify_file(control), qualify_file(treatment)]
    problems = list(pair['problems'])
    if [r['source_sha256'] for r in pair['sessions']] != [r['source_sha256'] for r in qualifications]:
        problems.append('Source files changed between the two offline reviews')
    if any(not r['qualified_for_pair'] for r in qualifications):
        problems.append('Both traces must pass the additional terminal baseline and HAL gain review')
    result = {'kind': 'baseline_qualified_pair_review_v1', 'baseline_qualifications': qualifications,
              'verdict': 'not_comparable' if problems else pair['verdict'], 'problems': problems,
              'execution_authorized': False, 'hardware_acceptance_complete': False,
              'repeatability_verified': False, 'compensation_effectiveness_proven': False,
              'legacy_pair_verdict': pair['verdict']}
    # Do not expose improvement metrics as a qualified result when the new gate fails.
    if not problems:
        result['qualified_numeric_comparison'] = pair
    return result
