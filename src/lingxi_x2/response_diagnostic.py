"""Descriptive historical response analysis; no control or parameter fitting.

Receive timestamps delimit phases. Source timestamps only associate targets;
neither clock establishes physical actuator latency or a friction threshold.
"""
from bisect import bisect_left, bisect_right
from collections import Counter
import hashlib
import math
from pathlib import Path
from statistics import median

from .acceptance_report import evaluate_trace as fixed_review
from .gravity_trial_report import evaluate_trace as gravity_review, OBSERVED_ENCODER_STEP_RAD
from .replay import strict_json


def value_summary(values):
    if not values or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError('Expected nonempty finite numeric observations')
    levels = sorted(set(values))
    gaps = [b-a for a, b in zip(levels, levels[1:])]
    return {'count': len(values), 'median': median(values), 'min': min(values),
            'max': max(values), 'spread': max(values)-min(values),
            'distinct_levels': len(levels),
            'adjacent_level_gap_range': [min(gaps), max(gaps)] if gaps else None,
            'adjacent_level_gap_histogram_rounded_9dp': dict(sorted(Counter(
                f'{gap:.9f}' for gap in gaps).items())),
            'unchanged_adjacent_sample_fraction': (
                sum(a == b for a, b in zip(values, values[1:]))/(len(values)-1)
                if len(values) > 1 else None)}


def crossing_interval(rows, baseline, direction, threshold, start, stop, value):
    """Bracket first threshold crossing on the receive clock, or leave it unknown.

    First sample already beyond threshold is left censored; absent crossing is
    right censored. A single crossing is not a sustained response criterion.
    """
    previous = None
    for row in rows:
        t = row['received_monotonic_ns']
        if t >= stop:
            break
        if t < start:
            continue
        if direction*(value(row)-baseline) >= threshold:
            return {'previous_below_threshold_s': (previous-start)/1e9 if previous is not None else None,
                    'first_at_or_above_threshold_s': (t-start)/1e9,
                    'left_censored': previous is None, 'right_censored': False}
        previous = t
    return {'previous_below_threshold_s': (previous-start)/1e9 if previous is not None else None,
            'first_at_or_above_threshold_s': None,
            'left_censored': False, 'right_censored': True}


def analyze_trace(trace):
    result = trace['result']
    kind = result['kind']
    if kind == 'fixed_command_baseline_session':
        review = fixed_review(trace)
        ramp, hold, back = 'target_ramp', 'target_hold', 'recovery_ramp'
        delta = result['delta_rad']
        profile = result.get('trajectory_profile', 'linear')
    elif kind == 'limited_gravity_baseline_session' and result.get('bias_only') is True:
        review = gravity_review(trace)
        ramp, hold, back = 'bias_in', 'compensated_baseline_hold', 'bias_out'
        delta = .002 if result['gravity_compensation_enabled'] else 0.
        profile = 'quintic_bias_only'
    else:
        raise ValueError('Supports fixed baseline and bias-only historical traces only')
    if not review['quality_checks'] or not all(c['passed'] for c in review['quality_checks']):
        failed = [c['name'] for c in review['quality_checks'] if not c['passed']]
        raise ValueError(f'Trace quality insufficient: {failed}')
    index = result['joint_index']
    events = result['events']
    by = {e['phase']: e['monotonic_ns'] for e in events}
    start, stop = by['baseline_hold'], by['recovery_complete']
    active = {k: [r for r in trace['samples'][k] if start <= r['received_monotonic_ns'] < stop]
              for k in ('upper', 'hal_arm', 'arm_state')}
    q = lambda row: row['joints'][index]['position']
    u = lambda row: row['arm_pos'][index]
    phases = {}
    for event, following in zip(events[1:], events[2:]):
        name, begin, end = event['phase'], event['monotonic_ns'], following['monotonic_ns']
        if begin >= stop:
            break
        rows = [r for r in active['arm_state'] if begin <= r['received_monotonic_ns'] < end]
        phase = {'duration_s': (end-begin)/1e9,
                 'encoder_rad': value_summary([q(r) for r in rows]),
                 'reported_effort': value_summary([r['joints'][index]['effort'] for r in rows])}
        hal = [r for r in active['hal_arm'] if begin <= r['received_monotonic_ns'] < end]
        phase['hal_stiffness_damping_pairs'] = sorted({
            (r['joints'][index].get('stiffness'), r['joints'][index].get('damping')) for r in hal}, key=str)
        if name in ('baseline_hold', hold, 'recovery_hold'):
            first = [q(r) for r in rows if r['received_monotonic_ns'] < begin+200_000_000]
            last = [q(r) for r in rows if r['received_monotonic_ns'] >= end-200_000_000]
            if len(first) < 10 or len(last) < 10:
                raise ValueError(f'Insufficient first/last dwell samples: {name}')
            phase.update(first_200ms=value_summary(first), last_200ms=value_summary(last),
                         last_minus_first_median_rad=median(last)-median(first))
        phases[name] = phase
    baseline = phases['baseline_hold']['last_200ms']['median']
    target = phases[hold]['last_200ms']['median']
    recovery = phases['recovery_hold']['last_200ms']['median']
    # Match complete HAL vectors to nearby upper commands; never nearest-neighbor
    # interpolation or inferred one-to-one delivery/latency from repeated targets.
    upper = active['upper']
    stamps = [r['stamp_ns'] for r in upper]
    unmatched = 0
    for row in active['hal_arm']:
        near = upper[bisect_left(stamps, row['stamp_ns']-50_000_000):
                     bisect_right(stamps, row['stamp_ns']+50_000_000)]
        matched = any(max(abs(j['position']-v) for j, v in zip(row['joints'], candidate['arm_pos'])) <= 1e-5
                      for candidate in near)
        unmatched += not matched
    direction = 1 if delta >= 0 else -1
    threshold = OBSERVED_ENCODER_STEP_RAD/2
    crossing = {}
    for stream, getter, base in [('upper', u, result['baseline_command_rad'][index]),
                                 ('hal_arm', q, result['baseline_command_rad'][index]),
                                 ('arm_state', q, baseline)]:
        crossing[stream] = crossing_interval(active[stream], base, direction, threshold,
                                             by[ramp], by[back], getter)
    gains = sorted({(r['joints'][index].get('stiffness'), r['joints'][index].get('damping'))
                    for r in active['hal_arm']}, key=str)
    gain_changes, previous_gain = [], None
    for row in active['hal_arm']:
        gain = (row['joints'][index].get('stiffness'), row['joints'][index].get('damping'))
        if gain != previous_gain:
            gain_changes.append({'stiffness_damping': gain, 'source_stamp_ns': row['stamp_ns'],
                                 'receive_s_from_baseline_event': (row['received_monotonic_ns']-start)/1e9})
            previous_gain = gain
    return {'source_review': review, 'trajectory_profile': profile, 'joint_index': index,
            'desired_excursion_rad': result['delta_rad'], 'sent_excursion_rad': delta,
            'baseline_command_rad': result['baseline_command_rad'], 'phases': phases,
            'active_encoder_rad': value_summary([q(r) for r in active['arm_state']]),
            'hal_stiffness_damping_pairs': gains,
            'hal_gain_observations': gain_changes,
            'hal_target_association': {'tolerance_rad': 1e-5, 'source_window_s': .05,
                                       'sample_count': len(active['hal_arm']), 'unmatched_count': unmatched},
            'metrics': {'baseline_encoder_rad': baseline, 'target_encoder_rad': target,
                        'recovery_encoder_rad': recovery, 'encoder_increment_rad': target-baseline,
                        'urs_return_residual_rad': recovery-baseline,
                        'encoder_increment_over_sent_excursion': (target-baseline)/delta if delta else None},
            'first_threshold_crossing': {'threshold_rad': threshold,
                'origin': ramp, 'clock': 'PC2 receive monotonic relative to phase event',
                'meaning': 'first observed crossing, not sustained onset, transport or physical latency',
                'streams': crossing}}


def analyze_files(paths):
    sessions, seen = [], set()
    for path in paths:
        raw = Path(path).read_bytes()
        sha = hashlib.sha256(raw).hexdigest()
        if sha in seen:
            raise ValueError('Duplicate source trace')
        seen.add(sha)
        sessions.append({'source_path': str(path), 'source_sha256': sha,
                         **analyze_trace(strict_json(raw.decode('utf-8')))})
    if not sessions:
        raise ValueError('At least one trace required')
    return {'kind': 'historical_response_diagnostic_v1', 'sessions': sessions,
            'robot_connected': False, 'hardware_validated': False,
            'execution_authorized': False, 'combined_model_ready': False,
            'causal_identification': False, 'repeatability_accepted': False,
            'limitations': [
                'Different signed amplitudes, profiles, initial poses and hands are separate observations, not repetitions or an on/off pair.',
                'Dwell changes describe only the recorded interval; no asymptotic settling conclusion.',
                'Distinct encoder levels are observed output quantization, not a manufacturer encoder specification.',
                'Receive clocks include scheduling and transport; source stamps are not verified synchronized measurement times.',
                'Reported effort is an uncalibrated feedback field, not identified external torque.',
                'No friction/deadband model, causal parameter fit, compensation recommendation or actuator command is generated.']}
