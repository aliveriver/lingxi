"""Offline evidence review for the bounded gravity diagnostic, never actuation.

Recompute from source streams; missing/invalid evidence is inconclusive. A pair
is a descriptive numeric comparison, not causal proof or repeatability approval.
"""
from dataclasses import asdict
from bisect import bisect_left, bisect_right
import hashlib
import math
from pathlib import Path
from statistics import median

from .acceptance_report import AcceptanceCriteria, _finite_vector, _standing
from .gravity_preparation import review_preparation
from .gravity import StaticArmModel
from .gravity_diagnostic import (CALIBRATION_SHA256, MODEL_SHA256, LimitedGravityGuard,
                                 diagnostic_phases, diagnostic_protocol,
                                 limited_gravity_plan, validate_model_plan)
from .errors import SafetyInterlockError
from .drive_trace import review_drive_window
from .models import ARM_JOINT_NAMES
from .publication import summarize_frames
from .replay import strict_json

HYPOTHESES = tuple(f'{sensor}_{mount}' for sensor in ('chest_imu', 'pelvis_imu')
                   for mount in ('identity', 'factory', 'transpose'))
STREAMS = ('upper', 'hal_arm', 'arm_state', 'mc')
# An observed step, not a verified manufacturer encoder specification.
OBSERVED_ENCODER_STEP_RAD = .00019168853759765625


def preview_pair(urdf, baseline):
    model = StaticArmModel(urdf)
    if model.sha256 != MODEL_SHA256:
        raise ValueError('Diagnostic preview requires the audited PC2 model hash')
    plans = {}
    for enabled in (False, True):
        frames = list(limited_gravity_plan(baseline, compensation_enabled=enabled))
        validate_model_plan(model, frames)
        plans['on' if enabled else 'off'] = {
            'protocol': diagnostic_protocol(compensation_enabled=enabled), 'frames': frames}
    return {'kind': 'limited_gravity_pair_preview_v1', 'model_sha256': model.sha256,
            'baseline_command_rad': list(baseline), 'plans': plans,
            'gravity_source': 'no_live_feedback; hypothetical capped bias pending six-hypothesis qualification',
            'execution_authorized': False, 'hardware_acceptance_complete': False}


def evaluate_trace(trace):
    c = AcceptanceCriteria()
    quality, motion, windows = [], [], {}
    report = {'kind': 'limited_gravity_trial_review_v1', 'verdict': 'inconclusive',
              'criteria': asdict(c), 'quality_checks': quality, 'motion_checks': motion,
              'windows': windows, 'execution_authorized': False,
              'hardware_acceptance_complete': False, 'model_and_mounting_verified': False,
              'scope': 'one capped diagnostic; not repeatability, contact, or model-loop acceptance'}

    def check(name, passed, detail=None):
        quality.append({'name': name, 'passed': bool(passed), 'detail': detail})

    def metric(name, value, limit):
        motion.append({'name': name, 'measured_rad': value, 'maximum_rad': limit, 'passed': value <= limit})

    try:
        r = trace['result']
        if trace['kind'] != 'control_path_trace' or r['kind'] != 'limited_gravity_baseline_session':
            raise ValueError('Expected a limited-gravity control trace')
        bias_only = r['bias_only']
        enabled = r.get('compensation_requested', r['gravity_compensation_enabled'])
        protocol = diagnostic_protocol(bias_only=bias_only, compensation_enabled=enabled)
        baseline = r['baseline_command_rad']
        plan = list(limited_gravity_plan(baseline, bias_only=bias_only, compensation_enabled=enabled))
        report.update(protocol=protocol, baseline_command_rad=list(baseline),
                      model_sha256=r['model_sha256'], calibration_sha256=r['calibration_sha256'])
        check('physical_trial', r['dry_run'] is False and r['gravity_compensation_enabled'] is enabled)
        check('audited_inputs', r['model_sha256'] == MODEL_SHA256 and r['calibration_sha256'] == CALIBRATION_SHA256)
        check('protocol_metadata', r.get('diagnostic_protocol', protocol) == protocol
              and r['joint_index'] == 0 and r['delta_rad'] == protocol['delta_rad']
              and r['bias_limit_rad'] == .002 and r['assumed_stiffness_nm_rad'] == 40.
              and r['additional_payload_mass_kg'] == 0.)
        check('session_complete', not r.get('error') and not r.get('feedback_error'))
        check('trace_quality', trace['errors'] == [] and all(trace['dropped'].get(k) == 0 for k in STREAMS))
        check('gravity_capture_quality', r['final_gravity_snapshot']['errors'] == []
              and r['final_gravity_snapshot']['error_count'] == 0)
        phases = diagnostic_phases(bias_only=bias_only)
        expected = ['before_mode', *[p for p, _ in phases], 'recovery_complete', 'publishing_stopped', 'final']
        events = r['events']
        if [e['phase'] for e in events] != expected:
            raise ValueError('Missing, duplicate or unexpected phase events')
        times = [e['monotonic_ns'] for e in events]
        if any(type(t) is not int or t <= 0 for t in times) or any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError('Invalid or unordered phase clocks')
        by = {e['phase']: e for e in events}
        start, stop = by['baseline_hold']['monotonic_ns'], by['recovery_complete']['monotonic_ns']
        report.update(start_monotonic_ns=start, stop_monotonic_ns=stop)
        report['drive_evidence'] = (review_drive_window(r['drive_trace'], start, stop)
            if 'drive_trace' in r else {'raw_samples_available': False, 'drive_cause_identified': False,
                                       'reason': 'Legacy trial has no synchronous raw drive trace'})
        version = r.get('session_schema_version', 1)
        check('session_schema', type(version) is int and version in (1, 2))
        if version == 2 or 'preparation' in r:
            check('raw_drive_evidence', report['drive_evidence']['raw_samples_available'])
            report['preparation'] = review_preparation(r, start)
            check('preparation_verified', report['preparation']['verified'])
            check('no_upper_before_stable', not any(
                by['before_mode']['monotonic_ns'] <= row['received_monotonic_ns'] < start
                for row in trace['samples']['upper']))
        else:
            report['preparation'] = {'verified': False, 'reason': 'legacy trace lacks continuous preparation evidence'}
        phase_ends = {name: by[phases[i+1][0] if i+1 < len(phases) else 'recovery_complete']['monotonic_ns']
                      for i, (name, _) in enumerate(phases)}
        durations = {name: (phase_ends[name]-by[name]['monotonic_ns'])/1e9 for name, _ in phases}
        report['observed_phase_duration_s'] = durations
        check('phase_durations', all(duration-.05 <= durations[name] <= duration*2.5+.05 for name, duration in phases))
        active = {}
        for kind in STREAMS:
            rows = trace['samples'][kind]
            if not rows or any(type(row[k]) is not int or row[k] <= 0 for row in rows
                               for k in ('received_monotonic_ns', 'stamp_ns')):
                raise ValueError(f'Invalid {kind} clocks')
            for clock in ('received_monotonic_ns', 'stamp_ns'):
                check(f'{kind}_{clock}_order', all(b[clock] > a[clock] for a, b in zip(rows, rows[1:])))
            selected = [row for row in rows if start <= row['received_monotonic_ns'] < stop]
            if not selected:
                raise ValueError(f'No active {kind} samples')
            active[kind] = selected
            receive = [start, *[row['received_monotonic_ns'] for row in selected], stop]
            gap = max(b-a for a, b in zip(receive, receive[1:]))/1e9
            check(f'{kind}_coverage', gap <= (c.max_mc_gap_s if kind == 'mc' else c.max_stream_gap_s), gap)
        check('urs_standing_throughout', all(_standing(row, 'UPPERBODY_REMOTE_SPLIT') for row in active['mc']))
        final_time = by['final']['monotonic_ns']

        def fresh(state):
            t = state.get('timestamp', {}).get('monotonic_ns')
            # These snapshots are obtained sequentially around the final event.
            return type(t) is int and abs(final_time-t) <= 250_000_000

        final_mc, restored = by['final']['mc'], r.get('mode_restore', {}).get('mc_after', {})
        check('restored_standing', _standing(final_mc, 'STAND_DEFAULT', event=True)
              and _standing(restored, 'STAND_DEFAULT', event=True) and fresh(final_mc) and fresh(restored))
        final_arm = by['final']['arm']
        check('final_arm_faults_clear', fresh(final_arm)
              and tuple(j['name'] for j in final_arm['joints']) == ARM_JOINT_NAMES
              and all(j['fault_code'] == 0 for j in final_arm['joints']))
        hands = r['final_hands']
        check('final_hands_faults_clear', set(hands) == {'left', 'right'} and all(
            fresh(hands[s]) and len(hands[s]['joints']) == 10
            and all(j['fault_code'] == 0 for j in hands[s]['joints']) for s in ('left', 'right')))
        for kind in ('hal_arm', 'arm_state'):
            for row in active[kind]:
                if (tuple(j['name'] for j in row['joints']) != ARM_JOINT_NAMES
                        or any(not _finite_vector([j['position'], j['velocity'], j['effort']], 3) for j in row['joints'])):
                    raise ValueError(f'Incomplete, unordered or nonfinite {kind} feedback')
        check('arm_faults_clear', all(j['error_code'] == 0 for row in active['arm_state'] for j in row['joints']))
        upper = active['upper']
        report['source_time_span_ns'] = [upper[0]['stamp_ns'], upper[-1]['stamp_ns']]
        if any(not _finite_vector(row['arm_pos'], 14) or not _finite_vector(row['hand_pos'], 20) for row in upper):
            raise ValueError('Invalid upper command vector')
        check('upper_sequence', all(type(row['sequence']) is int and 0 <= row['sequence'] < 2**32 for row in upper)
              and all(b['sequence'] == (a['sequence']+1) % 2**32 for a, b in zip(upper, upper[1:])))
        check('hands_held', all(row['hand_sub_mode'] == 2 and max(abs(a-b) for a, b in
              zip(row['hand_pos'], upper[0]['hand_pos'])) <= c.command_tolerance_rad for row in upper))
        report['hand_command_rad'] = upper[0]['hand_pos']
        ceiling = baseline[0] + protocol['delta_rad'] + (.002 if enabled else 0.)
        for kind in ('upper', 'hal_arm'):
            vectors = [row['arm_pos'] if kind == 'upper' else [j['position'] for j in row['joints']] for row in active[kind]]
            check(f'{kind}_envelope', all(baseline[0]-1e-5 <= q[0] <= ceiling+1e-5
                  and max(abs(a-b) for a, b in zip(q[1:], baseline[1:])) <= 1e-5 for q in vectors))
        # Check ramps as well as settled medians. The source-clock tolerance is
        # an association window, NOT an estimate of transport latency/causality.
        stamps = [row['stamp_ns'] for row in upper]
        matches = []
        for row in active['hal_arm']:
            nearby = upper[bisect_left(stamps, row['stamp_ns']-50_000_000):
                           bisect_right(stamps, row['stamp_ns']+50_000_000)]
            q = [j['position'] for j in row['joints']]
            matches.append(any(max(abs(a-b) for a, b in zip(q, candidate['arm_pos'])) <= 1e-5
                               for candidate in nearby))
        check('hal_matches_nearby_upper_targets', all(matches),
              'every HAL sample matches an upper target within +/-50 ms source time; not causal latency measurement')

        decisions = r['decisions']
        if len(decisions) != len(plan) or type(r['physical_publish_count']) is not int or r['physical_publish_count'] != len(plan):
            raise ValueError('Not all planned frames were physically published')
        frames, gravities, ratios = [], {key: [] for key in HYPOTHESES}, []
        for i, (d, frame) in enumerate(zip(decisions, plan)):
            if (type(d['index']) is not int or d['index'] != i or d['phase'] != frame['phase']
                    or not _finite_vector(d['desired_rad'], 14) or not _finite_vector(d['command_rad'], 14)
                    or any(abs(a-b) > 1e-12 for key in ('desired_rad', 'command_rad') for a, b in zip(d[key], frame[key]))
                    or not _finite_vector([d['applied_bias_rad'], d['bias_fraction']], 2)
                    or abs(d['applied_bias_rad']-frame['applied_bias_rad']) > 1e-12
                    or abs(d['bias_fraction']-frame['bias_fraction']) > 1e-12):
                raise ValueError('Decision differs from the fixed diagnostic plan')
            receipt = d['receipt']
            t = receipt['monotonic_ns']
            if (type(t) is not int or not by[d['phase']]['monotonic_ns'] <= t < phase_ends[d['phase']]
                    or type(receipt['sequence']) is not int or not 0 <= receipt['sequence'] < 2**32):
                raise ValueError('Decision receipt outside its phase')
            frames.append(receipt)
            snap = d['snapshot']
            # Reconstruct ROS now from the snapshot's paired capture clocks.
            now_ros = snap['captured_ros_ns'] + t - snap['captured_monotonic_ns']
            LimitedGravityGuard.require_fresh(snap, t, now_ros)
            hypotheses = d['gravity']['hypotheses']
            if set(hypotheses) != set(HYPOTHESES):
                raise ValueError('Missing gravity hypotheses')
            for key, h in hypotheses.items():
                if (not _finite_vector([h['torque_nm'], h['unbounded_bias_rad'], h['clipped_bias_rad']], 3)
                        or not _finite_vector(h['gravity_torso_m_s2'], 3)
                        or abs(h['unbounded_bias_rad']-h['torque_nm']/40.) > 1e-12
                        or h['unbounded_bias_rad'] < .002-1e-12 or abs(h['clipped_bias_rad']-.002) > 1e-12):
                    raise ValueError('Inconsistent or unsupported gravity bias')
                gravities[key].append(h['gravity_torso_m_s2'])
                ratios.append(.002/h['unbounded_bias_rad'])
        timing = summarize_frames(frames, 50.)
        check('publication_timing', timing['sequence_continuous'] is True
              and timing['min_frame_interval_s'] > 0 and timing['max_frame_interval_s'] <= .05)
        report['publication'] = {k: v for k, v in timing.items() if k != 'frames'}
        # Match transport observations to local receipts, not just summary counts.
        observed = {row['sequence']: row for row in upper}
        check('upper_matches_decisions', len(observed) == len(plan) and all(
            receipt['sequence'] in observed and max(abs(a-b) for a, b in zip(
                observed[receipt['sequence']]['arm_pos'], d['command_rad'])) <= c.command_tolerance_rad
            for receipt, d in zip(frames, decisions)))
        report['gravity_summary'] = {
            'median_torso_m_s2': {key: [median(v[i] for v in rows) for i in range(3)] for key, rows in gravities.items()},
            'capped_to_model_bias_fraction_range': [min(ratios), max(ratios)],
            'note': 'model-relative cap only; not measured gravity cancellation; zero bias applied in off arm'}

        holds = [('baseline_hold', 0.), ('compensated_baseline_hold', .002 if enabled else 0.)]
        if not bias_only:
            holds += [('target_hold', .01+(.002 if enabled else 0.)),
                      ('compensated_recovery_hold', .002 if enabled else 0.)]
        holds += [('recovery_hold', 0.)]
        for name, offset in holds:
            end = phase_ends[name]
            begin = end-round(c.window_s*1e9)
            check(f'{name}_dwell', durations[name] >= c.min_dwell_s)
            window = {'start_monotonic_ns': begin, 'stop_monotonic_ns': end}
            for kind in ('hal_arm', 'arm_state'):
                rows = [row for row in active[kind] if begin <= row['received_monotonic_ns'] < end]
                if not rows:
                    raise ValueError(f'No {name}/{kind} window samples')
                check(f'{name}_{kind}_samples', len(rows) >= c.min_window_samples)
                columns = [[row['joints'][i]['position'] for row in rows] for i in range(14)]
                window[kind] = {'count': len(rows), 'positions_rad': [median(v) for v in columns],
                                'spread_rad': [max(v)-min(v) for v in columns]}
                if kind == 'hal_arm':
                    check(f'{name}_hal_delivered', all(abs(v-baseline[i]-(offset if i == 0 else 0.)) <= 1e-5
                          for i, values in enumerate(columns) for v in values))
            windows[name] = window
        q = {name: w['arm_state']['positions_rad'] for name, w in windows.items()}
        q0, qb, qr = (q[n][0] for n in ('baseline_hold', 'compensated_baseline_hold', 'recovery_hold'))
        metrics = {'bias_introduction_encoder_delta_rad': qb-q0, 'final_return_residual_rad': qr-q0,
                   'baseline_absolute_desired_error_rad': q0-baseline[0],
                   'bias_hold_absolute_desired_error_rad': qb-baseline[0],
                   'return_absolute_desired_error_rad': qr-baseline[0]}
        report['metrics'] = metrics
        metric('original_baseline_return_residual', abs(qr-q0), c.return_tolerance_rad)
        metric('nonselected_axis_drift', max(abs(row['joints'][i]['position']-q['baseline_hold'][i])
               for row in active['arm_state'] for i in range(1, 14)), c.other_joint_drift_rad)
        metric('settled_window_spread', max(v for w in windows.values() for v in w['arm_state']['spread_rad']), c.settled_spread_rad)
        if not bias_only:
            qt, qc = q['target_hold'][0], q['compensated_recovery_hold'][0]
            metrics.update(encoder_target_delta_from_compensated_baseline_rad=qt-qb,
                           tracking_fraction=(qt-qb)/.01, compensated_return_residual_rad=qc-qb,
                           target_absolute_desired_error_rad=qt-(baseline[0]+.01),
                           increment_following_error_rad=(qt-qb)-.01)
            metric('increment_following_error', abs((qt-qb)-.01), c.following_tolerance_rad)
            metric('absolute_desired_target_error', abs(qt-baseline[0]-.01), c.absolute_position_tolerance_rad)
            metric('compensated_return_residual', abs(qc-qb), c.return_tolerance_rad)
        if all(item['passed'] for item in quality):
            report['verdict'] = ('bias_only_diagnostic_complete' if bias_only else 'single_trial_pass_under_engineering_criteria') if all(
                item['passed'] for item in motion) else 'fail'
    except (KeyError, TypeError, ValueError, IndexError, AttributeError, ZeroDivisionError, SafetyInterlockError) as exc:
        check('trace_structure', False, str(exc))
    return report


def evaluate_file(path):
    path = Path(path)
    raw = path.read_bytes()
    return {'source_path': str(path), 'source_sha256': hashlib.sha256(raw).hexdigest(),
            **evaluate_trace(strict_json(raw.decode('utf-8')))}


def compare_files(control_path, treatment_path):
    # Accept raw traces only; a hand-edited summary cannot grant a passing pair.
    control, treatment = evaluate_file(control_path), evaluate_file(treatment_path)
    problems = []
    result = {'kind': 'limited_gravity_pair_review_v1', 'sessions': [control, treatment],
              'verdict': 'not_comparable', 'problems': problems,
              'execution_authorized': False, 'hardware_acceptance_complete': False,
              'repeatability_verified': False, 'compensation_effectiveness_proven': False,
              'scope': 'descriptive pair only; setup, actual gains, model validity and causal effect remain unverified'}
    if control['source_sha256'] == treatment['source_sha256']:
        problems.append('Duplicate source trace')
    if any(r['verdict'] == 'inconclusive' for r in (control, treatment)):
        problems.append('Both sessions require complete quality evidence')
        return result
    spans = sorted(r['source_time_span_ns'] for r in (control, treatment))
    if spans[0][1] >= spans[1][0]:
        problems.append('Source time intervals overlap; need separate sessions with consistent ROS clocks')
    for r, enabled in ((control, False), (treatment, True)):
        if r['protocol']['bias_only'] or r['protocol']['compensation_enabled'] is not enabled:
            problems.append('Need a full-motion off control followed by an on treatment input')
    if control['preparation'] != treatment['preparation']:
        # Timestamps naturally differ; policy and evidence generation must match.
        if any(control['preparation'].get(k) != treatment['preparation'].get(k) for k in ('verified', 'policy')):
            problems.append('Preparation policy or evidence generation differs')
    a, b = (dict(r['protocol']) for r in (control, treatment))
    a.pop('compensation_enabled'); b.pop('compensation_enabled')
    if a != b or any(control[k] != treatment[k] for k in ('model_sha256', 'calibration_sha256', 'criteria')):
        problems.append('Protocol, model, calibration or criteria differ')
    for field, limit in (('baseline_command_rad', 1e-5), ('hand_command_rad', 1e-5)):
        if max(abs(x-y) for x, y in zip(control[field], treatment[field])) > limit:
            problems.append(f'{field} differs')
    for phase, duration in control['observed_phase_duration_s'].items():
        other = treatment['observed_phase_duration_s'].get(phase, 0.)
        if abs(duration-other) > .1*max(duration, other):
            problems.append(f'Observed {phase} durations differ by more than 10%')
    if max(abs(x-y) for x, y in zip(control['windows']['baseline_hold']['arm_state']['positions_rad'],
                                    treatment['windows']['baseline_hold']['arm_state']['positions_rad'])) > .001:
        problems.append('Settled starting encoders differ by more than 0.001 rad')
    for key in HYPOTHESES:
        ga, gb = (r['gravity_summary']['median_torso_m_s2'][key] for r in (control, treatment))
        norms = math.sqrt(sum(x*x for x in ga)*sum(x*x for x in gb))
        if not norms or sum(x*y for x, y in zip(ga, gb))/norms < math.cos(math.radians(1)):
            problems.append(f'{key} median gravity differs by more than 1 degree')
    if problems:
        return result
    floor = 2*OBSERVED_ENCODER_STEP_RAD
    improvements = {}
    for key in ('increment_following_error_rad', 'target_absolute_desired_error_rad',
                'compensated_return_residual_rad', 'final_return_residual_rad'):
        before, after = control['metrics'][key], treatment['metrics'][key]
        reduction = abs(before)-abs(after)
        improvements[key] = {'control_rad': before, 'treatment_rad': after, 'absolute_error_reduction_rad': reduction,
                             'comparison': 'lower_beyond_resolution_floor' if reduction > floor else (
                                 'higher_beyond_resolution_floor' if reduction < -floor else 'unresolved_at_resolution_floor')}
    result.update(verdict='paired_numeric_comparison', improvements=improvements,
                  resolution_review_floor_rad=floor,
                  resolution_note='two observed encoder steps; heuristic, not statistical significance or calibrated precision')
    return result
