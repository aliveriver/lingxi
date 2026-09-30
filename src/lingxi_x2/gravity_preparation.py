"""Receive every preparation sample; fixed references survive window resets.

These conservative engineering gates are not hardware qualification. No command,
mode change, fault reset, or recovery motion is performed by this module.
"""
from collections import deque
from copy import deepcopy
import math
import threading
import time

from .control_trace import trace_sample
from .errors import SafetyInterlockError
from .gravity_capture import TOPICS as GRAVITY_TOPICS, gravity_sample
from .gravity_diagnostic import static_imu, StaticSettlingError, _angle, _vec
from .gravity_observation import WAIST_NAMES
from .models import ARM_JOINT_NAMES, HAND_JOINT_SUFFIXES
from .safety import HAND_PLAUSIBILITY_RAD, validate_hand_target

HAND_NAMES = tuple(f'{side}_{name}' for side in ('L', 'R') for name in HAND_JOINT_SUFFIXES)
TOPICS = {**GRAVITY_TOPICS,
          'hands': ('/aima/hal/joint/hand/state', 'HandStateArray'),
          'hal_arm': ('/aima/hal/joint/arm/command', 'JointCommandArray'),
          'mc': ('/aima/mc/common/state', 'McCommonState')}
POLICY = {'version': 4, 'stable_duration_s': 1., 'timeout_s': 10.,
          'max_sample_gap_s': .1, 'max_mc_gap_s': .25,
          'min_samples': 20, 'min_mc_samples': 3,
          'joint_spread_rad': .001, 'hand_spread_rad': .005, 'joint_velocity_rad_s': .02,
          'waist_excursion_rad': .005, 'hand_excursion_rad': .005,
          'hand_absolute_plausibility_rad': HAND_PLAUSIBILITY_RAD,
          'selected_arm_excursion_rad': .025, 'other_arm_excursion_rad': .005,
          'hal_tolerance_rad': 1e-5, 'imu_acceleration_spread_m_s2': .3,
          'imu_direction_spread_rad': math.radians(.5)}


def preparation_sample(kind, message, mono, ros):
    if kind in GRAVITY_TOPICS:
        return gravity_sample(kind, message, mono, ros)
    if kind in ('hal_arm', 'mc'):
        row = trace_sample(kind, message)
        row.update(received_monotonic_ns=mono, received_ros_ns=ros)
        if kind == 'mc':
            row['player'] = int(message.motion_status.player_state.value)
        return row
    if kind != 'hands':
        raise ValueError('Unknown preparation stream')
    row = {'received_monotonic_ns': mono, 'received_ros_ns': ros,
           'stamp_ns': int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec),
           'joints': [], 'hand_types': []}
    for side, prefix in (('left', 'L'), ('right', 'R')):
        row['hand_types'].append(int(getattr(message, f'{side}_hand_type').value))
        joints = getattr(message, f'{side}_hands')
        if len(joints) != 10:
            raise ValueError('Incomplete O10 hand array')
        for name, j in zip(HAND_JOINT_SUFFIXES, joints):
            row['joints'].append({'name': str(getattr(j, 'name', '') or f'{prefix}_{name}'),
                'position': float(j.position), 'velocity': float(j.velocity),
                'error_code': int(getattr(j, 'faultcode', getattr(j, 'error_code', -1)))})
    return row


class PreparationCapture:
    """Bounded drainable queue: never silently replace/drop an intervening fault."""
    def __init__(self, capacity=30000):
        self.lock = threading.Lock()
        self.queue = deque()
        self.capacity = capacity
        self.error = None

    def receive(self, kind, message, *, mono, ros):
        with self.lock:
            try:
                row = preparation_sample(kind, message, mono, ros)
                if 'joints' in row:
                    for joint in row['joints']:
                        _vec((joint['position'], joint['velocity']), 2)
                if len(self.queue) >= self.capacity:
                    raise ValueError('Preparation queue overflow')
                self.queue.append({'stream': kind, 'sample': row})
            except Exception as exc:
                self.error = f'{kind}: {exc}'

    def drain(self):
        with self.lock:
            rows = list(self.queue)
            self.queue.clear()
            return {'rows': rows, 'error': self.error}


def _positions(row, names, *, faults=True):
    joints = row['joints']
    if tuple(j['name'] for j in joints) != names:
        raise SafetyInterlockError('Missing, duplicate or reordered preparation joints')
    for j in joints:
        _vec((j['position'], j['velocity']), 2)
        if faults and (type(j.get('error_code')) is not int or j['error_code'] != 0):
            raise SafetyInterlockError(f"Joint fault during preparation/trajectory: {j['name']}")
    return tuple(j['position'] for j in joints)


def fixed_references(samples):
    """Called once before mode entry; never called after a settling reset."""
    result = {kind: list(_positions(samples[kind], names)) for kind, names in (
        ('arm_state', ARM_JOINT_NAMES), ('waist_state', WAIST_NAMES), ('hands', HAND_NAMES))}
    validate_preparation_hands(result['hands'])
    result['hal_gains'] = [list(_vec((j['stiffness'], j['damping']), 2)) for j in samples['hal_arm']['joints']]
    if len(result['hal_gains']) != 14:
        raise SafetyInterlockError('Incomplete initial HAL gains')
    return result


def validate_preparation_hands(values):
    """Reject grossly implausible feedback before it can become a hold target.

    This symmetric +/- pi diagnostic envelope is deliberately broader than
    every angle in the O10 manual section 2.4. It is NOT calibrated joint limits
    or proof of correct zero/sign; values inside still need all other gates.
    Never clamp, wrap, or substitute a guessed hand position.
    """
    values = _vec(values, 20)
    for name, value in zip(HAND_NAMES, values):
        if abs(value) > POLICY['hand_absolute_plausibility_rad']:
            raise SafetyInterlockError(f'Implausible hand hold position: {name}={value:.9f} rad')
    validate_hand_target(values[:10])
    validate_hand_target(values[10:])
    return values


class PreparationMonitor:
    def __init__(self, baseline, references, *, started_ns, action):
        self.baseline = _vec(baseline, 14)
        self.references = deepcopy(references)
        for key, size in (('arm_state', 14), ('waist_state', 3), ('hands', 20)):
            _vec(self.references[key], size)
        validate_preparation_hands(self.references['hands'])
        self.started_ns = started_ns
        self.action = action
        self.latest = {}
        self.windows = {key: deque() for key in TOPICS}
        self.extrema = {key: [] for key in TOPICS}
        self.reset_count = 0
        self.last_reason = 'awaiting samples'
        self.imu_reference = {}
        self.ready = False
        self.urs_gains_confirmed = False

    def begin_preparation(self, now_ns, action):
        self.started_ns, self.action = now_ns, action
        self.reset('mode entry')

    def reset(self, reason):
        self.ready = False
        self.reset_count += 1
        self.last_reason = reason
        for key, window in self.windows.items():
            window.clear()
            self.extrema[key] = []

    def _sample(self, key, row, *, active, allow_gain_transition=False):
        p = POLICY
        if key.endswith('imu'):
            g, checks = static_imu(row)
            if key in self.imu_reference and _angle(g, self.imu_reference[key]) > math.radians(1):
                raise SafetyInterlockError('IMU direction moved beyond fixed pre-mode reference')
            self.imu_reference.setdefault(key, g)
            return [*g, checks['acceleration_norm']]
        if key == 'mc':
            if row['body'] != 1 or row['fsm'] != 4 or row['action'] not in ('STAND_DEFAULT', 'UPPERBODY_REMOTE_SPLIT'):
                raise SafetyInterlockError('MC left standing preparation envelope')
            if row['action'] != self.action or row['action_status'] != 100 or row['player'] != 0:
                raise StaticSettlingError('MC preparation not idle/running in expected mode')
            return []
        names = {'arm_state': ARM_JOINT_NAMES, 'waist_state': WAIST_NAMES,
                 'hands': HAND_NAMES, 'hal_arm': ARM_JOINT_NAMES}[key]
        q = _positions(row, names, faults=key != 'hal_arm')
        if key == 'hands':
            validate_preparation_hands(q)
        if key == 'hands' and row['hand_types'] != [1, 1]:
            raise SafetyInterlockError('Expected two O10 hands')
        if key == 'hal_arm':
            gains = [_vec((j['stiffness'], j['damping']), 2) for j in row['joints']]
            original_gains = all(abs(a-b) <= 1e-6 for pair,ref in zip(gains,self.references['hal_gains']) for a,b in zip(pair,ref))
            urs_gains = all(abs(k-40.) <= 1e-6 and abs(d-2.) <= 1e-6 for k,d in gains)
            if active and self.action == 'UPPERBODY_REMOTE_SPLIT':
                if urs_gains:
                    self.urs_gains_confirmed = True
                elif not (allow_gain_transition and not self.urs_gains_confirmed and original_gains):
                    raise StaticSettlingError('HAL gains have not reached URS 40/2 or regressed')
            elif not original_gains:
                raise StaticSettlingError('HAL preparation gains changed from original fixed baseline')
            if active:
                if not self.baseline[0]-1e-5 <= q[0] <= self.baseline[0]+.012+1e-5 or any(
                        abs(a-b) > p['hal_tolerance_rad'] for a, b in zip(q[1:], self.baseline[1:])):
                    raise SafetyInterlockError('HAL targets left fixed diagnostic envelope')
            elif any(abs(a-b) > p['hal_tolerance_rad'] for a, b in zip(q, self.baseline)):
                raise StaticSettlingError('HAL targets have not returned to original baseline')
            return list(q)
        limits = ([p['selected_arm_excursion_rad'], *([p['other_arm_excursion_rad']]*13)] if key == 'arm_state'
                  else [p['hand_excursion_rad'] if key == 'hands' else p['waist_excursion_rad']]*len(q))
        if any(abs(a-b) > limit for a, b, limit in zip(q, self.references[key], limits)):
            raise SafetyInterlockError(f'{key} excursion beyond fixed pre-mode reference')
        # During trajectory, the commanded shoulder may move; all other axes
        # and the hands/waist must retain the static velocity gate.
        checked = row['joints'][1:] if active and key == 'arm_state' else row['joints']
        # Official AIMDK marks OmniHand velocity as unadapted raw feedback.
        # Its continuous position spread is the hand stationarity measurement.
        if key != 'hands' and any(abs(j['velocity']) > p['joint_velocity_rad_s'] for j in checked):
            raise StaticSettlingError(f'{key} still moving')
        return list(q)

    def consume(self, batch, *, now_ns, now_ros, active=False, allow_gain_transition=False):
        if batch['error']:
            raise SafetyInterlockError(batch['error'])
        for item in batch['rows']:
            key, row = item['stream'], item['sample']
            if key not in TOPICS:
                raise SafetyInterlockError('Unexpected preparation stream')
            t, stamp, received_ros = (row[k] for k in ('received_monotonic_ns', 'stamp_ns', 'received_ros_ns'))
            if any(type(v) is not int or v <= 0 for v in (t, stamp, received_ros)) or t > now_ns:
                raise SafetyInterlockError('Invalid preparation timestamp')
            limit = round((POLICY['max_mc_gap_s'] if key == 'mc' else POLICY['max_sample_gap_s'])*1e9)
            if not 0 <= received_ros-stamp <= limit or abs((now_ns-t)-(now_ros-received_ros)) > 50_000_000:
                raise SafetyInterlockError('Inconsistent preparation source/receive clocks')
            previous = self.latest.get(key)
            if previous and (t <= previous['received_monotonic_ns'] or stamp <= previous['stamp_ns']
                             or t-previous['received_monotonic_ns'] > limit or stamp-previous['stamp_ns'] > limit):
                raise SafetyInterlockError(f'{key} duplicate/backward timestamp or telemetry gap')
            self.latest[key] = row
            try:
                vector = self._sample(key, row, active=active, allow_gain_transition=allow_gain_transition)
            except StaticSettlingError as exc:
                if active:
                    raise
                self.reset(str(exc))
                continue
            window = self.windows[key]
            window.append((t, vector))
            # Keep the sample just before the one-second boundary, so every
            # stream must actually span a full second, not merely 20 polls.
            while len(window) > 1 and window[1][0] <= t-1_000_000_000:
                window.popleft()
            if vector and (not active or key != 'hal_arm'):
                # Monotone queues keep per-channel extrema in amortized O(1).
                # Scanning a 500 Hz window per callback starves DDS reception.
                extrema = self.extrema[key]
                if not extrema:
                    extrema.extend((deque(), deque()) for _ in vector)
                ranges = []
                for value, (low, high) in zip(vector, extrema):
                    while low and low[-1][1] >= value:
                        low.pop()
                    while high and high[-1][1] <= value:
                        high.pop()
                    low.append((t, value)); high.append((t, value))
                    for queue in (low, high):
                        while queue[0][0] < window[0][0]:
                            queue.popleft()
                    ranges.append(high[0][1]-low[0][1])
                if key.endswith('imu'):
                    # The bounding-box diagonal bounds every pair of gravity
                    # vectors, conservatively enforcing the angular spread.
                    chord = 2*9.81*math.sin(POLICY['imu_direction_spread_rad']/2)
                    stable = ranges[-1] <= POLICY['imu_acceleration_spread_m_s2'] and sum(
                        x*x for x in ranges[:3]) <= chord*chord
                else:
                    bound = (POLICY['hal_tolerance_rad'] if key == 'hal_arm' else
                             POLICY['hand_spread_rad'] if key == 'hands' else POLICY['joint_spread_rad'])
                    selected = ranges[1:] if active and key == 'arm_state' else ranges
                    stable = all(spread <= bound for spread in selected)
                if not stable:
                    if active:
                        raise StaticSettlingError(f'{key} lost continuous stability during trajectory')
                    self.reset(f'{key} window not settled')
        self.require_fresh(now_ns, now_ros)
        if active and self.action == 'UPPERBODY_REMOTE_SPLIT' and not allow_gain_transition and not self.urs_gains_confirmed:
            raise SafetyInterlockError('URS 40/2 not confirmed before compensation/trajectory')
        self.ready = all(len(w) >= (POLICY['min_mc_samples'] if k == 'mc' else POLICY['min_samples'])
                         and w[-1][0]-w[0][0] >= 1_000_000_000 for k, w in self.windows.items())
        if not active and now_ns-self.started_ns >= round(POLICY['timeout_s']*1e9):
            raise SafetyInterlockError(f'Preparation timeout: {self.last_reason}')
        return {'ready': self.ready, 'reset_count': self.reset_count, 'last_reason': self.last_reason,
                'window_counts': {k: len(w) for k, w in self.windows.items()},
                'checked_monotonic_ns': now_ns}

    def require_fresh(self, now_ns, now_ros):
        # Recheck after computation/journaling and immediately before sending.
        for key in TOPICS:
            limit = round((POLICY['max_mc_gap_s'] if key == 'mc' else POLICY['max_sample_gap_s'])*1e9)
            row = self.latest.get(key)
            if row is None or any(not 0 <= age <= limit for age in (
                    now_ns-row['received_monotonic_ns'], now_ros-row['stamp_ns'],
                    now_ros-row['received_ros_ns'])):
                raise SafetyInterlockError(f'{key} missing/stale preparation feedback')


def wait_for_preparation(check, *, sleep=time.sleep):
    """Poll pacing only; elapsed sleep alone can never release a command."""
    while not check()['ready']:
        sleep(.02)


def review_preparation(result, first_command_ns):
    """Replay raw receive batches; never trust a saved ready flag."""
    evidence = result['preparation']
    if evidence['policy'] != POLICY:
        raise ValueError('Unknown preparation policy')
    events = evidence['events']
    if [e['phase'] for e in events] != ['preparation_started', 'mode_returned', 'stable_confirmed']:
        raise ValueError('Incomplete preparation events')
    started, returned, stable = [e['monotonic_ns'] for e in events]
    if 'events' in result:
        before = next(e['monotonic_ns'] for e in result['events'] if e['phase'] == 'before_mode')
        if not before <= started:
            raise ValueError('Preparation started before the fixed pre-mode event')
    if not evidence['started_ns'] < started <= returned < stable <= first_command_ns:
        raise ValueError('Preparation/command event ordering invalid')
    batches = evidence['batches']
    acquired = {}
    while batches and batches[0]['stage'] == 'acquisition':
        acquisition = batches[0]
        if acquisition['error'] or any(j.get('error_code') != 0
                for item in acquisition['rows'] if item['stream'] in {'arm_state', 'waist_state', 'hands'}
                for j in item['sample']['joints']):
            raise ValueError('Fault/error during initial telemetry acquisition')
        acquired.update({item['stream']: item['sample'] for item in acquisition['rows']})
        batches = batches[1:]
    if not batches or batches[0]['stage'] != 'qualification':
        raise ValueError('Missing original preparation references')
    initial = {item['stream']: item['sample'] for item in batches[0]['rows']}
    if acquired and acquired != initial:
        raise ValueError('Initial fixed references differ from acquisition latest samples')
    if fixed_references(initial) != evidence['references'] or result['encoder_reference_rad'] != evidence['references']['arm_state']:
        raise ValueError('Preparation references were changed')
    monitor = PreparationMonitor(result['baseline_command_rad'], evidence['references'],
                                 started_ns=evidence['started_ns'], action='STAND_DEFAULT')
    stage, was_ready, previous_time = 'qualification', False, 0
    confirmed = False
    trajectory_checks = []
    for batch in batches:
        t = batch['now_ns']
        if t < previous_time:
            raise ValueError('Unordered preparation batches')
        previous_time = t
        if batch['stage'] == 'after_restore':
            continue  # Different physical window: not URS excursion/response.
        if batch['stage'] == 'preparation' and stage == 'qualification':
            if not was_ready:
                raise ValueError('Mode entered without pre-mode stable evidence')
            monitor.begin_preparation(started, 'STAND_DEFAULT' if result['dry_run'] else 'UPPERBODY_REMOTE_SPLIT')
            stage = 'preparation'
        elif batch['stage'] == 'trajectory' and stage == 'preparation':
            if not was_ready or t < stable:
                raise ValueError('Trajectory started without continuous stable evidence')
            confirmed = True
            stage = 'trajectory'
        if batch['stage'] == 'urs_observation' and stage == 'trajectory':
            stage = 'urs_observation'
        if batch['stage'] == 'trajectory':
            trajectory_checks.append(t)
        if batch['stage'] != stage:
            raise ValueError('Invalid preparation stage sequence')
        if (stage == 'qualification' and t > started) or (stage == 'preparation' and not returned <= t <= stable):
            raise ValueError('Preparation samples outside declared phase')
        allow_gain_transition = (stage == 'trajectory' and 'decisions' in result and
                                 result['decisions'][len(trajectory_checks)-1]['phase'] == 'baseline_hold')
        status = monitor.consume(batch, now_ns=t, now_ros=batch['now_ros'],
                                 active=stage in ('trajectory', 'urs_observation'),
                                 allow_gain_transition=allow_gain_transition)
        was_ready = status['ready']
    if not confirmed:
        raise ValueError('No verified transition from preparation to trajectory')
    if 'physical_publish_count' in result:
        decisions = result['decisions']
        if len(trajectory_checks) != result['physical_publish_count'] or len(decisions) != len(trajectory_checks):
            raise ValueError('Missing continuous trajectory checks')
        for checked, decision in zip(trajectory_checks, decisions):
            if not 0 <= decision['receipt']['monotonic_ns']-checked <= 50_000_000:
                raise ValueError('Preparation/trajectory check not adjacent to actual publication')
    return {'verified': True, 'stable_monotonic_ns': stable, 'policy': dict(POLICY),
            'reference_preserved': True, 'scope': 'sampled telemetry, not external pose or recovery qualification'}
