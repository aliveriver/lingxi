from copy import deepcopy
from types import SimpleNamespace as NS
from contextlib import contextmanager

import pytest

from lingxi_x2.errors import SafetyInterlockError
from lingxi_x2.gravity_diagnostic import diagnostic_mode, static_imu, StaticSettlingError
from lingxi_x2.gravity_preparation import (PreparationMonitor, PreparationCapture,
    HAND_NAMES, POLICY, TOPICS, fixed_references, wait_for_preparation, preparation_sample, review_preparation)
from lingxi_x2.gravity_observation import WAIST_NAMES
from lingxi_x2.models import ARM_JOINT_NAMES

START = 1_000_000_000
ROS_OFFSET = 100_000_000_000


def batch(t, *, action='UPPERBODY_REMOTE_SPLIT'):
    stamp = {'received_monotonic_ns': t, 'received_ros_ns': t+ROS_OFFSET, 'stamp_ns': t+ROS_OFFSET}
    samples = {}
    for key in TOPICS:
        row = dict(stamp)
        if key.endswith('imu'):
            row.update(frame_id='base_link', orientation_xyzw=[0.,0.,0.,1.],
                       orientation_covariance=[0.]*9, linear_acceleration_m_s2=[0.,0.,9.81])
        elif key == 'mc':
            row.update(action=action, action_status=100, fsm=4, body=1, player=0)
        else:
            names = {'arm_state': ARM_JOINT_NAMES, 'hal_arm': ARM_JOINT_NAMES,
                     'waist_state': WAIST_NAMES, 'hands': HAND_NAMES}[key]
            row['joints'] = [{'name': n, 'position': 0., 'velocity': 0., 'error_code': 0, 'stiffness':40., 'damping':2.} for n in names]
            if key == 'hands': row['hand_types'] = [1,1]
        samples[key] = row
    return {'rows': [{'stream': k, 'sample': v} for k,v in samples.items()], 'error': None,
            'now_ns': t, 'now_ros': t+ROS_OFFSET}


def row(b, key):
    return next(item['sample'] for item in b['rows'] if item['stream'] == key)


def monitor():
    references = fixed_references({item['stream']: item['sample'] for item in batch(START)['rows']})
    return PreparationMonitor([0.]*14, references, started_ns=START, action='UPPERBODY_REMOTE_SPLIT')


def consume(m, b, **kw):
    return m.consume(b, now_ns=b['now_ns'], now_ros=b['now_ros'], **kw)


def feed(m, start=0, count=51, **kw):
    status = None
    for i in range(start, start+count):
        status = consume(m, batch(START+i*20_000_000), **kw)
    return status


def test_requires_full_continuous_second_of_each_stream():
    m = monitor()
    assert not feed(m, count=50)['ready']
    assert feed(m, start=50, count=1)['ready']
    assert m.references['arm_state'] == [0.]*14


def test_original_imu_transient_resets_window_without_relaxing_thresholds():
    m = monitor()
    feed(m, count=30)
    b = batch(START+600_000_000)
    row(b,'chest_imu')['linear_acceleration_m_s2'][-1] = 12.543753
    row(b,'pelvis_imu')['linear_acceleration_m_s2'][-1] = 10.681534
    assert not consume(m,b)['ready']
    assert not feed(m,start=31,count=50)['ready']
    assert feed(m,start=81,count=1)['ready']
    assert m.references['arm_state'] == [0.]*14


@pytest.mark.parametrize('index,value', [(4, -23.549616699218753), (13, -3.19638671875), (4, 23.55)])
def test_stable_fault_free_implausible_hands_cannot_become_fixed_references(index, value):
    b = batch(START)
    row(b, 'hands')['joints'][index]['position'] = value
    samples = {item['stream']: item['sample'] for item in b['rows']}
    with pytest.raises(SafetyInterlockError, match='Implausible hand hold position'):
        fixed_references(samples)
    # A hand jump in a later callback must also be rejected, even with fault=0.
    m = monitor()
    with pytest.raises(SafetyInterlockError, match='Implausible hand hold position'):
        consume(m, b)
    references = deepcopy(m.references)
    references['hands'][index] = value
    with pytest.raises(SafetyInterlockError, match='Implausible hand hold position'):
        PreparationMonitor([0.]*14, references, started_ns=START, action='STAND_DEFAULT')


@pytest.mark.parametrize('key', ['arm_state','waist_state','hands'])
def test_slow_cumulative_drift_cannot_rebase_away(key):
    m = monitor()
    with pytest.raises(SafetyInterlockError, match='excursion'):
        for i in range(100):
            b = batch(START+i*20_000_000)
            row(b,key)['joints'][-1]['position'] = i*.0001
            consume(m,b)
    assert m.references[key][-1] == 0.


@pytest.mark.parametrize('key', ['arm_state','waist_state','hands'])
def test_intervening_fault_in_drained_batch_is_not_hidden_by_good_latest(key):
    m = monitor()
    first, last = batch(START), batch(START+20_000_000)
    row(first,key)['joints'][0]['error_code'] = 1
    last['rows'] = first['rows']+last['rows']
    with pytest.raises(SafetyInterlockError, match='fault'):
        consume(m,last)


@pytest.mark.parametrize('damage', ['stale','duplicate','source_backwards','gap','missing','overflow','future','nan','hand_type','fault','mc'])
def test_corrupt_or_unsafe_feedback_aborts(damage):
    m = monitor();consume(m,batch(START))
    b = batch(START+20_000_000)
    if damage == 'stale': b['now_ns'] += 110_000_000;b['now_ros'] += 110_000_000
    elif damage == 'duplicate': row(b,'hands')['stamp_ns'] -= 20_000_000
    elif damage == 'source_backwards': row(b,'waist_state')['stamp_ns'] -= 30_000_000
    elif damage == 'gap': b = batch(START+110_000_000)
    elif damage == 'missing':
        m = monitor();b['rows'] = [item for item in b['rows'] if item['stream'] != 'hands']
    elif damage == 'overflow': b['error'] = 'Preparation queue overflow'
    elif damage == 'future': row(b,'chest_imu')['stamp_ns'] += 1
    elif damage == 'nan': row(b,'hands')['joints'][0]['position'] = float('nan')
    elif damage == 'hand_type': row(b,'hands')['hand_types'][1] = 2
    elif damage == 'fault': row(b,'hands')['joints'][0]['error_code'] = None
    elif damage == 'mc': row(b,'mc')['body'] = 2
    with pytest.raises(SafetyInterlockError): consume(m,b)


def test_repolling_latest_cannot_satisfy_window_and_eventually_stales():
    m = monitor();consume(m,batch(START))
    b = batch(START+90_000_000);b['rows'] = []
    assert not consume(m,b)['ready']
    b['now_ns'] += 20_000_000;b['now_ros'] += 20_000_000
    with pytest.raises(SafetyInterlockError,match='stale'):consume(m,b)


def test_hal_target_must_return_to_original_baseline_before_readiness():
    m = monitor()
    for i in range(60):
        b=batch(START+i*20_000_000);row(b,'hal_arm')['joints'][3]['position'] = .002
        assert not consume(m,b)['ready']
    assert feed(m,start=60)['ready']
    assert m.baseline == (0.,)*14


def test_mode_running_alone_or_stationary_but_fast_reported_velocity_is_insufficient():
    for key in ('mc','waist_state'):
        m=monitor()
        for i in range(60):
            b=batch(START+i*20_000_000)
            if key=='mc':row(b,key)['player']=1
            else:row(b,key)['joints'][0]['velocity']=.021
            assert not consume(m,b)['ready']


def test_timeout_never_opens_stream_and_restoration_is_separate():
    m=monitor();calls=[]
    config=NS(control=NS(enabled=False))
    @contextmanager
    def stream(_):
        calls.append('stream');yield
    client=NS(config=config,status=lambda:NS(backend='ros2'),
              set_motion_mode=lambda mode,**kw:calls.append(mode),_backend=NS(command_stream=stream))
    def check():
        i=len(calls)+check.count;check.count+=1
        b=batch(START+i*20_000_000);row(b,'chest_imu')['linear_acceleration_m_s2'][-1]=12.54
        return consume(m,b)
    check.count=0
    with pytest.raises(SafetyInterlockError,match='timeout'):
        with diagnostic_mode(client,{},dry_run=False,
                             prepare=lambda:wait_for_preparation(check,sleep=lambda _:None)):
            calls.append('publish')
    assert calls == ['UPPERBODY_REMOTE_SPLIT','STAND_DEFAULT']
    assert not config.control.enabled


def test_preparation_completes_before_stream_opens():
    calls=[]
    @contextmanager
    def stream(_):calls.append('stream');yield
    c=NS(config=NS(control=NS(enabled=False)),status=lambda:NS(backend='ros2'),
         set_motion_mode=lambda mode,**kw:calls.append(mode),_backend=NS(command_stream=stream))
    with diagnostic_mode(c,{},dry_run=False,prepare=lambda:calls.append('stable')):calls.append('publish')
    assert calls == ['UPPERBODY_REMOTE_SPLIT','stable','stream','publish','STAND_DEFAULT']


def test_active_transient_aborts_instead_of_waiting_and_allows_only_selected_motion():
    m=monitor();feed(m)
    b=batch(START+1_020_000_000)
    row(b,'arm_state')['joints'][0].update(position=.01,velocity=.03)
    row(b,'hal_arm')['joints'][0]['position']=.012
    consume(m,b,active=True)
    b=batch(START+1_040_000_000);row(b,'chest_imu')['linear_acceleration_m_s2'][-1]=10.51
    with pytest.raises(StaticSettlingError):consume(m,b,active=True)


def test_capture_latches_overflow_and_bad_parser_even_after_drain():
    c=PreparationCapture(capacity=0)
    c.receive('invalid',NS(),mono=1,ros=2)
    assert c.drain()['error']
    assert c.drain()['error']
    message=NS(header=NS(stamp=NS(sec=1,nanosec=0),frame_id='base_link'),joints=[])
    c.receive('arm_state',message,mono=1,ros=2)
    assert 'overflow' in c.drain()['error']


def evidence():
    m=monitor()
    batches=[{**batch(START+i*20_000_000,action='STAND_DEFAULT'),'stage':'qualification'} for i in range(51)]
    started=START+1_010_000_000
    batches += [{**batch(START+i*20_000_000),'stage':'preparation'} for i in range(51,102)]
    stable=START+2_025_000_000
    batches += [{**batch(START+2_040_000_000),'stage':'trajectory'}]
    return {'baseline_command_rad':[0.]*14,'encoder_reference_rad':[0.]*14,'dry_run':False,
            'preparation':{'policy':dict(POLICY),'references':m.references,'started_ns':START,
                'events':[{'phase':p,'monotonic_ns':t} for p,t in (
                    ('preparation_started',started),('mode_returned',started+1),('stable_confirmed',stable))],
                'batches':batches}}


def test_offline_preparation_recomputes_readiness_and_ignores_return_excursion():
    r=evidence()
    b=batch(START+2_100_000_000,action='STAND_DEFAULT');row(b,'arm_state')['joints'][3]['position']=.01975
    r['preparation']['batches'].append({**b,'stage':'after_restore'})
    assert review_preparation(r,START+2_030_000_000)['verified']


@pytest.mark.parametrize('damage',['rebase','fake_ready','missing_hand','early_command','policy','phase','duplicate','fault'])
def test_offline_rejects_manipulated_preparation_evidence(damage):
    r=evidence();p=r['preparation'];first=START+2_030_000_000
    if damage=='rebase':p['references']['arm_state'][0]=.001
    elif damage=='fake_ready':
        p['batches']=p['batches'][:52]+p['batches'][-1:]
        for b in p['batches']:b['status']={'ready':True}
    elif damage=='missing_hand':
        for b in p['batches']:b['rows']=[i for i in b['rows'] if i['stream']!='hands']
    elif damage=='early_command':first=START+1_500_000_000
    elif damage=='policy':p['policy']['stable_duration_s']=.01
    elif damage=='phase':p['events'].pop()
    elif damage=='duplicate':p['batches'].insert(10,deepcopy(p['batches'][9]))
    elif damage=='fault':row(p['batches'][70],'hands')['joints'][0]['error_code']=2
    with pytest.raises((ValueError,KeyError,SafetyInterlockError)):review_preparation(r,first)


def test_hand_raw_velocity_is_not_misinterpreted_as_radians_per_second():
    m=monitor()
    for i in range(51):
        b=batch(START+i*20_000_000)
        row(b,'hands')['joints'][0]['velocity']=4096.
        status=consume(m,b)
    assert status['ready']
    b=batch(START+1_020_000_000);row(b,'hands')['joints'][0]['position']=.003
    consume(m,b,active=True)
    b=batch(START+1_040_000_000);row(b,'hands')['joints'][0]['position']=-.003
    with pytest.raises(SafetyInterlockError,match='stability'):consume(m,b,active=True)


def test_stand_gains_cannot_qualify_urs_and_changed_gain_aborts_active():
    m=monitor()
    for i in range(51):
        b=batch(START+i*20_000_000);row(b,'hal_arm')['joints'][0]['stiffness']=100.
        assert not consume(m,b)['ready']
    assert feed(m,start=51)['ready']
    b=batch(START+2_040_000_000);row(b,'hal_arm')['joints'][0]['damping']=1.
    with pytest.raises(SafetyInterlockError,match='40/2'):consume(m,b,active=True)


def test_hand_parser_preserves_all_twenty_axes_and_nonzero_fault():
    joints=[NS(position=0.,velocity=4096.,faultcode=0) for _ in range(10)]
    message=NS(header=NS(stamp=NS(sec=10,nanosec=1)),left_hands=joints,right_hands=deepcopy(joints),
               left_hand_type=NS(value=1),right_hand_type=NS(value=1))
    message.right_hands[0].faultcode=2
    r=preparation_sample('hands',message,1,2)
    assert tuple(j['name'] for j in r['joints']) == HAND_NAMES
    assert r['joints'][10]['error_code']==2
    assert r['joints'][0]['velocity']==4096.


def test_nonfinite_hand_sample_latches_capture_error_instead_of_poisoning_json():
    joints=[NS(position=0.,velocity=0.,faultcode=0) for _ in range(10)]
    message=NS(header=NS(stamp=NS(sec=10,nanosec=1)),left_hands=joints,right_hands=deepcopy(joints),
               left_hand_type=NS(value=1),right_hand_type=NS(value=1))
    message.right_hands[0].position=float('nan')
    c=PreparationCapture();c.receive('hands',message,mono=1,ros=2)
    b=c.drain()
    assert b['error'] and not b['rows']


def test_invalid_orientation_is_not_hidden_by_transient_acceleration():
    s=row(batch(START),'chest_imu')
    s['orientation_xyzw']=[0.]*4;s['linear_acceleration_m_s2'][-1]=12.54
    with pytest.raises(SafetyInterlockError,match='quaternion') as exc:static_imu(s)
    assert not isinstance(exc.value,StaticSettlingError)


def test_feedback_that_was_fresh_at_check_cannot_be_sent_after_slow_journal():
    m=monitor();consume(m,batch(START))
    with pytest.raises(SafetyInterlockError,match='stale'):
        m.require_fresh(START+101_000_000,START+ROS_OFFSET+101_000_000)


def test_restore_failure_keeps_original_preparation_failure_and_request_times():
    result={};calls=[]
    def mode(name,**kw):
        calls.append(name)
        if name=='STAND_DEFAULT':raise RuntimeError('restore failed')
    def prepare():raise SafetyInterlockError('hand fault')
    client=NS(config=NS(control=NS(enabled=False)),status=lambda:NS(backend='ros2'),set_motion_mode=mode)
    with pytest.raises(RuntimeError,match='restore failed'):
        with diagnostic_mode(client,result,dry_run=False,prepare=prepare):pytest.fail('entered stream')
    assert 'hand fault' in result['primary_error']
    assert 'restore failed' in result['mode_restore_error']
    assert result['mode_restore_requested_monotonic_ns']>=result['mode_entry_returned_monotonic_ns']
    assert not client.config.control.enabled


def test_discovery_history_cannot_count_as_a_stable_window_or_hide_a_fault():
    r=evidence();p=r['preparation']
    acquisition=deepcopy(p['batches'][0]);acquisition['stage']='acquisition'
    old=deepcopy(acquisition)
    for item in old['rows']:item['sample']['stamp_ns']-=300_000_000
    p['batches']=[old,acquisition]+p['batches']
    assert review_preparation(r,START+2_030_000_000)['verified']
    row(old,'hands')['joints'][0]['error_code']=1
    with pytest.raises(ValueError,match='Fault/error'):review_preparation(r,START+2_030_000_000)


def test_mode_running_can_keep_stand_gains_until_first_fixed_baseline_command():
    initial=batch(START,action='STAND_DEFAULT')
    for j in row(initial,'hal_arm')['joints']:j['stiffness']=100.;j['damping']=1.
    refs=fixed_references({i['stream']:i['sample'] for i in initial['rows']})
    m=PreparationMonitor([0.]*14,refs,started_ns=START,action='UPPERBODY_REMOTE_SPLIT')
    for i in range(51):
        b=batch(START+i*20_000_000)
        for j in row(b,'hal_arm')['joints']:j['stiffness']=100.;j['damping']=1.
        status=consume(m,b)
    assert status['ready'] and not m.urs_gains_confirmed
    b=batch(START+1_020_000_000)
    for j in row(b,'hal_arm')['joints']:j['stiffness']=100.;j['damping']=1.
    consume(m,b,active=True,allow_gain_transition=True)
    with pytest.raises(SafetyInterlockError,match='40/2'):
        consume(m,batch(START+1_040_000_000)|{'rows':[]},active=True)
    consume(m,batch(START+1_060_000_000),active=True,allow_gain_transition=True)
    assert m.urs_gains_confirmed
    b=batch(START+1_080_000_000)
    for j in row(b,'hal_arm')['joints']:j['stiffness']=100.;j['damping']=1.
    with pytest.raises(SafetyInterlockError,match='regressed'):
        consume(m,b,active=True,allow_gain_transition=True)
