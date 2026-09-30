from copy import deepcopy
from contextlib import contextmanager
from types import SimpleNamespace as NS
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from lingxi_x2.errors import SafetyInterlockError
from lingxi_x2.gravity_diagnostic import (LimitedGravityGuard, limited_gravity_plan,
                                        check_encoder_excursion, diagnostic_mode)
from lingxi_x2.models import ARM_JOINT_NAMES, JointSample
from test_gravity import urdf
from test_gravity_observation import waist_urdf, snapshot


def calibration():
    return {key:{'R_baselink_sensor':np.eye(3).reshape(-1).tolist()} for key in ('chest_imu','waist_imu')}


def live_snapshot():
    s=snapshot()
    for key in ('chest_imu','pelvis_imu'):
        s['samples'][key]['frame_id']='base_link'
        s['samples'][key]['linear_acceleration_m_s2']=[0.,0.,9.81]
    return s


@pytest.fixture
def model(urdf,waist_urdf):
    root=ET.parse(urdf).getroot()
    for origin in root.findall('link/inertial/origin'):
        origin.set('xyz','-1 0 0')  # +19.62 Nm analytic torque about +Y at q=0.
    for element in ET.parse(waist_urdf).getroot():
        if element.tag=='link' and element.attrib['name']=='torso_link':continue
        root.append(element)
    ET.ElementTree(root).write(urdf)
    return urdf


def evaluate(guard,s):
    return guard.evaluate(s,[0.]*14,now_mono=s['captured_monotonic_ns'],now_ros=s['captured_ros_ns'])


def test_capped_hypotheses_require_same_positive_compensation(model):
    r=evaluate(LimitedGravityGuard(model,calibration()),live_snapshot())
    assert len(r['hypotheses'])==6
    assert not r['model_and_mounting_verified']
    for h in r['hypotheses'].values():
        assert h['torque_nm']==pytest.approx(19.62)
        assert h['unbounded_bias_rad']==pytest.approx(19.62/40)
        assert h['clipped_bias_rad']==.002


@pytest.mark.parametrize('damage',['stale','future','skew','quaternion','accel_norm','accel_direction',
                                    'missing_waist','waist_fault','frame','capture_error','mount','tilt','changed_model'])
def test_live_guard_rejects_invalid_sources(model,damage):
    s=live_snapshot();c=calibration();guard=LimitedGravityGuard(model,c)
    if damage=='stale':s['samples']['chest_imu']['received_monotonic_ns']-=110_000_000
    elif damage=='future':s['samples']['pelvis_imu']['stamp_ns']+=100_000_000
    elif damage=='skew':s['samples']['waist_state']['stamp_ns']-=70_000_000
    elif damage=='quaternion':s['samples']['chest_imu']['orientation_xyzw']=[0.,0.,0.,0.]
    elif damage=='accel_norm':s['samples']['chest_imu']['linear_acceleration_m_s2']=[0.,0.,5.]
    elif damage=='accel_direction':s['samples']['chest_imu']['linear_acceleration_m_s2']=[9.81,0.,0.]
    elif damage=='missing_waist':s['samples'].pop('waist_state')
    elif damage=='waist_fault':s['samples']['waist_state']['joints'][1]['error_code']=1
    elif damage=='frame':s['samples']['pelvis_imu']['frame_id']='unknown'
    elif damage=='capture_error':s['error_count']=1
    elif damage=='mount':
        c['chest_imu']['R_baselink_sensor']=[-1,0,0,0,1,0,0,0,1]
        with pytest.raises(ValueError):LimitedGravityGuard(model,c)
        return
    elif damage=='tilt':
        angle=math.radians(12)
        for key in ('chest_imu','pelvis_imu'):
            s['samples'][key]['orientation_xyzw']=[0.,math.sin(angle/2),0.,math.cos(angle/2)]
            s['samples'][key]['linear_acceleration_m_s2']=[-9.81*math.sin(angle),0.,9.81*math.cos(angle)]
    elif damage=='changed_model':model.write_bytes(model.read_bytes()+b'\n')
    with pytest.raises((SafetyInterlockError,ValueError,KeyError)):
        evaluate(guard,s)


def test_negative_or_insufficient_gravity_is_not_replaced_with_fixed_bias(model):
    root=ET.parse(model).getroot()
    for element in root.findall('link/inertial/mass'):element.set('value','0')
    ET.ElementTree(root).write(model)
    with pytest.raises(SafetyInterlockError,match='hypotheses'):
        evaluate(LimitedGravityGuard(model,calibration()),live_snapshot())
    root=ET.parse(model).getroot()
    for e in root.findall('link/inertial/mass'):e.set('value','1')
    for e in root.findall('link/inertial/origin'):e.set('xyz','1 0 0')
    ET.ElementTree(root).write(model)
    with pytest.raises(SafetyInterlockError,match='hypotheses'):
        evaluate(LimitedGravityGuard(model,calibration()),live_snapshot())


def test_pose_change_after_qualification_aborts(model):
    guard=LimitedGravityGuard(model,calibration());s=live_snapshot();evaluate(guard,s)
    angle=math.radians(2)
    for key in ('chest_imu','pelvis_imu'):
        s['samples'][key]['orientation_xyzw']=[0.,math.sin(angle/2),0.,math.cos(angle/2)]
        s['samples'][key]['linear_acceleration_m_s2']=[-9.81*math.sin(angle),0.,9.81*math.cos(angle)]
    with pytest.raises(SafetyInterlockError,match='since qualification'):evaluate(guard,s)


def test_checks_freshness_again_at_publication(model):
    s=live_snapshot();guard=LimitedGravityGuard(model,calibration());evaluate(guard,s)
    with pytest.raises(SafetyInterlockError,match='100 ms'):
        guard.require_fresh(s,s['captured_monotonic_ns']+101_000_000,s['captured_ros_ns']+101_000_000)


def test_limited_plan_ramps_bias_and_returns_to_same_command():
    q=(.4,0.,0.,-1.2,0.,0.,0.)*2
    plan=list(limited_gravity_plan(q))
    assert len(plan)==650
    assert plan[0]['command_rad']==plan[-1]['command_rad']==list(q)
    assert all(f['command_rad'][1:]==list(q[1:]) for f in plan)
    for frame in plan:
        assert frame['command_rad'][0]==pytest.approx(frame['desired_rad'][0]+frame['applied_bias_rad'])
        assert 0<=frame['applied_bias_rad']<=.002
        assert abs(frame['command_rad'][0]-q[0])<=.012+1e-12
    assert max(abs(b['command_rad'][0]-a['command_rad'][0])*50 for a,b in zip(plan,plan[1:]))<.012
    assert max(abs(b['applied_bias_rad']-a['applied_bias_rad'])*50 for a,b in zip(plan,plan[1:]))<.002
    means={phase:next(f for f in plan if f['phase']==phase)['command_rad'][0]
           for phase in ('compensated_baseline_hold','target_hold','compensated_recovery_hold','recovery_hold')}
    assert means==pytest.approx({'compensated_baseline_hold':.402,'target_hold':.412,
                                'compensated_recovery_hold':.402,'recovery_hold':.4})


@pytest.mark.parametrize('damage',['fault','other_axis','selected_axis','names'])
def test_encoder_envelope(damage):
    rows=[JointSample(n,0.,0.,0.,0,0) for n in ARM_JOINT_NAMES]
    from dataclasses import replace
    if damage=='fault':rows[0]=replace(rows[0],fault_code=1)
    elif damage=='other_axis':rows[7]=replace(rows[7],position_rad=.006)
    elif damage=='selected_axis':rows[0]=replace(rows[0],position_rad=.026)
    else:rows.reverse()
    with pytest.raises(SafetyInterlockError):check_encoder_excursion(rows,[0.]*14)


@pytest.mark.parametrize('fail_at',['entry','stream','body',None])
def test_mode_restoration_on_all_failures(fail_at):
    calls=[];config=NS(control=NS(enabled=False))
    def mode(name,**kwargs):
        assert config.control.enabled
        calls.append(name)
        if fail_at=='entry' and name=='UPPERBODY_REMOTE_SPLIT':raise RuntimeError('entry')
        return {'mode':name}
    @contextmanager
    def stream(_):
        if fail_at=='stream':raise RuntimeError('stream')
        yield
    client=NS(config=config,status=lambda:NS(backend='ros2'),set_motion_mode=mode,
              _backend=NS(command_stream=stream))
    result={}
    def run():
        with diagnostic_mode(client,result,dry_run=False):
            if fail_at=='body':raise RuntimeError('body')
    if fail_at:
        with pytest.raises(RuntimeError):run()
    else:run()
    assert calls==['UPPERBODY_REMOTE_SPLIT','STAND_DEFAULT']
    assert config.control.enabled is False
    assert result['mode_restore']['mode']=='STAND_DEFAULT'


def test_shadow_never_enters_mode_or_opens_stream():
    def forbidden(*a,**kw):pytest.fail('Shadow performed write operation')
    client=NS(config=NS(control=NS(enabled=False)),status=lambda:NS(backend='ros2'),
              set_motion_mode=forbidden,_backend=NS(command_stream=forbidden))
    with diagnostic_mode(client,{},dry_run=True):pass
    assert not client.config.control.enabled


def test_gravity_basis_optimization_matches_direct_model(model):
    c=calibration();angle=.01
    c['chest_imu']['R_baselink_sensor']=[math.cos(angle),0,math.sin(angle),0,1,0,-math.sin(angle),0,math.cos(angle)]
    guard=LimitedGravityGuard(model,c);s=live_snapshot()
    r=evaluate(guard,s)
    for h in r['hypotheses'].values():
        direct=guard.model.evaluate('left',[0.]*7,h['gravity_torso_m_s2'],payload_mass_kg=0,payload_com_wrist_m=[0,0,0])
        assert h['torque_nm']==pytest.approx(direct['gravity_torque_nm'][0],abs=1e-12)


def test_restore_failure_still_disables_process_control():
    config=NS(control=NS(enabled=False))
    def mode(name,**kw):
        if name=='STAND_DEFAULT':raise RuntimeError('restore unavailable')
        return {}
    @contextmanager
    def stream(_):yield
    client=NS(config=config,status=lambda:NS(backend='ros2'),set_motion_mode=mode,_backend=NS(command_stream=stream))
    with pytest.raises(RuntimeError,match='restore unavailable'):
        with diagnostic_mode(client,{},dry_run=False):pass
    assert config.control.enabled is False


def test_precomputed_geometry_does_not_cache_gravity_or_skip_freshness(model,monkeypatch):
    guard=LimitedGravityGuard(model,calibration());q=[0.]*14
    guard.prepare_desired_poses([q,q])
    monkeypatch.setattr(guard.model,'evaluate',lambda *a,**kw:pytest.fail('Repeated FK for prepared pose'))
    s=live_snapshot();first=evaluate(guard,s)
    angle=.005
    for key in ('chest_imu','pelvis_imu'):
        s['samples'][key]['orientation_xyzw']=[0.,math.sin(angle/2),0.,math.cos(angle/2)]
        s['samples'][key]['linear_acceleration_m_s2']=[-9.81*math.sin(angle),0.,9.81*math.cos(angle)]
    second=evaluate(guard,s)
    assert first['hypotheses']['chest_imu_identity']['torque_nm']!=second['hypotheses']['chest_imu_identity']['torque_nm']
    s['samples']['chest_imu']['stamp_ns']-=200_000_000
    with pytest.raises(SafetyInterlockError):evaluate(guard,s)


def test_fast_live_frames_match_offline_quaternion_waist_and_mount_math(model):
    from lingxi_x2.gravity_observation import analyze_gravity_observation,_quaternion
    from lingxi_x2.gravity import _rotation
    c=calibration();c['chest_imu']['R_baselink_sensor']=_rotation(np.array([0.,1.,0.]),.004).reshape(-1).tolist()
    guard=LimitedGravityGuard(model,c);s=live_snapshot();rng=np.random.default_rng(618)
    for _ in range(20):
        axis=rng.normal(size=3);axis/=np.linalg.norm(axis);angle=rng.uniform(-.004,.004)
        quat=[*(axis*math.sin(angle/2)),math.cos(angle/2)]
        acc=-_quaternion(quat).T@np.array([0.,0.,-9.81])
        for key in ('chest_imu','pelvis_imu'):
            s['samples'][key]['orientation_xyzw']=list(quat)
            s['samples'][key]['linear_acceleration_m_s2']=acc.tolist()
        for row in s['samples']['waist_state']['joints']:row['position']=float(rng.uniform(-.002,.002))
        result=evaluate(guard,s)
        for label,mount in guard.mounts.items():
            offline=analyze_gravity_observation(s,mount,urdf=model)
            assert offline['usable_for_offline_estimate']
            np.testing.assert_allclose(result['hypotheses'][label]['gravity_torso_m_s2'],offline['gravity_torso_m_s2'],atol=1e-12,rtol=0)


@pytest.mark.parametrize('damage',['no_orientation','clock_jump','negative_stamp','missing_arm','duplicate_joint'])
def test_fast_validator_retains_strict_input_checks(model,damage):
    s=live_snapshot()
    if damage=='no_orientation':s['samples']['chest_imu']['orientation_covariance'][0]=-1.
    elif damage=='clock_jump':s['captured_ros_ns']+=60_000_000
    elif damage=='negative_stamp':s['samples']['chest_imu']['stamp_ns']=-1
    elif damage=='missing_arm':s['samples']['arm_state']['joints'].pop()
    else:s['samples']['waist_state']['joints'].append(s['samples']['waist_state']['joints'][0])
    with pytest.raises(SafetyInterlockError):evaluate(LimitedGravityGuard(model,calibration()),s)


def test_bias_only_never_adds_desired_motion_and_preserves_return():
    q=(.4,0.,0.,-1.2,0.,0.,0.)*2
    plan=list(limited_gravity_plan(q,bias_only=True))
    assert len(plan)==350
    assert all(f['desired_rad']==list(q) for f in plan)
    assert max(f['command_rad'][0]-q[0] for f in plan)==pytest.approx(.002)
    assert plan[-1]['command_rad']==plan[0]['command_rad']==list(q)
    assert {f['phase'] for f in plan}=={'baseline_hold','bias_in','compensated_baseline_hold','bias_out','recovery_hold'}


def test_bounded_gc_window_restores_original_state_after_failure():
    import gc
    from lingxi_x2.gravity_diagnostic import bounded_timing_window
    original=gc.isenabled()
    try:
        for enabled in (True,False):
            gc.enable() if enabled else gc.disable()
            with pytest.raises(RuntimeError):
                with bounded_timing_window():
                    assert not gc.isenabled()
                    raise RuntimeError('stop')
            assert gc.isenabled()==enabled
    finally:
        gc.enable() if original else gc.disable()


@pytest.mark.parametrize('bias_only', [False, True])
def test_matched_control_preserves_every_phase_and_desired_pose(bias_only):
    q = (.4, 0., 0., -1.2, 0., 0., 0.)*2
    on = list(limited_gravity_plan(q, bias_only=bias_only))
    off = list(limited_gravity_plan(q, bias_only=bias_only, compensation_enabled=False))
    assert len(on) == len(off) == (350 if bias_only else 650)
    assert [f['phase'] for f in on] == [f['phase'] for f in off]
    for a, b in zip(on, off):
        assert a['desired_rad'] == b['desired_rad'] == b['command_rad']
        assert b['applied_bias_rad'] == 0.
        assert a['command_rad'][1:] == b['command_rad'][1:] == list(q[1:])
        assert a['command_rad'][0]-b['command_rad'][0] == pytest.approx(a['applied_bias_rad'])
    assert on[0]['command_rad'] == on[-1]['command_rad'] == off[-1]['command_rad'] == list(q)


def test_model_plan_rejects_sent_bias_crossing_urdf_limit_before_stream(model):
    from lingxi_x2.gravity_diagnostic import validate_model_plan
    # The desired pose and zero-bias motion fit; ONLY the added bias crosses.
    root = ET.parse(model).getroot()
    root.find("joint[@name='left_shoulder_pitch_joint']/limit").set('upper', '.011')
    ET.ElementTree(root).write(model)
    guard = LimitedGravityGuard(model, calibration())
    off = list(limited_gravity_plan([0.]*14, compensation_enabled=False))
    on = list(limited_gravity_plan([0.]*14))
    validate_model_plan(guard.model, off)
    with pytest.raises(SafetyInterlockError, match='command_rad outside installed model limits'):
        validate_model_plan(guard.model, on)


@pytest.mark.parametrize('field,value', [('bias_only', 1), ('compensation_enabled', 'off'),
                                        ('compensation_enabled', None)])
def test_diagnostic_flags_are_not_truthy_coerced(field, value):
    with pytest.raises(ValueError):
        list(limited_gravity_plan([0.]*14, **{field: value}))


def test_live_cli_requires_explicit_on_off_before_ros_initialization(monkeypatch, tmp_path):
    import importlib.util
    from pathlib import Path
    path = Path(__file__).resolve().parents[1]/'scripts/gravity_baseline_session.py'
    spec = importlib.util.spec_from_file_location('gravity_session_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'reexec_with_ros_environment', lambda: pytest.fail('Touched ROS'))
    monkeypatch.setattr(module, 'X2Client', lambda *_: pytest.fail('Created client'))
    with pytest.raises(SystemExit) as exc:
        module.main(['--confirm-hardware', '--trace', str(tmp_path/'unused.json')])
    assert exc.value.code == 2
    args = module.build_parser().parse_args(['--dry-run', '--compensation', 'off', '--trace', 'new.json'])
    assert args.compensation == 'off' and args.dry_run


def test_bounded_timing_restores_python_thread_interval_even_on_failure():
    import sys
    from lingxi_x2.gravity_diagnostic import bounded_timing_window
    original=sys.getswitchinterval()
    try:
        for before in (.005,.0005):
            sys.setswitchinterval(before)
            with pytest.raises(RuntimeError):
                with bounded_timing_window():
                    assert sys.getswitchinterval()==pytest.approx(min(before,.001))
                    raise RuntimeError('stop')
            assert sys.getswitchinterval()==pytest.approx(before)
    finally:sys.setswitchinterval(original)
