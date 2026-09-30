from copy import deepcopy
from dataclasses import replace
import json

import pytest

from lingxi_x2.acceptance_report import AcceptanceCriteria, evaluate_trace, evaluate_file, evaluate_repeatability
from lingxi_x2.models import ARM_JOINT_NAMES


def synthetic_trace(*, gain=1., residual=0., offset=0., clock_offset=0):
    """Independent timestamped source streams; not derived from report code."""
    baseline = [.4, 0., 0., -1.2, 0., 0., 0.] * 2
    phases = [('before_mode',.5),('baseline_hold',1),('target_ramp',2),('target_hold',3),
              ('recovery_ramp',4),('recovery_hold',5),('recovery_complete',6),('publishing_stopped',6.5),('final',6.6)]
    stand = {'timestamp':{'monotonic_ns':6_600_000_000+clock_offset},'action':'STAND_DEFAULT','action_status':100,'fsm_state':4,'body_state':1}
    events = [{'phase':name,'monotonic_ns':round(t*1e9)+clock_offset,'mc':stand} for name,t in phases]
    trace = {'kind':'control_path_trace','errors':[], 'dropped':dict.fromkeys(('upper','hal_arm','arm_state','mc'),0),
             'result':{'kind':'fixed_command_baseline_session','joint_index':0,'delta_rad':.02,
                       'baseline_command_rad':baseline,'events':events,'mode_restore':{'mc_after':stand},
                       'final_hands':{side:{'joints':[{'fault_code':0} for _ in range(10)]} for side in ('left','right')}},
             'samples':{k:[] for k in ('upper','hal_arm','arm_state','mc')}}
    events[-1]['arm']={'timestamp':{'monotonic_ns':6_600_000_000+clock_offset},
                       'joints':[{'name':n,'fault_code':0} for n in ARM_JOINT_NAMES]}
    for side in ('left','right'):
        trace['result']['final_hands'][side]['timestamp']={'monotonic_ns':6_600_000_000+clock_offset}
    for step in range(500):
        t = 1+step/100
        stamp={'received_monotonic_ns':1_000_000_000+step*10_000_000+clock_offset,
               'stamp_ns':100_000_000_000+step*10_000_000+clock_offset}
        d = 0. if t<2 or t>=5 else (.02*(t-2) if t<3 else (.02 if t<4 else .02*(5-t)))
        target=baseline.copy();target[0]+=d
        if step%2 == 0:
            trace['samples']['upper'].append({**stamp,'sequence':step//2,'arm_pos':target,
                                             'hand_pos':[0.]*20,'hand_sub_mode':2})
        for kind in ('hal_arm','arm_state'):
            q=target.copy()
            if kind=='arm_state':q[0]=baseline[0]+gain*d+offset+(residual if t>=5 else 0)
            trace['samples'][kind].append({**stamp,'joints':[{'name':name,'position':pos,'velocity':0.,'effort':0.,'error_code':0}
                                                           for name,pos in zip(ARM_JOINT_NAMES,q)]})
        if step%10 == 0:
            trace['samples']['mc'].append({**stamp,'action':'UPPERBODY_REMOTE_SPLIT','action_status':100,'fsm':4,'body':1})
    return trace


def failed(report, key):
    return next(c for c in report['quality_checks']+report['motion_checks'] if c['name']==key)['passed'] is False


def test_passing_differential_and_absolute_data_never_unlocks_robot():
    report=evaluate_trace(synthetic_trace())
    assert report['verdict']=='pass_under_criteria'
    assert report['metrics']['tracking_fraction']==pytest.approx(1.)
    assert not report['hardware_acceptance_complete'] and not report['execution_authorized']


@pytest.mark.parametrize('gain,residual,offset,check',[(.163,.0021,0.,'increment_following_error'),
                                                     (1.,.0021,0.,'return_residual'),(1.,0.,.02,'selected_absolute_position_error')])
def test_published_targets_and_mode_restore_cannot_pass_bad_motion(gain,residual,offset,check):
    trace=synthetic_trace(gain=gain,residual=residual,offset=offset)
    trace['result']['publication_stats']={'sequence_continuous':True,'published_count':250,'success':True}
    report=evaluate_trace(trace)
    assert report['verdict']=='fail' and failed(report,check)


@pytest.mark.parametrize('gain,execution_code,expected', [(1.,0,0),(.134,0,3),(1.,2,2)])
def test_session_exit_code_uses_encoder_acceptance(gain, execution_code, expected):
    import runpy
    from pathlib import Path
    session = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/command_baseline_session.py'))
    trace = synthetic_trace(gain=gain)
    assert session['review_completed_session'](trace, execution_code) == expected
    assert trace['result']['acceptance']['verdict'] == ('fail' if gain < 1 else 'pass_under_criteria')
    assert not trace['result']['acceptance']['execution_authorized']


def test_session_inconclusive_evidence_cannot_exit_successfully():
    import runpy
    from pathlib import Path
    session = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/command_baseline_session.py'))
    trace = synthetic_trace()
    trace['samples']['arm_state'] = []
    assert session['review_completed_session'](trace, 0) == 3
    assert trace['result']['acceptance']['verdict'] == 'inconclusive'


@pytest.mark.parametrize('damage,check', [
    ('drops','trace_no_buffer_drops'),('errors','trace_parse_errors'),('mc','urs_standing_throughout'),
    ('fault','arm_faults_zero'),('source_clock','arm_state_stamp_ns_increasing'),
    ('sequence','upper_sequence'),('gap','arm_state_receive_coverage'),
    ('other_command','hal_arm_fixed_baseline_single_axis'),('hand','hand_commands_held'),
    ('restore','restored_standing'),('target','target_hold_hal_target_delivered'),
    ('order','trace_structure'),('phase','trace_structure'),('nan','trace_structure')])
def test_invalid_evidence_is_inconclusive(damage,check):
    trace=synthetic_trace()
    if damage=='drops':trace['dropped']['arm_state']=1
    if damage=='errors':trace['errors']=['parse failure']
    if damage=='mc':trace['samples']['mc'][20]['fsm']=6
    if damage=='fault':trace['samples']['arm_state'][100]['joints'][0]['error_code']=3
    if damage=='source_clock':trace['samples']['arm_state'][100]['stamp_ns']=1
    if damage=='sequence':trace['samples']['upper'][20]['sequence']=99
    if damage=='gap':del trace['samples']['arm_state'][100:120]
    if damage=='other_command':trace['samples']['hal_arm'][100]['joints'][7]['position']+=.01
    if damage=='hand':trace['samples']['upper'][100]['hand_pos'][2]=.1
    if damage=='restore':trace['result']['mode_restore']={}
    if damage=='target':
        for row in trace['samples']['hal_arm'][200:300]:row['joints'][0]['position']-=.01
    if damage=='order':trace['samples']['arm_state'][100]['joints'].reverse()
    if damage=='phase':trace['result']['events'].pop(6)
    if damage=='nan':trace['samples']['arm_state'][100]['joints'][0]['position']=float('nan')
    report=evaluate_trace(trace)
    assert report['verdict']=='inconclusive' and failed(report,check)


def test_recovery_window_excludes_standing_after_urs():
    trace=synthetic_trace(residual=.003)
    # A later standing sample perfectly matches baseline; it cannot replace URS recovery.
    row=deepcopy(trace['samples']['arm_state'][0]);row['received_monotonic_ns']=6_550_000_000;row['stamp_ns']+=6_000_000_000
    trace['samples']['arm_state'].append(row)
    report=evaluate_trace(trace)
    assert report['metrics']['encoder_recovery_delta_rad']==pytest.approx(.003)
    assert report['windows']['recovery_hold']['stop_monotonic_ns']==6_000_000_000
    assert report['verdict']=='fail'


def test_nonselected_encoder_drift_and_unsettled_window():
    trace=synthetic_trace()
    for row in trace['samples']['arm_state'][280:300]:row['joints'][7]['position']+=.01
    report=evaluate_trace(trace)
    assert report['verdict']=='fail' and failed(report,'nonselected_axis_drift')
    trace=synthetic_trace();trace['samples']['arm_state'][280]['joints'][0]['position']+=.003
    report=evaluate_trace(trace)
    assert report['verdict']=='fail' and failed(report,'settled_window_spread')


def test_repeatability_requires_distinct_compatible_passing_sessions(tmp_path):
    reports=[]
    for i in range(3):
        path=tmp_path/f'{i}.json';path.write_text(json.dumps(synthetic_trace(clock_offset=i*10_000_000_000)))
        reports.append(evaluate_file(path))
    assert evaluate_repeatability(reports)['verdict']=='pass_under_criteria'
    assert evaluate_repeatability(reports[:2])['verdict']=='not_passed'
    assert evaluate_repeatability([reports[0]]*3)['verdict']=='not_passed'
    incompatible=deepcopy(reports);incompatible[1]['delta_rad']=.01
    assert evaluate_repeatability(incompatible)['verdict']=='not_passed'
    spread=deepcopy(reports);spread[1]['metrics']['encoder_target_delta_rad']+=.0015
    assert evaluate_repeatability(spread)['verdict']=='not_passed'
    overlap=deepcopy(reports);overlap[1]['start_monotonic_ns']=2_000_000_000
    assert evaluate_repeatability(overlap)['verdict']=='not_passed'


@pytest.mark.parametrize('options',[{'window_s':float('nan')},{'window_s':2.},{'min_repetitions':2},{'min_window_samples':2.5}])
def test_invalid_criteria_rejected(options):
    with pytest.raises(ValueError):AcceptanceCriteria(**options)


def test_cli_offline_hashes_and_nonzero_for_failed_acceptance(tmp_path, monkeypatch, capsys):
    from lingxi_x2.cli import main
    def forbidden(*args, **kwargs):pytest.fail('Acceptance analysis must not initialize ROS/client')
    monkeypatch.setattr('lingxi_x2.cli.X2Client',forbidden)
    monkeypatch.setattr('lingxi_x2.cli.reexec_with_ros_environment',forbidden)
    paths=[]
    for i in range(3):
        p=tmp_path/f'{i}.json';p.write_text(json.dumps(synthetic_trace(clock_offset=i*10_000_000_000)));paths.append(str(p))
    assert main(['acceptance-report',*paths])==0
    report=json.loads(capsys.readouterr().out)
    assert report['repeatability']['verdict']=='pass_under_criteria'
    assert not report['execution_authorized']
    assert len(report['sessions'][0]['source_sha256'])==64
    assert main(['acceptance-report',paths[0]])==3
    capsys.readouterr()
    p=tmp_path/'bad.json';p.write_text('{"kind":"bad","kind":"bad"}')
    assert main(['acceptance-report',str(p)])==2


def test_repeatability_does_not_mix_trajectory_profiles_or_durations(tmp_path):
    reports=[]
    for i in range(3):
        path=tmp_path/f'{i}.json';path.write_text(json.dumps(synthetic_trace(clock_offset=i*10_000_000_000)))
        reports.append(evaluate_file(path))
    for field,value in [('trajectory_profile','quintic'),('ramp_duration_s',2.)]:
        changed=deepcopy(reports);changed[1][field]=value
        result=evaluate_repeatability(changed)
        assert result['verdict']=='not_passed'
        assert any('trajectory profile' in p for p in result['problems'])
