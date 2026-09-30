from dataclasses import replace
from types import SimpleNamespace
import pytest
from lingxi_x2.models import ARM_JOINT_NAMES, ArmState, JointSample, Timestamp
from lingxi_x2.errors import SafetyInterlockError
from lingxi_x2.tracking_guard import check_tracking
from lingxi_x2.publication import publish_points


def state(now=0):
    return ArmState(Timestamp(0, 0, now), tuple(
        JointSample(n, 0., 0., 0., fault_code=0) for n in ARM_JOINT_NAMES), domain_state=0)


def test_every_axis_checked_and_boundary_inclusive():
    for i in range(14):
        joints=list(state().joints);joints[i]=replace(joints[i], position_rad=-.02)
        arm=replace(state(), joints=tuple(joints))
        check_tracking((0.,)*14, arm, now_ns=0, max_error_rad=.02)
        with pytest.raises(SafetyInterlockError, match='Tracking error'):
            check_tracking((0.,)*14, arm, now_ns=0, max_error_rad=.019)


@pytest.mark.parametrize('damage', ['stale','future','fault','unknown_fault','nan','missing','reordered','domain','unknown_domain'])
def test_bad_feedback_rejected(damage):
    arm=state();now=0
    if damage=='stale': now=100_000_001
    if damage=='future': now=-1
    if damage in ('fault','unknown_fault','nan'):
        changes={'fault_code':1} if damage=='fault' else {'fault_code':None} if damage=='unknown_fault' else {'position_rad':float('nan')}
        arm=replace(arm,joints=(replace(arm.joints[0],**changes),*arm.joints[1:]))
    if damage=='missing': arm=replace(arm,joints=arm.joints[:-1])
    if damage=='reordered': arm=replace(arm,joints=arm.joints[::-1])
    if damage in ('domain','unknown_domain'):arm=replace(arm,domain_state=1 if damage=='domain' else None)
    with pytest.raises(SafetyInterlockError):check_tracking((0.,)*14,arm,now_ns=now,max_error_rad=.02)


@pytest.mark.parametrize('bound',[0.,-1.,float('nan'),float('inf')])
def test_no_invalid_or_implicit_tolerance(bound):
    with pytest.raises(ValueError):check_tracking((0.,)*14,state(),now_ns=0,max_error_rad=bound)


def fake_time(monkeypatch):
    clock=SimpleNamespace(ns=0,extra_sleep=0)
    def sleep(seconds):clock.ns+=round(seconds*1e9)+clock.extra_sleep
    monkeypatch.setattr('lingxi_x2.publication.time',SimpleNamespace(
        monotonic=lambda:clock.ns/1e9,monotonic_ns=lambda:clock.ns,sleep=sleep))
    return clock


@pytest.mark.parametrize('cause',['wake_delay','validation_delay','publisher_delay'])
def test_gap_abort_never_sends_next_target(monkeypatch,cause):
    clock=fake_time(monkeypatch);sent=[];frames=[]
    def publish(point):
        start=clock.ns;sent.append(point)
        if cause=='wake_delay':clock.extra_sleep=60_000_000
        if cause=='publisher_delay':clock.ns+=60_000_000
        return {'monotonic_ns':start}
    def validate(point):
        if sent and cause=='validation_delay':clock.ns+=60_000_000
    with pytest.raises(SafetyInterlockError,match='interval'):
        publish_points([('a',(0.,)),('b',(1.,))],publish,50,frames,
                       max_frame_interval_s=.05,before_publish=validate)
    assert sent==[(0.,)] and len(frames)==1


def test_stuck_encoder_stops_instead_of_accumulating_point_five_error(monkeypatch):
    clock=fake_time(monkeypatch);sent=[];frames=[]
    # Synthetic .02 tolerance proves software behavior, not hardware suitability.
    def validate(point):check_tracking(point,state(clock.ns),now_ns=clock.ns,max_error_rad=.02)
    def publish(point):sent.append(point)
    plan=[('ramp',(i*.01,)+(0.,)*13) for i in range(51)]
    with pytest.raises(SafetyInterlockError,match='Tracking error'):
        publish_points(plan,publish,50,frames,max_frame_interval_s=.05,before_publish=validate)
    assert len(sent)==3 and sent[-1][0]==.02
    assert len(frames)==3  # No automatic retreat or mode request is made by this layer.


def test_feedback_checked_after_sleep(monkeypatch):
    clock=fake_time(monkeypatch);sent=[]
    def validate(point):check_tracking(point,state(),now_ns=clock.ns,max_error_rad=.02,max_age_ns=10_000_000)
    with pytest.raises(SafetyInterlockError,match='Stale'):
        publish_points([('a',(0.,)*14)]*2,lambda q:sent.append(q),50,[],
                       max_frame_interval_s=.05,before_publish=validate)
    assert len(sent)==1


def test_last_publish_overrun_is_recorded_then_rejected(monkeypatch):
    clock=fake_time(monkeypatch);frames=[]
    def publish(point):
        start=clock.ns;clock.ns+=60_000_000
        return {'monotonic_ns':start}
    with pytest.raises(SafetyInterlockError,match='interval'):
        publish_points([('final',(0.,))],publish,50,frames,max_frame_interval_s=.05)
    assert len(frames)==1 and frames[0]['phase']=='final'
