from types import SimpleNamespace

import pytest

from lingxi_x2.drive_trace import DriveTrace, review_drive_window
from lingxi_x2.errors import SafetyInterlockError


def recorder(capacity=3):
    subscriptions, destroyed = [], []
    def subscribe(cls, topic, callback, qos):
        subscriptions.append((topic, callback))
        return topic
    node = SimpleNamespace(create_subscription=subscribe, destroy_subscription=destroyed.append,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**12)))
    backend = SimpleNamespace(_node=node, _qos=None)
    trace = DriveTrace(backend, max_samples=capacity,
        message_types={'DcuMonitorData': object, 'JointNoRealTimeStateArray': object},
        converter=lambda msg: {'raw_current': 255})
    return trace, subscriptions, destroyed


def message():
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=999, nanosec=10)))


def test_passive_drive_capture_retains_clocks_raw_values_and_cleans_up():
    trace, subs, destroyed = recorder()
    with trace:
        for _, callback in subs:
            callback(message())
        snapshot = trace.snapshot()
        row = snapshot['samples']['udcu'][0]
        assert row['message']['raw_current'] == 255
        assert row['stamp_ns'] == 999_000_000_010
        assert row['received_ros_ns'] == 10**12
        trace.require_recent(now_ns=max(r[0]['received_monotonic_ns'] for r in snapshot['samples'].values()))
        assert not snapshot['units_verified'] and not snapshot['control_mode_decoded']
    assert len(destroyed) == 2
    subs[0][1](message())
    assert trace.snapshot()['counts'] == {'udcu': 1, 'joints': 1}


def test_headerless_installed_drive_schema_retains_raw_data_without_inventing_time():
    trace, subs, _ = recorder()
    with trace:
        subs[0][1](SimpleNamespace())
        row = trace.snapshot()['samples']['udcu'][0]
        assert row['stamp_ns'] is None
        assert row['message'] == {'raw_current': 255}


def test_drive_overflow_latches_loss_and_refuses_readiness():
    trace, subs, _ = recorder(1)
    with trace:
        for _ in range(2):
            for _, callback in subs:
                callback(message())
        snapshot = trace.snapshot()
        assert snapshot['dropped'] == {'udcu': 1, 'joints': 1}
        with pytest.raises(SafetyInterlockError):
            trace.require_recent(now_ns=snapshot['samples']['joints'][-1]['received_monotonic_ns'])


def test_conversion_failure_stays_latched_after_good_sample():
    trace, subs, _ = recorder()
    with trace:
        trace.converter = lambda msg: []
        for _ in range(30):
            subs[0][1](message())
        trace.converter = lambda msg: {'raw_current': 1}
        for _, callback in subs:
            callback(message())
        snapshot = trace.snapshot()
        assert snapshot['error_count'] == 30 and len(snapshot['errors']) == 20
        with pytest.raises(SafetyInterlockError):
            trace.require_recent(now_ns=snapshot['samples']['joints'][-1]['received_monotonic_ns'])


@pytest.mark.parametrize('age', [-1, 1_000_000_001])
def test_drive_future_or_stale_receive_time_cannot_release_trial(age):
    trace, subs, _ = recorder()
    with trace:
        for _, callback in subs:
            callback(message())
        snapshot = trace.snapshot()
        times = [rows[-1]['received_monotonic_ns'] for rows in snapshot['samples'].values()]
        with pytest.raises(SafetyInterlockError):
            trace.require_recent(now_ns=(min(times) if age < 0 else max(times)) + age)


def test_partial_subscription_failure_cleans_up_and_is_reported():
    trace, subs, destroyed = recorder()
    del trace.types['JointNoRealTimeStateArray']
    with trace:
        assert trace.unavailable_reason
        with pytest.raises(SafetyInterlockError):
            trace.require_recent(now_ns=10**12)
    assert len(subs) == len(destroyed) == 1


@pytest.mark.parametrize('damage', ['missing', 'unordered', 'dropped', 'error'])
def test_window_rejects_missing_or_lost_raw_evidence(damage):
    trace = {'samples': {key: [{'received_monotonic_ns': t, 'stamp_ns': None}
        for t in (2, 3)] for key in ('udcu', 'joints')}, 'dropped': {'udcu': 0, 'joints': 0}, 'error_count': 0}
    if damage == 'missing': trace['samples']['joints'] = []
    elif damage == 'unordered': trace['samples']['udcu'].reverse()
    elif damage == 'dropped': trace['dropped']['udcu'] = 1
    else: trace['error_count'] = 1
    report = review_drive_window(trace, 1, 4)
    assert not report['raw_samples_available']
    assert not report['drive_cause_identified']


def test_window_rejects_long_outage_even_when_final_sample_is_recent():
    trace = {'samples': {key: [{'received_monotonic_ns': t, 'stamp_ns': t}
        for t in (1_000_000_001, 4_000_000_000)] for key in ('udcu', 'joints')}}
    report = review_drive_window(trace, 1_000_000_000, 4_000_000_001)
    assert not report['raw_samples_available']
    assert any('coverage gap' in p for p in report['problems'])


def test_window_rejects_repeated_source_stamp_despite_fresh_receives():
    trace = {'samples': {key: [{'received_monotonic_ns': t, 'stamp_ns': 5}
        for t in (2, 3)] for key in ('udcu', 'joints')}}
    assert not review_drive_window(trace, 1, 4)['raw_samples_available']
