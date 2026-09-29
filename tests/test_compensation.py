import copy
from dataclasses import replace
import json
import math

import numpy as np
import pytest

from lingxi_x2.backends.mock import MockBackend
from lingxi_x2.client import X2Client
from lingxi_x2.cli import main
from lingxi_x2.compensation import CompensationRequest, plan_compensated_joint_session, quintic
from lingxi_x2.config import PlatformConfig
from lingxi_x2.errors import SafetyInterlockError
from lingxi_x2.models import ARM_JOINT_NAMES
from test_gravity import urdf  # Synthetic analytically soluble two-arm fixture.


def scenario():
    q = dict.fromkeys(ARM_JOINT_NAMES, 0.)
    target = dict(q)
    target[ARM_JOINT_NAMES[0]] = .01
    return dict(baseline_rad=q, target_rad=target,
                motion_joints=[ARM_JOINT_NAMES[0]], compensation_joints=[ARM_JOINT_NAMES[0]],
                enabled=True, gravity_torso_m_s2=[0, 0, -9.81],
                payloads={side: dict(mass_kg=0., com_wrist_m=[0, 0, 0]) for side in ('left', 'right')},
                assumed_stiffness_nm_rad=40., bias_limit_rad=.01,
                max_bias_rate_rad_s=.03, max_command_rate_rad_s=.05,
                max_command_excursion_rad=.02, rate_hz=50.,
                ramp_duration_s=1., move_duration_s=1., hold_duration_s=.1)


def config():
    return PlatformConfig(backend='mock', control={'enabled': True, 'publish_rate_hz': 50})


def test_quintic_boundary_and_midpoint():
    assert [quintic(p) for p in (0, .5, 1)] == [0, .5, 1]
    # Endpoint velocity and acceleration converge to zero with a cubic tail.
    assert quintic(1e-4) < 1e-10
    assert 1 - quintic(1 - 1e-4) < 1e-10
    for p in (-.1, 1.1, float('nan')):
        with pytest.raises(ValueError):
            quintic(p)


def test_clipping_mask_smooth_return_and_analytic_torque(urdf):
    request = scenario()
    original = copy.deepcopy(request)
    plan = plan_compensated_joint_session(urdf, request)
    assert request == original
    frames = plan['frames']
    assert frames[0]['command_rad'] == frames[-1]['command_rad'] == [0.] * 14
    assert plan['summary']['saturated_frame_count'] > 0
    assert not plan['hardware_validated'] and not plan['hardware_execution_allowed']
    assert all(f['command_rad'][1:] == [0.] * 13 for f in frames)
    for frame in frames:
        expected_tau = -2 * 9.81 * math.cos(frame['desired_rad'][0])
        assert frame['gravity_torque_nm'][0] == pytest.approx(expected_tau)
        assert frame['unbounded_bias_rad'][0] == pytest.approx(expected_tau / 40)
        assert frame['applied_bias_rad'][0] == pytest.approx(-.01 * frame['bias_fraction'])
        np.testing.assert_allclose(np.array(frame['desired_rad']) + frame['applied_bias_rad'], frame['command_rad'])
    command = np.array([f['command_rad'] for f in frames])
    bias = np.array([f['applied_bias_rad'] for f in frames])
    assert np.max(np.abs(np.diff(command, axis=0))) * 50 <= .05
    assert np.max(np.abs(np.diff(bias, axis=0))) * 50 <= .03
    assert max(abs(command[:, 0])) <= .02
    # Same desired baseline yields exactly the same full compensation on return.
    after_ramp = [f for f in frames if f['phase'] == 'bias_ramp_in'][-1]
    after_return = [f for f in frames if f['phase'] == 'return'][-1]
    assert after_ramp['command_rad'] == after_return['command_rad']


def test_unsaturated_equation_and_disabled_comparison(urdf):
    request = scenario()
    request['assumed_stiffness_nm_rad'] = 4000.
    enabled = plan_compensated_joint_session(urdf, request)
    assert enabled['summary']['saturated_frame_count'] == 0
    for f in enabled['frames']:
        assert f['applied_bias_rad'][0] == pytest.approx(f['gravity_torque_nm'][0] / 4000 * f['bias_fraction'])
    request['enabled'] = False
    disabled = plan_compensated_joint_session(urdf, request)
    assert [f['desired_rad'] for f in enabled['frames']] == [f['desired_rad'] for f in disabled['frames']]
    assert all(f['command_rad'] == f['desired_rad'] and f['applied_bias_rad'] == [0.] * 14 for f in disabled['frames'])


@pytest.mark.parametrize('field,value', [
    ('bias_limit_rad', .03), ('assumed_stiffness_nm_rad', 0.),
    ('assumed_stiffness_nm_rad', float('nan')), ('enabled', 'true'),
    ('rate_hz', 0.), ('ramp_duration_s', float('inf')),
    ('gravity_torso_m_s2', [0, 0, 0]), ('compensation_joints', ['bad']),
    ('compensation_joints', [ARM_JOINT_NAMES[0]] * 2),
    ('compensation_joints', []), ('payloads', {}), ('unknown', 1),
])
def test_bad_configuration_rejected(field, value):
    request = scenario()
    request[field] = value
    with pytest.raises(ValueError):
        CompensationRequest.model_validate(request)


@pytest.mark.parametrize('change,match', [
    ('unselected', 'Unselected'), ('delta', 'excursion'), ('bias_rate', 'Bias rate'),
    ('command_rate', 'Command rate'), ('envelope', 'envelope'), ('joint_limit', 'outside'),
])
def test_whole_plan_rejected_before_stream(urdf, change, match, tmp_path, monkeypatch):
    request = scenario()
    if change == 'unselected':
        request['target_rad'][ARM_JOINT_NAMES[7]] = .01
    elif change == 'delta':
        request['target_rad'][ARM_JOINT_NAMES[0]] = .021
    elif change == 'bias_rate':
        request['ramp_duration_s'] = .1
    elif change == 'command_rate':
        request['max_command_rate_rad_s'] = .001
    elif change == 'envelope':
        request['max_command_excursion_rad'] = .005
    elif change == 'joint_limit':
        request['baseline_rad'][ARM_JOINT_NAMES[0]] = -3.
        request['target_rad'][ARM_JOINT_NAMES[0]] = -2.99
        # Synthetic positive gravity bias pushes shoulder beyond its URDF limit.
        request['gravity_torso_m_s2'] = [0, 0, 9.81]
    with X2Client(config()) as client:
        monkeypatch.setattr(client._backend, 'command_stream', lambda *a: pytest.fail('Invalid plan opened stream'))
        with pytest.raises((ValueError, SafetyInterlockError), match=match):
            client.run_mock_compensated_session(urdf, request, journal=tmp_path / 'never.jsonl')
    assert not (tmp_path / 'never.jsonl').exists()


def test_offline_cli_and_exclusive_output(urdf, tmp_path, monkeypatch, capsys):
    def forbidden(*a, **kw):
        pytest.fail('Offline command touched ROS/client')
    monkeypatch.setattr('lingxi_x2.cli.X2Client', forbidden)
    monkeypatch.setattr('lingxi_x2.cli.reexec_with_ros_environment', forbidden)
    request = tmp_path / 'scenario.json'
    request.write_text(json.dumps(scenario()))
    output = tmp_path / 'plan.json'
    args = ['--config', 'missing.yaml', 'plan-compensation', '--urdf', str(urdf), '--input', str(request), '--output', str(output)]
    assert main(args) == 0
    before = output.read_bytes()
    assert main(args) == 2
    assert output.read_bytes() == before
    assert json.loads(before)['frames'][-1]['command_rad'] == [0.] * 14
    capsys.readouterr()


def test_mock_execution_one_stream_and_preserved_hands(urdf, tmp_path, monkeypatch):
    monkeypatch.setattr('lingxi_x2.publication.time.sleep', lambda _: None)
    with X2Client(config()) as client:
        from lingxi_x2.models import Side
        client._backend.publish_hand(Side.LEFT, (.2,) * 10)
        calls = []
        original = client._backend.control_preflight
        def preflight(*args, **kwargs):
            calls.append(args)
            return original(*args, **kwargs)
        monkeypatch.setattr(client._backend, 'control_preflight', preflight)
        result = client.run_mock_compensated_session(urdf, scenario(), journal=tmp_path / 'session.jsonl')
        assert result['mock_returned_to_baseline'] and not result['hardware_validated']
        assert len(calls) == 1
        assert all(j.position_rad == .2 for j in client.hand_states()[Side.LEFT].joints)
        rows = [json.loads(line) for line in (tmp_path / 'session.jsonl').read_text().splitlines()]
        commands = [r for r in rows if r['kind'] == 'mock_command']
        assert len(commands) == result['publication']['published_count']
        assert commands[-1]['command_rad'] == [0.] * 14
        assert all(r['publish_return_monotonic_ns'] >= r['publish_monotonic_ns'] for r in commands)


def test_actual_backend_gate_even_when_config_mock(urdf, tmp_path, monkeypatch):
    with X2Client(config()) as client:
        status = client.status()
        monkeypatch.setattr(client._backend, 'status', lambda: replace(status, backend='ros2'))
        monkeypatch.setattr(client._backend, 'command_stream', lambda *a: pytest.fail('Hardware stream opened'))
        with pytest.raises(SafetyInterlockError, match='mock-only'):
            client.run_mock_compensated_session(urdf, scenario(), journal=tmp_path / 'never.jsonl')
    assert not (tmp_path / 'never.jsonl').exists()


@pytest.mark.parametrize('change', ['stale', 'future', 'wrong_names', 'fault', 'baseline', 'rate', 'disabled'])
def test_execution_guards(urdf, tmp_path, monkeypatch, change):
    with X2Client(config()) as client:
        arm = client.arm_state()
        if change in {'stale', 'future'}:
            arm = replace(arm, timestamp=replace(arm.timestamp, monotonic_ns=arm.timestamp.monotonic_ns + (10**12 if change == 'future' else -10**12)))
        elif change == 'wrong_names':
            arm = replace(arm, joints=tuple(reversed(arm.joints)))
        elif change == 'fault':
            arm = replace(arm, joints=(replace(arm.joints[0], fault_code=1),) + arm.joints[1:])
        elif change == 'baseline':
            arm = replace(arm, joints=(replace(arm.joints[0], position_rad=.005),) + arm.joints[1:])
        elif change == 'rate':
            client.config.control.publish_rate_hz = 25.
        elif change == 'disabled':
            client.config.control.enabled = False
        if change not in {'rate', 'disabled'}:
            monkeypatch.setattr(client, 'arm_state', lambda: arm)
        monkeypatch.setattr(client._backend, 'publish_arm', lambda *a: pytest.fail('Invalid execution published'))
        with pytest.raises(SafetyInterlockError):
            client.run_mock_compensated_session(urdf, scenario(), journal=tmp_path / 'session.jsonl')


def test_midstream_stale_feedback_leaves_abort_evidence(urdf, tmp_path, monkeypatch):
    monkeypatch.setattr('lingxi_x2.publication.time.sleep', lambda _: None)
    with X2Client(config()) as client:
        original = client.arm_state
        count = 0
        def state():
            nonlocal count
            count += 1
            value = original()
            if count > 4:
                value = replace(value, timestamp=replace(value.timestamp, monotonic_ns=1))
            return value
        monkeypatch.setattr(client, 'arm_state', state)
        with pytest.raises(SafetyInterlockError, match='stale'):
            client.run_mock_compensated_session(urdf, scenario(), journal=tmp_path / 'abort.jsonl')
        rows = [json.loads(line) for line in (tmp_path / 'abort.jsonl').read_text().splitlines()]
        assert rows[-1]['kind'] == 'mock_compensation_aborted'
        assert rows[-1]['published_count'] == 3
        assert rows[-1]['return_to_baseline_verified'] is False


def test_replay_offline_and_rejects_modified_frames_or_model(urdf, tmp_path, monkeypatch):
    from lingxi_x2.compensation import read_compensation_plan, replay_compensation_plan
    plan = plan_compensated_joint_session(urdf, scenario())
    path = tmp_path / 'plan.json'
    path.write_text(json.dumps(plan))
    def forbidden(*args, **kwargs):
        pytest.fail('Replay initialized a client')
    monkeypatch.setattr('lingxi_x2.cli.X2Client', forbidden)
    monkeypatch.setattr('lingxi_x2.cli.reexec_with_ros_environment', forbidden)
    rows = list(replay_compensation_plan(path))
    assert len(rows) == len(plan['frames'])
    assert all(row['execution'] == 'none' for row in rows)
    assert rows[-1]['command_rad'] == rows[0]['command_rad']
    relocated = tmp_path / 'same-model.urdf'
    relocated.write_bytes(urdf.read_bytes())
    assert read_compensation_plan(path, urdf=relocated)['model'] == plan['model']
    for field in ('command_rad', 'desired_rad', 'applied_bias_rad'):
        edited = copy.deepcopy(plan)
        edited['frames'][3][field][0] += .001
        path.write_text(json.dumps(edited))
        with pytest.raises(ValueError, match='differs'):
            list(replay_compensation_plan(path))
    path.write_text(json.dumps(plan))
    relocated.write_bytes(urdf.read_bytes() + b'\n')
    with pytest.raises(ValueError, match='differs'):
        list(replay_compensation_plan(path, urdf=relocated))


def test_mock_cli_uses_explicit_simulator_and_rejects_auto(urdf, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr('lingxi_x2.publication.time.sleep', lambda _: None)
    monkeypatch.setattr('lingxi_x2.cli.reexec_with_ros_environment', lambda *a: pytest.fail('Touched ROS'))
    request = scenario()
    request['baseline_rad'][ARM_JOINT_NAMES[0]] = .4
    request['target_rad'][ARM_JOINT_NAMES[0]] = .41
    input_path = tmp_path / 'scenario.json'
    input_path.write_text(json.dumps(request))
    output = tmp_path / 'session.jsonl'
    args = ['mock-compensation', '--urdf', str(urdf), '--input', str(input_path), '--output', str(output)]
    assert main(args) == 2
    assert not output.exists()
    assert main(['--config', 'config/mock.yaml', *args]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['final_positions_rad'][0] == .4
    assert not result['hardware_validated']


@pytest.mark.parametrize('difference,accepted', [(1e-15, True), (1e-8, False)])
def test_replay_accepts_only_machine_roundoff(urdf, tmp_path, difference, accepted):
    from lingxi_x2.compensation import read_compensation_plan
    plan = plan_compensated_joint_session(urdf, scenario())
    plan['frames'][20]['gravity_torque_nm'][0] += difference
    plan['frames'][20]['command_rad'][0] += difference
    path = tmp_path / 'roundoff.json'
    path.write_text(json.dumps(plan))
    if accepted:
        assert read_compensation_plan(path) == plan
    else:
        with pytest.raises(ValueError, match='differs'):
            read_compensation_plan(path)
