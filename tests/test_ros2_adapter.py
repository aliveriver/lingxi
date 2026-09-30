from __future__ import annotations

from types import SimpleNamespace
from dataclasses import replace
import threading
import time

import pytest

from lingxi_x2.backends.ros2 import (
    HAND_DEXTEROUS_JOINT,
    _enum_value,
    populate_upper_body_message,
    validate_set_mc_action_response,
)
from lingxi_x2.errors import SafetyInterlockError
from lingxi_x2.models import Side
from lingxi_x2.backends.ros2 import Ros2Backend, UPPER_BODY_TOPIC
from lingxi_x2.backends.mock import MockBackend
from lingxi_x2.config import PlatformConfig, ControlConfig


def _message():
    return SimpleNamespace(header=SimpleNamespace(stamp=None, frame_id="", sequence=0))


def test_upper_body_message_uses_official_v1_1_layout() -> None:
    arm = tuple(float(index) for index in range(14))
    left = tuple(100.0 + index for index in range(10))
    right = tuple(200.0 + index for index in range(10))
    message = populate_upper_body_message(
        _message(),
        "stamp",
        42,
        arm,
        {Side.LEFT: left, Side.RIGHT: right},
    )

    assert message.header.stamp == "stamp"
    assert message.header.frame_id == "mc_upper_body"
    assert message.header.sequence == 42
    assert message.source == "lingxi_x2"
    assert message.hand_sub_mode == HAND_DEXTEROUS_JOINT
    assert message.head_pos == [0.0, 0.0]
    assert message.arm_pos == list(arm)
    assert message.hand_pos == list(left + right)


def test_upper_body_message_rejects_wrong_hand_length() -> None:
    with pytest.raises(SafetyInterlockError, match="left hand target"):
        populate_upper_body_message(
            _message(),
            "stamp",
            0,
            (0.0,) * 14,
            {Side.LEFT: (0.0,) * 9, Side.RIGHT: (0.0,) * 10},
        )


def test_enum_value_accepts_wrapped_and_primitive_values() -> None:
    assert _enum_value(SimpleNamespace(value=100)) == 100
    assert _enum_value(100) == 100
    assert _enum_value(None, -1) == -1


def test_set_mc_action_requires_success_code_and_status() -> None:
    success = SimpleNamespace(
        response=SimpleNamespace(
            header=SimpleNamespace(code=0),
            status=SimpleNamespace(value=1),
            message="",
        )
    )
    validate_set_mc_action_response(success, "UPPERBODY_REMOTE_SPLIT")

    rejected = SimpleNamespace(
        response=SimpleNamespace(
            header=SimpleNamespace(code=6),
            status=SimpleNamespace(value=0),
            message="protected",
        )
    )
    with pytest.raises(SafetyInterlockError, match="SecureLevelForbid"):
        validate_set_mc_action_response(rejected, "UPPERBODY_REMOTE_SPLIT")


@pytest.fixture
def stream_backend():
    config = PlatformConfig(backend="mock", control=ControlConfig(enabled=True, authority="upper_body_mc"))
    mock = MockBackend(config)
    mock.start()
    backend = Ros2Backend.__new__(Ros2Backend)
    backend.config = config
    backend._condition = threading.Condition()
    backend._stream_lock = threading.Lock()
    backend._stream_owner = backend._stream_subsystem = None
    backend._arm = mock.arm_state()
    backend._hands = dict(mock.hand_states())
    backend._mc_state = mock.mc_state()
    mock.close()
    backend._held_hands = {}
    backend._upper_sequence = 2**32 - 1
    backend._upper_publisher = None
    backend._echo_subscription = None
    backend._last_command_echo = None
    backend._last_upper_command = None
    backend._hal_arm_command = (backend._arm.timestamp, (0.,) * 14)
    backend._hal_arm_error = None
    backend._stream_feedback_error = None
    backend._monitor_feedback = False
    backend._feedback_stamps = {}
    backend._hold_provenance = {}
    backend._subscriptions = []
    backend._types = {"UpperBodyCommandArray": _message, "McCommonState": object}
    backend._qos = None
    messages, graph_calls, timers = [], [], []
    publisher = SimpleNamespace(publish=messages.append, get_subscription_count=lambda: 1)
    def graph():
        graph_calls.append(True)
        return [(UPPER_BODY_TOPIC, ["aimdk_msgs/msg/UpperBodyCommandArray"])]
    backend._node = SimpleNamespace(
        get_topic_names_and_types=graph,
        get_publishers_info_by_topic=lambda topic: [],
        get_subscriptions_info_by_topic=lambda topic: [],
        get_fully_qualified_name=lambda: "/test",
        create_publisher=lambda *args: publisher,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=time.time_ns(), to_msg=lambda: SimpleNamespace(sec=1, nanosec=0))),
        create_timer=lambda period, callback: timers.append(callback) or callback,
        destroy_timer=timers.remove,
    )
    return backend, messages, graph_calls, timers


def send_arm(backend):
    return backend.publish_arm((0.0,) * 14, (0.0,) * 14, (0.0,) * 14, 20., 2.)


def test_stream_publishes_without_graph_reads_and_revokes_on_exit(stream_backend, monkeypatch):
    backend, messages, graph_calls, timers = stream_backend
    with pytest.raises(SafetyInterlockError, match="command_stream"):
        send_arm(backend)
    with backend.command_stream("arm"):
        count = len(graph_calls)
        monkeypatch.setattr(backend, "control_preflight", lambda *args: pytest.fail("per-frame preflight"))
        monkeypatch.setattr(backend, "hand_states", lambda *args: pytest.fail("per-frame hand wait"))
        first = send_arm(backend)
        second = send_arm(backend)
        assert len(graph_calls) == count
        assert first["sequence"] == 2**32 - 1
        assert second["sequence"] == 0
        assert second["publish_return_monotonic_ns"] >= second["monotonic_ns"]
    assert not timers
    with pytest.raises(SafetyInterlockError, match="command_stream"):
        send_arm(backend)
    assert len(messages) == 2


@pytest.mark.parametrize("failure", ["stale", "fault", "mode", "domain", "hand_type", "graph_stale", "competitor", "subscriber", "disabled"])
def test_stream_safety_change_stops_before_next_publish(stream_backend, failure):
    backend, messages, _, timers = stream_backend
    with pytest.raises(SafetyInterlockError):
        with backend.command_stream("arm"):
            send_arm(backend)
            if failure == "stale":
                backend._arm = replace(backend._arm, timestamp=replace(backend._arm.timestamp, monotonic_ns=0))
            elif failure == "fault":
                backend._arm = replace(backend._arm, joints=(replace(backend._arm.joints[0], fault_code=1), *backend._arm.joints[1:]))
            elif failure == "mode":
                backend._mc_state = replace(backend._mc_state, action="STAND_DEFAULT")
            elif failure == "domain":
                backend._arm = replace(backend._arm, domain_state=2)
            elif failure == "hand_type":
                backend._hands[Side.LEFT] = replace(backend._hands[Side.LEFT], hand_type=0)
            elif failure == "graph_stale":
                backend._graph_guard = (0, None)
            elif failure == "competitor":
                backend._node.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(node_namespace="/", node_name="other")]
                timers[0]()
            elif failure == "subscriber":
                backend._upper_publisher.get_subscription_count = lambda: 0
                timers[0]()
            else:
                backend.config.control.enabled = False
            send_arm(backend)
    assert len(messages) == 1
    assert backend._stream_owner is None
    assert not timers


def test_hand_stream_freezes_other_limbs_and_echo_compares_full_command(stream_backend):
    backend, messages, _, _ = stream_backend
    with backend.command_stream("hand"):
        original_arm = backend._held_arm
        backend._arm = replace(backend._arm, joints=tuple(replace(j, position_rad=.01) for j in backend._arm.joints))
        backend.publish_hand(Side.RIGHT, (.1,) * 10)
        assert messages[-1].arm_pos == list(original_arm)
        assert messages[-1].hand_pos == [0.] * 10 + [.1] * 10
        backend._on_command_echo(messages[-1])
        assert backend._last_command_echo is not None
        backend._last_command_echo = None
        messages[-1].source = "other"
        backend._on_command_echo(messages[-1])
        assert backend._last_command_echo is None


def test_local_echo_does_not_count_as_external_subscriber(stream_backend):
    backend, _, _, _ = stream_backend
    backend._upper_publisher = SimpleNamespace(get_subscription_count=lambda: 1)
    backend._echo_subscription = object()
    backend._node.get_subscriptions_info_by_topic = lambda topic: [SimpleNamespace(node_namespace="/", node_name="test")]
    backend._check_stream_graph()
    assert backend._graph_guard[1] == "upper-body subscriber lost"


def test_trace_and_echo_subscriptions_do_not_mask_external_subscriber_loss(stream_backend):
    backend, _, _, _ = stream_backend
    backend._upper_publisher = SimpleNamespace(get_subscription_count=lambda: 2)
    backend._node.get_subscriptions_info_by_topic = lambda topic: [
        SimpleNamespace(node_namespace="/", node_name="test") for _ in range(2)]
    backend._check_stream_graph()
    assert backend._graph_guard[1] == "upper-body subscriber lost"


@pytest.mark.parametrize("subsystem", ["arm", "hand"])
@pytest.mark.parametrize("side,index,value", [(Side.RIGHT, 3, -3.19638671875), (Side.LEFT, 4, -23.549616699)])
def test_historical_hand_anomaly_never_creates_command_publisher(stream_backend, subsystem, side, index, value):
    backend, messages, _, _ = stream_backend
    hand = backend._hands[side]
    joints = list(hand.joints)
    joints[index] = replace(joints[index], position_rad=value)
    backend._hands[side] = replace(hand, joints=tuple(joints))
    report = backend.control_preflight(subsystem, require_enabled=False)
    assert not report.ready
    assert any(c.name == "feedback_validity" and not c.passed for c in report.checks)
    with pytest.raises(SafetyInterlockError, match="Implausible"):
        with backend.command_stream(subsystem):
            pytest.fail("Anomalous feedback opened a command stream")
    assert backend._upper_publisher is None and not messages


@pytest.mark.parametrize("method", ["move_arm", "move_hand", "hold_upper_body"])
def test_public_motion_methods_cannot_bypass_hand_anomaly(stream_backend, method):
    from lingxi_x2.client import X2Client
    from lingxi_x2.models import ArmCommand, HandCommand
    backend, messages, _, _ = stream_backend
    hand = backend._hands[Side.RIGHT]
    backend._hands[Side.RIGHT] = replace(hand, joints=(replace(hand.joints[0], position_rad=-23.), *hand.joints[1:]))
    client = X2Client(backend.config, backend=backend)
    args = {"move_arm": [ArmCommand.from_positions((0.,)*14, .02)],
            "move_hand": [HandCommand.from_positions("left", (.1,)*10, .02)], "hold_upper_body": [.02]}
    with pytest.raises(SafetyInterlockError, match="Implausible"):
        getattr(client, method)(*args[method], confirm_hardware=True)
    assert not messages and backend._upper_publisher is None


@pytest.mark.parametrize("damage", ["unknown_fault", "unknown_domain", "arm_order", "hand_order", "hand_type",
                                   "source_stale", "source_future", "receive_future", "hand_anomaly"])
def test_feedback_damage_blocks_next_frame(stream_backend, damage):
    backend, messages, _, _ = stream_backend
    with backend.command_stream("arm"):
        send_arm(backend)
        arm = backend._arm
        if damage == "unknown_fault":
            backend._arm = replace(arm, joints=(replace(arm.joints[0], fault_code=None), *arm.joints[1:]))
        elif damage == "unknown_domain": backend._arm = replace(arm, domain_state=None)
        elif damage == "arm_order": backend._arm = replace(arm, joints=arm.joints[::-1])
        elif damage in ("hand_order", "hand_type", "hand_anomaly"):
            hand = backend._hands[Side.RIGHT]
            backend._hands[Side.RIGHT] = (replace(hand, joints=hand.joints[::-1]) if damage == "hand_order" else
                replace(hand, hand_type=0) if damage == "hand_type" else
                replace(hand, joints=(replace(hand.joints[0], position_rad=-3.2), *hand.joints[1:])))
        else:
            stamp = arm.timestamp
            if damage == "source_stale": stamp = replace(stamp, sec=stamp.sec-10)
            elif damage == "source_future": stamp = replace(stamp, sec=stamp.sec+10)
            else: stamp = replace(stamp, monotonic_ns=time.monotonic_ns()+10**9)
            backend._arm = replace(arm, timestamp=stamp)
        with pytest.raises(SafetyInterlockError): send_arm(backend)
    assert len(messages) == 1


def test_missing_fault_field_remains_unknown():
    joint = Ros2Backend._joint(SimpleNamespace(position=0.), "test")
    assert joint.fault_code is None and joint.state is None


def test_intervening_hand_fault_is_latched_even_after_good_callback(stream_backend):
    backend, messages, _, _ = stream_backend
    backend._extract_tactile = lambda *_: {}
    backend._sample_seen = {}
    def hand_message(fault):
        now = time.time_ns()
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=now//10**9, nanosec=now%10**9)),
            left_hand_type=SimpleNamespace(value=1), right_hand_type=SimpleNamespace(value=1),
            left_hands=[SimpleNamespace(name=j.name, position=j.position_rad, faultcode=fault) for j in backend._hands[Side.LEFT].joints],
            right_hands=[SimpleNamespace(name=j.name, position=j.position_rad, faultcode=0) for j in backend._hands[Side.RIGHT].joints])
    with backend.command_stream("arm"):
        send_arm(backend)
        backend._on_hand(hand_message(2))
        backend._on_hand(hand_message(0))
        assert all(j.fault_code == 0 for j in backend._hands[Side.LEFT].joints)
        with pytest.raises(SafetyInterlockError, match="Latched"): send_arm(backend)
    assert len(messages) == 1


def test_hand_action_keeps_hal_arm_target_not_encoder_pose(stream_backend):
    backend, messages, _, _ = stream_backend
    hal = (.4, 0., 0., -1.2, 0., 0., 0.) * 2
    backend._hal_arm_command = (backend._arm.timestamp, hal)
    with backend.command_stream("hand"):
        backend.publish_hand(Side.RIGHT, (.1,)*10)
    assert messages[0].arm_pos == list(hal)
    assert backend.stream_diagnostics()["hold_targets"]["arm_source"] == "fresh_hal_arm_command"


def test_frozen_hand_targets_survive_small_feedback_quantization(stream_backend):
    backend, messages, _, _ = stream_backend
    hand = backend._hands[Side.RIGHT]
    backend._hands[Side.RIGHT] = replace(hand, joints=tuple(replace(j, position_rad=.001) for j in hand.joints))
    fixed = {side: (0.,)*10 for side in Side}
    with backend.command_stream("arm", hand_targets=fixed):
        fixed[Side.RIGHT] = (1.,)*10  # Caller mutation cannot alter the captured target.
        send_arm(backend)
    assert messages[0].hand_pos == [0.]*20


@pytest.mark.parametrize("damage", ["missing", "stale", "reordered"])
def test_invalid_hal_arm_baseline_blocks_hand_stream(stream_backend, damage):
    backend, messages, _, _ = stream_backend
    if damage == "missing": backend._hal_arm_command = None
    elif damage == "stale":
        backend._hal_arm_command = (replace(backend._arm.timestamp, sec=1), (0.,)*14)
    else:
        backend._on_hal_arm_command(SimpleNamespace(joints=[]))
    with pytest.raises(SafetyInterlockError):
        with backend.command_stream("hand"): pytest.fail("Invalid baseline opened stream")
    assert not messages and backend._upper_publisher is None
