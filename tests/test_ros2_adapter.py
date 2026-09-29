from __future__ import annotations

from types import SimpleNamespace
from dataclasses import replace
import threading

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
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=1, nanosec=0))),
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
