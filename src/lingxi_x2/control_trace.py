"""Bounded, read-only ROS subscriptions for command-path diagnosis."""
from collections import deque
from contextlib import AbstractContextManager
import threading
import time


TOPICS = {
    "upper": ("/mc/upper_body_command", "UpperBodyCommandArray"),
    "hal_arm": ("/aima/hal/joint/arm/command", "JointCommandArray"),
    "arm_state": ("/aima/hal/joint/arm/state", "JointStateArray"),
    "mc": ("/aima/mc/common/state", "McCommonState"),
}


def trace_sample(kind, message):
    stamp = message.header.stamp
    result = {"received_monotonic_ns": time.monotonic_ns(),
              "stamp_ns": int(stamp.sec) * 10**9 + int(stamp.nanosec)}
    if kind == "upper":
        result.update(sequence=int(message.header.sequence), source=str(message.source),
                      arm_pos=list(message.arm_pos), hand_pos=list(message.hand_pos),
                      hand_sub_mode=int(message.hand_sub_mode))
    elif kind in {"hal_arm", "arm_state"}:
        fields = ("name", "position", "velocity", "effort", "stiffness", "damping") if kind == "hal_arm" else (
            "name", "position", "velocity", "effort", "error_code", "state")
        result["joints"] = [{key: getattr(joint, key, None) for key in fields} for joint in message.joints]
    else:
        def value(field):
            return int(getattr(field, "value", field))
        result.update(action=str(message.action_info.action_desc),
                      action_status=value(message.action_info.status),
                      fsm=value(message.fsm_state.current_state),
                      body=value(message.body_status))
    return result


class ControlTrace(AbstractContextManager):
    def __init__(self, backend, max_samples=20000):
        self.backend = backend
        self.samples = {kind: deque(maxlen=max_samples) for kind in TOPICS}
        self.dropped = {kind: 0 for kind in TOPICS}
        self.errors = []
        self.lock = threading.Lock()
        self.subscriptions = []
        self.active = False

    def _receive(self, kind, message):
        try:
            sample = trace_sample(kind, message)
        except Exception as exc:
            with self.lock:
                if len(self.errors) < 20:
                    self.errors.append(f"{kind}: {exc}")
            return
        with self.lock:
            if self.active:
                if len(self.samples[kind]) == self.samples[kind].maxlen:
                    self.dropped[kind] += 1
                self.samples[kind].append(sample)

    def __enter__(self):
        self.active = True
        try:
            for kind, (topic, message_type) in TOPICS.items():
                qos = self.backend._state_qos if kind == "mc" else self.backend._qos
                self.subscriptions.append(self.backend._node.create_subscription(
                    self.backend._types[message_type], topic,
                    lambda msg, selected=kind: self._receive(selected, msg), qos,
                ))
        except Exception:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, *args):
        with self.lock:
            self.active = False
        for subscription in self.subscriptions:
            self.backend._node.destroy_subscription(subscription)
        self.subscriptions.clear()

    def snapshot(self):
        with self.lock:
            return {"kind": "control_path_trace", "clock": "PC2 receive monotonic; source ROS stamp retained",
                    "samples": {kind: list(samples) for kind, samples in self.samples.items()},
                    "dropped": dict(self.dropped), "errors": list(self.errors)}
