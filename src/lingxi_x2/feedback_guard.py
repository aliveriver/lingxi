"""Common ROS publication gates. Passing is not encoder calibration or arrival."""
import math

from .errors import SafetyInterlockError
from .models import ARM_JOINT_NAMES, HAND_JOINT_SUFFIXES, Side
from .safety import validate_hand_target


def require_fresh(timestamp, *, now_ns, now_ros_ns, max_age_ns):
    source = timestamp.sec * 10**9 + timestamp.nanosec
    if (type(timestamp.sec) is not int or type(timestamp.nanosec) is not int
            or timestamp.sec < 0 or not 0 <= timestamp.nanosec < 10**9
            or type(timestamp.monotonic_ns) is not int or source <= 0
            or not 0 <= now_ns - timestamp.monotonic_ns <= max_age_ns
            or not 0 <= now_ros_ns - source <= max_age_ns):
        raise SafetyInterlockError("Missing, stale or future source/receive feedback timestamp")


def validate_joint_feedback(arm, hands):
    if arm is None or arm.domain_state != 0:
        raise SafetyInterlockError("Arm domain is not explicitly ready")
    if tuple(j.name for j in arm.joints) != ARM_JOINT_NAMES:
        raise SafetyInterlockError("Incomplete or reordered arm feedback")
    if set(hands) != set(Side):
        raise SafetyInterlockError("Both O10 hands are required; arm-only isolation is not verified")
    for side in Side:
        hand = hands[side]
        names = tuple(f'{side.value[0].upper()}_{name}' for name in HAND_JOINT_SUFFIXES)
        if hand.side != side or hand.hand_type != 1 or tuple(j.name for j in hand.joints) != names:
            raise SafetyInterlockError("Invalid hand side/type or incomplete/reordered hand feedback")
        validate_hand_target(tuple(j.position_rad for j in hand.joints))
    for joint in (*arm.joints, *(j for side in Side for j in hands[side].joints)):
        if joint.fault_code != 0 or not math.isfinite(joint.position_rad):
            raise SafetyInterlockError(f"Unknown/faulted or nonfinite joint feedback: {joint.name}")


def validate_feedback(arm, hands, mc, *, now_ns, now_ros_ns, max_age_ns, require_mc=True):
    validate_joint_feedback(arm, hands)
    if require_mc and mc is None:
        raise SafetyInterlockError("Missing MC feedback")
    for state in (arm, *hands.values(), *((mc,) if require_mc else ())):
        require_fresh(state.timestamp, now_ns=now_ns, now_ros_ns=now_ros_ns, max_age_ns=max_age_ns)
