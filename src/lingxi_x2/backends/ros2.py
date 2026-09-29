from __future__ import annotations

from collections.abc import Mapping
from contextlib import contextmanager
from io import BytesIO
import os
import math
import threading
import time
from typing import Any

from PIL import Image as PillowImage

from ..config import PlatformConfig
from ..errors import BackendUnavailableError, CapabilityUnavailableError, DataTimeoutError, SafetyInterlockError
from ..models import (
    ARM_JOINT_NAMES,
    HAND_JOINT_SUFFIXES,
    ArmState,
    CameraFrame,
    CameraName,
    Capability,
    ControlPreflight,
    HandState,
    JointSample,
    McState,
    PlatformStatus,
    PreflightCheck,
    Side,
    TactileFrame,
    TactileSurface,
    Timestamp,
)
from .base import Backend
from .ros_environment import prepare_current_process


CAMERA_TOPICS: dict[CameraName, tuple[str, str]] = {
    CameraName.RGBD_FRONT_RGB: ("/aima/hal/sensor/rgbd_head_front/rgb_image/compressed", "compressed"),
    CameraName.RGBD_FRONT_DEPTH: ("/aima/hal/sensor/rgbd_head_front/depth_image", "raw"),
    CameraName.STEREO_FRONT_LEFT: ("/aima/hal/sensor/stereo_head_front_left/rgb_image/compressed", "compressed"),
    CameraName.STEREO_FRONT_RIGHT: ("/aima/hal/sensor/stereo_head_front_right/rgb_image/compressed", "compressed"),
    CameraName.HEAD_REAR: ("/aima/hal/sensor/rgb_head_rear/rgb_image/compressed", "compressed"),
    CameraName.HEAD_FRONT_CENTER: ("/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed", "compressed"),
}

UPPER_BODY_TOPIC = "/mc/upper_body_command"
MC_STATE_TOPIC = "/aima/mc/common/state"
UPPER_BODY_MODE = "UPPERBODY_REMOTE_SPLIT"
HAND_DEXTEROUS_JOINT = 2
MC_ACTION_RUNNING = 100
MC_MOTION_IDLE = 0
# v1.1.4 MC publishes 4 for stable STAND_DEFAULT. Its installed .msg labels 4
# as SAFE, while the official MC-state page labels wire value 4 as STABLE.
MC_FSM_STABLE = 4
MC_BODY_STAND = 1

SET_MC_ACTION_CODES = {
    0: "success",
    2: "InvalidRequest",
    3: "UnknownAction",
    4: "InvalidPosture",
    5: "InRecovery",
    6: "SecureLevelForbid",
    7: "Starting",
    8: "MovingBusy",
    9: "MotionBusy",
    10: "NoTransitionPath",
    100: "PositionControlPrepare",
    101: "ForceControlPrepare",
    102: "Walking",
    103: "Offroad",
    104: "VrTeleop",
    105: "LieUp",
    106: "ProneUp",
    107: "SitDown",
    108: "SitUp",
    109: "DataCollection",
    110: "WholeBodyTeleop",
}


def populate_upper_body_message(
    message: Any,
    stamp: Any,
    sequence: int,
    arm: tuple[float, ...],
    hands: Mapping[Side, tuple[float, ...]],
) -> Any:
    """Populate the official v1.1 UpperBodyCommandArray joint layout."""
    if len(arm) != 14:
        raise SafetyInterlockError(f"Upper-body arm target must contain 14 values, got {len(arm)}")
    missing = [side.value for side in Side if side not in hands]
    if missing:
        raise SafetyInterlockError(f"Upper-body hand target is missing: {', '.join(missing)}")
    for side in Side:
        if len(hands[side]) != 10:
            raise SafetyInterlockError(
                f"Upper-body {side.value} hand target must contain 10 values, got {len(hands[side])}"
            )
    message.header.stamp = stamp
    message.header.frame_id = "mc_upper_body"
    message.header.sequence = sequence
    message.source = "lingxi_x2"
    message.hand_sub_mode = HAND_DEXTEROUS_JOINT
    message.head_pos = [0.0, 0.0]
    message.arm_pos = list(arm)
    # Official v1.1 layout: left HandCommand[0:10], then right HandCommand[0:10].
    message.hand_pos = list(hands[Side.LEFT]) + list(hands[Side.RIGHT])
    return message


def _enum_value(container: Any, default: int = 0) -> int:
    if container is None:
        return default
    return int(getattr(container, "value", container))


def validate_set_mc_action_response(response: Any, mode: str) -> None:
    result = getattr(response, "response", None)
    code = int(getattr(getattr(result, "header", None), "code", -1))
    status = _enum_value(getattr(result, "status", None), -1)
    if code == 0 and status == 1:
        return
    reason = SET_MC_ACTION_CODES.get(code, "unknown")
    message = str(getattr(result, "message", ""))
    raise SafetyInterlockError(
        f"SetMcAction({mode}) was rejected: code={code} ({reason}), status={status}, message={message!r}"
    )


class Ros2Backend(Backend):
    """Adapter for the ROS 2 interfaces installed on an X2 development computer."""

    def __init__(self, config: PlatformConfig):
        self.config = config
        prepare_current_process(config.ros.aimdk_prefix)
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CompressedImage, Image
            from aimdk_msgs.msg import (
                HandCommand,
                HandCommandArray,
                HandStateArray,
                JointCommand,
                JointCommandArray,
                JointStateArray,
            )
        except (ImportError, OSError) as exc:
            raise BackendUnavailableError(
                "ROS 2/AimDK runtime not available. Run on PC2 with the matching firmware message package."
            ) from exc

        self._rclpy = rclpy
        self._executor_type = SingleThreadedExecutor
        self._types = {
            "CompressedImage": CompressedImage,
            "Image": Image,
            "HandCommand": HandCommand,
            "HandCommandArray": HandCommandArray,
            "HandStateArray": HandStateArray,
            "JointCommand": JointCommand,
            "JointCommandArray": JointCommandArray,
            "JointStateArray": JointStateArray,
        }
        try:
            from aimdk_msgs.msg import UpperBodyCommandArray

            self._types["UpperBodyCommandArray"] = UpperBodyCommandArray
        except ImportError:
            pass
        try:
            from aimdk_msgs.msg import McCommonState

            self._types["McCommonState"] = McCommonState
        except ImportError:
            pass
        try:
            from aimdk_msgs.srv import GetMcAction, SetMcAction

            self._types["GetMcAction"] = GetMcAction
            self._types["SetMcAction"] = SetMcAction
        except ImportError:
            pass

        self._qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._condition = threading.Condition()
        self._arm: ArmState | None = None
        self._hands: dict[Side, HandState] = {}
        self._tactile: dict[Side, TactileFrame] = {}
        self._cameras: dict[CameraName, CameraFrame] = {}
        self._mc_state: McState | None = None
        self._camera_generation: dict[CameraName, int] = {}
        self._sample_seen = {"arm": False, "hand": False, "camera": False, "tactile": False, "mc": False}
        self._started = False
        self._thread: threading.Thread | None = None
        self._executor: Any = None
        self._node: Any = None
        self._arm_publisher: Any = None
        self._hand_publisher: Any = None
        self._upper_publisher: Any = None
        self._upper_sequence = 0
        self._stream_owner: int | None = None
        self._stream_subsystem: str | None = None
        self._stream_lock = threading.Lock()
        self._held_arm: tuple[float, ...] = ()
        self._held_hands: dict[Side, tuple[float, ...]] = {}
        self._graph_guard: tuple[int, str | None] = (0, "not checked")
        self._echo_subscription: Any = None
        self._last_command_echo: dict[str, Any] | None = None
        self._last_upper_command: dict[str, Any] | None = None
        self._subscriptions: list[Any] = []

    def start(self) -> None:
        if self._started:
            return
        if not self._rclpy.ok():
            self._rclpy.init(args=None)
        from rclpy.node import Node

        self._node = Node(f"{self.config.ros.node_name}_{os.getpid()}")
        self._executor = self._executor_type()
        self._executor.add_node(self._node)
        self._subscriptions.append(
            self._node.create_subscription(
                self._types["JointStateArray"],
                "/aima/hal/joint/arm/state",
                self._on_arm,
                self._qos,
            )
        )
        if "McCommonState" in self._types:
            self._subscriptions.append(
                self._node.create_subscription(
                    self._types["McCommonState"],
                    MC_STATE_TOPIC,
                    self._on_mc_state,
                    self._state_qos,
                )
            )
        self._subscriptions.append(
            self._node.create_subscription(
                self._types["HandStateArray"],
                "/aima/hal/joint/hand/state",
                self._on_hand,
                self._qos,
            )
        )
        for camera_text in self.config.ros.camera_streams:
            try:
                camera = CameraName(camera_text)
            except ValueError as exc:
                raise BackendUnavailableError(f"Unknown configured camera stream: {camera_text}") from exc
            topic, kind = CAMERA_TOPICS[camera]
            message_type = self._types["CompressedImage" if kind == "compressed" else "Image"]
            callback = lambda message, selected=camera: self._on_camera(selected, message)
            self._subscriptions.append(self._node.create_subscription(message_type, topic, callback, self._qos))
            self._camera_generation[camera] = 0
        self._started = True
        self._thread = threading.Thread(target=self._spin, name="lingxi-x2-ros", daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while self._started and self._rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def close(self) -> None:
        self._started = False
        if self._executor is not None:
            self._executor.wake()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._executor is not None and self._node is not None:
            self._executor.remove_node(self._node)
        if self._node is not None:
            self._node.destroy_node()
        self._node = None
        self._executor = None

    @staticmethod
    def _stamp(header: Any) -> Timestamp:
        stamp = header.stamp
        return Timestamp(int(stamp.sec), int(stamp.nanosec), time.monotonic_ns(), "ros")

    @staticmethod
    def _joint(item: Any, fallback: str) -> JointSample:
        return JointSample(
            str(getattr(item, "name", "") or fallback),
            float(item.position),
            float(getattr(item, "velocity", 0.0)),
            float(getattr(item, "effort", 0.0)),
            int(getattr(item, "faultcode", getattr(item, "error_code", 0))),
            int(getattr(item, "state", 0)),
        )

    def _on_arm(self, message: Any) -> None:
        joints = tuple(
            self._joint(item, ARM_JOINT_NAMES[index] if index < 14 else f"arm_{index}")
            for index, item in enumerate(message.joints)
        )
        state_value = getattr(getattr(message, "state", None), "value", None)
        state = ArmState(self._stamp(message.header), joints, state_value)
        with self._condition:
            self._arm = state
            self._sample_seen["arm"] = True
            self._condition.notify_all()

    @staticmethod
    def _value(container: Any, default: int = 0) -> int:
        return _enum_value(container, default)

    def _on_mc_state(self, message: Any) -> None:
        input_source = getattr(message, "input_source", None)
        action_info = getattr(message, "action_info", None)
        motion_status = getattr(message, "motion_status", None)
        runtime_model = getattr(message, "runtime_model", None)
        state = McState(
            self._stamp(message.header),
            str(getattr(input_source, "name", "")),
            int(getattr(input_source, "priority", 0)),
            int(getattr(input_source, "timeout", 0)),
            str(getattr(action_info, "action_desc", "")),
            self._value(getattr(action_info, "status", None)),
            self._value(getattr(motion_status, "player_state", None)),
            self._value(getattr(motion_status, "control_area", None)),
            str(getattr(motion_status, "motion", "")),
            self._value(getattr(motion_status, "type", None)),
            self._value(getattr(getattr(message, "fsm_state", None), "current_state", None)),
            self._value(getattr(message, "body_status", None)),
            self._value(getattr(runtime_model, "left_hand_status", None)),
            self._value(getattr(runtime_model, "right_hand_status", None)),
        )
        with self._condition:
            self._mc_state = state
            self._sample_seen["mc"] = True
            self._condition.notify_all()

    def _on_hand(self, message: Any) -> None:
        stamp = self._stamp(message.header)
        states: dict[Side, HandState] = {}
        for side in Side:
            prefix = "L" if side is Side.LEFT else "R"
            items = getattr(message, f"{side.value}_hands")
            hand_type = int(getattr(message, f"{side.value}_hand_type").value)
            joints = tuple(
                self._joint(item, f"{prefix}_{HAND_JOINT_SUFFIXES[index] if index < 10 else index}")
                for index, item in enumerate(items)
            )
            states[side] = HandState(stamp, side, hand_type, joints)
        tactile = self._extract_tactile(message, stamp)
        with self._condition:
            self._hands = states
            self._sample_seen["hand"] = True
            if tactile:
                self._tactile = tactile
                self._sample_seen["tactile"] = True
            self._condition.notify_all()

    @staticmethod
    def _surface(sensor: Any, field: str, name: str, shape: tuple[int, ...], count: int | None = None) -> TactileSurface:
        values = tuple(int(value) for value in getattr(sensor, field))
        if count is not None:
            values = values[:count]
        return TactileSurface(name, shape, values)

    def _extract_tactile(self, message: Any, stamp: Timestamp) -> dict[Side, TactileFrame]:
        result: dict[Side, TactileFrame] = {}
        for side in Side:
            field = f"{side.value}_touch_sensors"
            if not hasattr(message, field):
                return {}
            sensor = getattr(message, field)
            fingertips = {
                name: self._surface(sensor, source, name, (4, 4))
                for name, source in (
                    ("thumb", "thumb_touch_data"),
                    ("index", "index_finger_touch_data"),
                    ("middle", "middle_finger_touch_data"),
                    ("ring", "ring_finger_touch_data"),
                    ("little", "little_finger_touch_data"),
                )
            }
            result[side] = TactileFrame(
                stamp,
                side,
                self._surface(sensor, "palm_touch_data", "palm", (5, 5), 25),
                self._surface(sensor, "back_of_hand_touch_data", "back_of_hand", (6, 6)),
                fingertips,
            )
        return result

    def _on_camera(self, camera: CameraName, message: Any) -> None:
        topic, kind = CAMERA_TOPICS[camera]
        data = bytes(message.data)
        if kind == "compressed":
            try:
                with PillowImage.open(BytesIO(data)) as image:
                    width, height = image.size
            except Exception:
                width, height = 0, 0
            encoding = str(getattr(message, "format", "jpeg") or "jpeg")
            compressed = True
        else:
            width, height = int(message.width), int(message.height)
            encoding = str(message.encoding)
            compressed = False
        header = message.header
        frame = CameraFrame(
            self._stamp(header),
            camera,
            width,
            height,
            encoding,
            data,
            str(header.frame_id),
            compressed,
            topic,
        )
        with self._condition:
            self._cameras[camera] = frame
            self._camera_generation[camera] = self._camera_generation.get(camera, 0) + 1
            self._sample_seen["camera"] = True
            self._condition.notify_all()

    def _wait(self, predicate: Any, timeout_s: float, label: str) -> None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise DataTimeoutError(f"Timed out waiting for {label} after {timeout_s:.2f}s")
                self._condition.wait(remaining)

    def arm_state(self, timeout_s: float = 1.0) -> ArmState:
        self._wait(lambda: self._arm is not None, timeout_s, "arm state")
        assert self._arm is not None
        if time.monotonic_ns() - self._arm.timestamp.monotonic_ns > self.config.ros.state_timeout_s * 1e9:
            raise DataTimeoutError("Arm state is stale")
        return self._arm

    def hand_states(self, timeout_s: float = 1.0) -> Mapping[Side, HandState]:
        self._wait(lambda: len(self._hands) == 2, timeout_s, "hand state")
        if any(
            time.monotonic_ns() - state.timestamp.monotonic_ns > self.config.ros.state_timeout_s * 1e9
            for state in self._hands.values()
        ):
            raise DataTimeoutError("Hand state is stale")
        return dict(self._hands)

    def tactile_frames(self, timeout_s: float = 1.0) -> Mapping[Side, TactileFrame]:
        if not self._message_supports_tactile():
            firmware = self.config.robot.firmware or "the installed firmware"
            raise CapabilityUnavailableError(
                f"Installed HandStateArray has no tactile fields. {firmware} does not export pressure data through this schema."
            )
        self._wait(lambda: len(self._tactile) == 2, timeout_s, "tactile data")
        with self._condition:
            frames = dict(self._tactile)
        if any(time.monotonic_ns() - frame.timestamp.monotonic_ns > self.config.ros.state_timeout_s * 1e9
               for frame in frames.values()):
            raise DataTimeoutError("Tactile data is stale")
        return frames

    def camera_frame(self, camera: CameraName, timeout_s: float = 2.0) -> CameraFrame:
        if camera not in self._camera_generation:
            raise CapabilityUnavailableError(
                f"Camera {camera.value} is not subscribed; add it to ros.camera_streams before startup"
            )
        previous = self._camera_generation[camera]
        self._wait(lambda: self._camera_generation[camera] > previous, timeout_s, f"camera {camera.value}")
        return self._cameras[camera]

    def mc_state(self, timeout_s: float = 1.0) -> McState:
        if "McCommonState" not in self._types:
            raise CapabilityUnavailableError("Installed aimdk_msgs has no McCommonState message")
        self._wait(lambda: self._mc_state is not None, timeout_s, "MC common state")
        assert self._mc_state is not None
        if time.monotonic_ns() - self._mc_state.timestamp.monotonic_ns > self.config.ros.state_timeout_s * 1e9:
            raise DataTimeoutError("MC common state is stale")
        return self._mc_state

    def _message_supports_tactile(self) -> bool:
        try:
            return hasattr(self._types["HandStateArray"](), "left_touch_sensors")
        except Exception:
            return False

    def _graph_topics(self) -> dict[str, list[str]]:
        if self._node is None:
            return {}
        return dict(self._node.get_topic_names_and_types())

    def _wait_for_topic(self, topic: str, timeout_s: float = 1.0) -> bool:
        """Allow DDS graph discovery to settle before capability/preflight checks."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if topic in self._graph_topics():
                return True
            time.sleep(0.05)
        return topic in self._graph_topics()

    def status(self) -> PlatformStatus:
        if self._started and "UpperBodyCommandArray" in self._types:
            self._wait_for_topic("/mc/upper_body_command")
        topics = self._graph_topics()
        tactile_supported = self._message_supports_tactile()
        upper_supported = "UpperBodyCommandArray" in self._types and "/mc/upper_body_command" in topics
        camera_topics = [CAMERA_TOPICS[camera][0] for camera in self._camera_generation]
        capabilities = {
            "arm_state": Capability(
                "/aima/hal/joint/arm/state" in topics,
                self._sample_seen["arm"],
                "Observed a live sample" if self._sample_seen["arm"] else "Topic discovery only",
                "/aima/hal/joint/arm/state",
            ),
            "arm_control": Capability(
                True,
                False,
                (
                    "Upper-body MC interface discovered; command motion is not yet validated"
                    if upper_supported
                    else "HAL is implemented but requires MC stopped; upper-body MC is unavailable"
                ),
                "/mc/upper_body_command" if upper_supported else "/aima/hal/joint/arm/command",
            ),
            "hand_state": Capability(
                "/aima/hal/joint/hand/state" in topics,
                self._sample_seen["hand"],
                "Observed a live sample" if self._sample_seen["hand"] else "Topic discovery only",
                "/aima/hal/joint/hand/state",
            ),
            "hand_control": Capability(
                True,
                False,
                (
                    "Upper-body MC interface discovered; command motion is not yet validated"
                    if upper_supported
                    else "HAL is implemented but official documentation requires MC stopped"
                ),
                "/mc/upper_body_command" if upper_supported else "/aima/hal/joint/hand/command",
            ),
            "camera": Capability(
                any(topic in topics for topic in camera_topics),
                self._sample_seen["camera"],
                "Subscribed streams: " + ", ".join(camera.value for camera in self._camera_generation),
                ",".join(camera_topics),
            ),
            "tactile": Capability(
                tactile_supported,
                self._sample_seen["tactile"],
                "Raw uint8 arrays; no physical pressure unit is defined by AimDK"
                if tactile_supported
                else "Installed HandStateArray lacks left_touch_sensors/right_touch_sensors",
                "/aima/hal/joint/hand/state",
            ),
            "mc_state": Capability(
                "McCommonState" in self._types and MC_STATE_TOPIC in topics,
                self._sample_seen["mc"],
                "Mode, action, FSM, body state, and continuous-command input source"
                if self._sample_seen["mc"]
                else "Topic discovery only",
                MC_STATE_TOPIC,
            ),
        }
        warnings: list[str] = []
        if not tactile_supported:
            warnings.append("Tactile is unavailable in the installed HandStateArray schema.")
        if not upper_supported:
            warnings.append("The /mc/upper_body_command interface is not present; HAL requires stopping PC1 MC.")
        return PlatformStatus(
            "ros2",
            self._started,
            os.environ.get("AGIBOT_SOFTWARE_VERSION"),
            os.environ.get("AGIBOT_SKU"),
            capabilities,
            tuple(warnings),
        )

    def _competing_publishers(self, topic: str, *, allow_configured: bool = True) -> list[str]:
        assert self._node is not None
        own_name = self._node.get_fully_qualified_name()
        allowed = set(self.config.control.allowed_competing_nodes) if allow_configured else set()
        names: list[str] = []
        for endpoint in self._node.get_publishers_info_by_topic(topic):
            namespace = str(endpoint.node_namespace).rstrip("/")
            name = f"{namespace}/{endpoint.node_name}" if namespace else f"/{endpoint.node_name}"
            if name != own_name and name not in allowed and str(endpoint.node_name) not in allowed:
                names.append(name)
        return sorted(set(names))

    def control_preflight(self, subsystem: str, *, require_enabled: bool = True) -> ControlPreflight:
        if subsystem not in {"arm", "hand"}:
            raise ValueError(f"Unknown subsystem: {subsystem}")
        control = self.config.control
        checks: list[PreflightCheck] = [
            PreflightCheck(
                "control_enabled",
                control.enabled,
                f"control.enabled={str(control.enabled).lower()}",
            ),
            PreflightCheck(
                "authority",
                control.authority == "upper_body_mc",
                f"control.authority={control.authority}",
            ),
        ]
        mc_state: McState | None = None
        if control.authority == "upper_body_mc":
            interface_available = "UpperBodyCommandArray" in self._types and self._wait_for_topic(UPPER_BODY_TOPIC)
            checks.append(
                PreflightCheck("upper_body_interface", interface_available, f"{UPPER_BODY_TOPIC} discovered")
            )
            competing = (
                self._competing_publishers(UPPER_BODY_TOPIC, allow_configured=False)
                if interface_available
                else []
            )
            checks.append(
                PreflightCheck(
                    "exclusive_publisher",
                    not competing,
                    "no competing publishers" if not competing else f"competing publishers: {', '.join(competing)}",
                )
            )
            try:
                mc_state = self.mc_state()
            except (CapabilityUnavailableError, DataTimeoutError) as exc:
                checks.append(PreflightCheck("mc_state_fresh", False, str(exc)))
            else:
                checks.extend(
                    (
                        PreflightCheck("mc_state_fresh", True, MC_STATE_TOPIC),
                        PreflightCheck(
                            "mc_mode",
                            mc_state.action == UPPER_BODY_MODE,
                            f"action={mc_state.action or '<empty>'}",
                        ),
                        PreflightCheck(
                            "mc_action_running",
                            mc_state.action_status == MC_ACTION_RUNNING,
                            f"action_status={mc_state.action_status}",
                        ),
                        PreflightCheck(
                            "mc_stable",
                            mc_state.fsm_state == MC_FSM_STABLE,
                            f"fsm_state={mc_state.fsm_state}",
                        ),
                        PreflightCheck(
                            "body_standing",
                            mc_state.body_state == MC_BODY_STAND,
                            f"body_state={mc_state.body_state}",
                        ),
                        PreflightCheck(
                            "motion_idle",
                            mc_state.motion_player_state == MC_MOTION_IDLE,
                            f"motion_player_state={mc_state.motion_player_state}, motion={mc_state.motion or '<empty>'}",
                        ),
                    )
                )
            try:
                arm = self.arm_state()
                hands = self.hand_states()
            except DataTimeoutError as exc:
                checks.append(PreflightCheck("joint_feedback", False, str(exc)))
            else:
                joint_faults = [
                    joint.name
                    for joint in (*arm.joints, *(joint for side in Side for joint in hands[side].joints))
                    if joint.fault_code not in (None, 0)
                ]
                checks.append(
                    PreflightCheck(
                        "joint_feedback",
                        len(arm.joints) == 14 and all(len(hands[side].joints) == 10 for side in Side),
                        f"arm={len(arm.joints)}, left_hand={len(hands[Side.LEFT].joints)}, right_hand={len(hands[Side.RIGHT].joints)}",
                    )
                )
                checks.append(
                    PreflightCheck(
                        "arm_domain_ready",
                        arm.domain_state not in {1, 2, 3, 4},
                        f"domain_state={arm.domain_state}",
                    )
                )
                checks.append(
                    PreflightCheck(
                        "joint_faults_clear",
                        not joint_faults,
                        "all reported fault codes are zero"
                        if not joint_faults
                        else f"nonzero fault codes: {', '.join(joint_faults)}",
                    )
                )
            report = ControlPreflight(subsystem, UPPER_BODY_TOPIC, time.monotonic_ns(), tuple(checks), mc_state)
            failures = [check for check in checks if not check.passed]
            if require_enabled and failures:
                detail = "; ".join(f"{check.name}: {check.detail}" for check in failures)
                raise SafetyInterlockError(f"Control preflight failed: {detail}")
            return report
        if control.authority != "hal_mc_stopped":
            report = ControlPreflight(subsystem, "", time.monotonic_ns(), tuple(checks))
            if require_enabled:
                detail = "; ".join(f"{check.name}: {check.detail}" for check in checks if not check.passed)
                raise SafetyInterlockError(f"Control preflight failed: {detail}")
            return report
        topic = f"/aima/hal/joint/{subsystem}/command"
        competitors = self._competing_publishers(topic) if control.require_no_competing_publishers else []
        checks.append(
            PreflightCheck(
                "exclusive_publisher",
                not competitors,
                "no competing publishers" if not competitors else f"competing publishers: {', '.join(competitors)}",
            )
        )
        checks[1] = PreflightCheck("authority", True, "control.authority=hal_mc_stopped")
        report = ControlPreflight(subsystem, topic, time.monotonic_ns(), tuple(checks))
        failures = [check for check in checks if not check.passed]
        if require_enabled and failures:
            detail = "; ".join(f"{check.name}: {check.detail}" for check in failures)
            raise SafetyInterlockError(f"Control preflight failed: {detail}")
        return report

    def _check_stream_graph(self) -> None:
        """Runs on the ROS executor, never in the publication loop."""
        try:
            competing = self._competing_publishers(UPPER_BODY_TOPIC, allow_configured=False)
            matched = self._upper_publisher.get_subscription_count() > self._local_upper_subscriber_count()
            error = f"competing publishers: {competing}" if competing else (
                None if matched else "upper-body subscriber lost"
            )
        except Exception as exc:
            error = f"graph check failed: {exc}"
        with self._condition:
            self._graph_guard = (time.monotonic_ns(), error)

    def _local_upper_subscriber_count(self) -> int:
        own = self._node.get_fully_qualified_name()
        return sum(
            f"{str(endpoint.node_namespace).rstrip('/')}/{endpoint.node_name}" == own
            for endpoint in self._node.get_subscriptions_info_by_topic(UPPER_BODY_TOPIC)
        )

    @contextmanager
    def command_stream(self, subsystem: str):
        if not self._stream_lock.acquire(blocking=False):
            raise SafetyInterlockError("Another command stream is active")
        timer = None
        try:
            self.control_preflight(subsystem)
            if self.config.control.authority == "upper_body_mc":
                with self._condition:
                    self._last_command_echo = None
                    self._last_upper_command = None
                if self.config.control.command_echo and self._echo_subscription is None:
                    self._echo_subscription = self._node.create_subscription(
                        self._types["UpperBodyCommandArray"], UPPER_BODY_TOPIC,
                        self._on_command_echo, self._qos,
                    )
                    self._subscriptions.append(self._echo_subscription)
                arm, hands = self.arm_state(), self.hand_states()
                self._held_arm = tuple(j.position_rad for j in arm.joints)
                self._held_hands = {side: tuple(j.position_rad for j in hands[side].joints) for side in Side}
                if self._upper_publisher is None:
                    self._upper_publisher = self._node.create_publisher(
                        self._types["UpperBodyCommandArray"], UPPER_BODY_TOPIC, self._qos
                    )
                deadline = time.monotonic() + 2.0
                while self._upper_publisher.get_subscription_count() <= self._local_upper_subscriber_count():
                    if time.monotonic() >= deadline:
                        raise SafetyInterlockError("No matched upper-body subscriber")
                    time.sleep(0.02)
                self._check_stream_graph()
                timer = self._node.create_timer(0.1, self._check_stream_graph)
            self._stream_subsystem = subsystem
            self._stream_owner = threading.get_ident()
            yield
        finally:
            self._stream_owner = None
            self._stream_subsystem = None
            if timer is not None:
                self._node.destroy_timer(timer)
            self._stream_lock.release()

    @staticmethod
    def _command_fields(message: Any) -> dict[str, Any]:
        return {
            "sequence": int(message.header.sequence),
            "stamp_sec": int(message.header.stamp.sec),
            "stamp_nanosec": int(message.header.stamp.nanosec),
            "frame_id": str(message.header.frame_id),
            "source": str(message.source),
            "hand_sub_mode": int(message.hand_sub_mode),
            "head_pos": list(message.head_pos),
            "arm_pos": list(message.arm_pos),
            "hand_pos": list(message.hand_pos),
        }

    def _on_command_echo(self, message: Any) -> None:
        fields = self._command_fields(message)
        with self._condition:
            # Compare full stamp and sequence, excluding old streams and other nodes.
            if fields == self._last_upper_command:
                self._last_command_echo = {"received_monotonic_ns": time.monotonic_ns(), "command": fields}

    def stream_diagnostics(self) -> dict[str, Any]:
        with self._condition:
            command, echo = self._last_upper_command, self._last_command_echo
        return {
            "last_command": command,
            "local_echo_enabled": self.config.control.command_echo,
            "last_local_echo": echo,
            "last_command_echo_matches": (echo is not None and echo["command"] == command)
            if self.config.control.command_echo else None,
            "echo_evidence": "local DDS readback only; not proof of MC consumption",
        }

    def _check_stream_state(self, subsystem: str) -> None:
        if self._stream_owner != threading.get_ident() or self._stream_subsystem != subsystem:
            raise SafetyInterlockError("Publishing requires an active command_stream")
        if not self.config.control.enabled:
            raise SafetyInterlockError("Hardware writes are disabled in config")
        if self.config.control.authority != "upper_body_mc":
            # Preserve the existing HAL safety path; only MC streaming is optimized.
            self.control_preflight(subsystem)
            return
        with self._condition:
            arm, hands, mc = self._arm, dict(self._hands), self._mc_state
            graph_time, graph_error = self._graph_guard
        now = time.monotonic_ns()
        if graph_error or now - graph_time > 0.5e9:
            raise SafetyInterlockError(f"Stream graph guard unavailable: {graph_error or 'stale'}")
        states = [arm, mc, *(hands.get(side) for side in Side)]
        if any(state is None or now - state.timestamp.monotonic_ns > self.config.ros.state_timeout_s * 1e9 for state in states):
            raise SafetyInterlockError("Missing or stale stream safety feedback")
        if (mc.action != UPPER_BODY_MODE or mc.action_status != MC_ACTION_RUNNING
                or mc.fsm_state != MC_FSM_STABLE or mc.body_state != MC_BODY_STAND
                or mc.motion_player_state != MC_MOTION_IDLE):
            raise SafetyInterlockError("MC left stable standing upper-body mode")
        if (len(arm.joints) != 14 or arm.domain_state in {1, 2, 3, 4}
                or any(len(hands[side].joints) != 10 or hands[side].hand_type != 1 for side in Side)):
            raise SafetyInterlockError("Invalid arm/hand feedback or domain state")
        joints = (*arm.joints, *(joint for side in Side for joint in hands[side].joints))
        if any(j.fault_code not in (None, 0) or not math.isfinite(j.position_rad) for j in joints):
            raise SafetyInterlockError("Joint fault or non-finite stream feedback")

    def _publish_upper(self, arm: tuple[float, ...], hands: Mapping[Side, tuple[float, ...]]) -> dict[str, Any]:
        assert self._node is not None
        message_type = self._types["UpperBodyCommandArray"]
        if self._upper_publisher is None:
            self._upper_publisher = self._node.create_publisher(message_type, UPPER_BODY_TOPIC, self._qos)
        message = message_type()
        populate_upper_body_message(
            message,
            self._node.get_clock().now().to_msg(),
            self._upper_sequence,
            arm,
            hands,
        )
        with self._condition:
            self._last_upper_command = self._command_fields(message)
        started_ns = time.monotonic_ns()
        self._upper_publisher.publish(message)
        finished_ns = time.monotonic_ns()
        self._upper_sequence = (self._upper_sequence + 1) % (2**32)
        return {
            "monotonic_ns": started_ns,
            "publish_return_monotonic_ns": finished_ns,
            "sequence": int(message.header.sequence),
            "measurement": "rclpy_publish_call",
        }

    def publish_arm(
        self,
        positions_rad: tuple[float, ...],
        velocities_rad_s: tuple[float, ...],
        efforts_nm: tuple[float, ...],
        stiffness: float,
        damping: float,
    ) -> dict[str, Any] | None:
        self._check_stream_state("arm")
        assert self._node is not None
        if self.config.control.authority == "upper_body_mc":
            return self._publish_upper(positions_rad, self._held_hands)
        array_type = self._types["JointCommandArray"]
        command_type = self._types["JointCommand"]
        if self._arm_publisher is None:
            self._arm_publisher = self._node.create_publisher(
                array_type, "/aima/hal/joint/arm/command", self._qos
            )
        message = array_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = "arm_command"
        message.joints = []
        for name, position, velocity, effort in zip(
            ARM_JOINT_NAMES, positions_rad, velocities_rad_s, efforts_nm, strict=True
        ):
            command = command_type()
            command.name = name
            command.position = position
            command.velocity = velocity
            command.effort = effort
            command.stiffness = stiffness
            command.damping = damping
            message.joints.append(command)
        self._arm_publisher.publish(message)

    def publish_hand(self, side: Side, positions_rad: tuple[float, ...]) -> dict[str, Any] | None:
        self._check_stream_state("hand")
        assert self._node is not None
        if self.config.control.authority == "upper_body_mc":
            values = dict(self._held_hands)
            values[side] = positions_rad
            return self._publish_upper(self._held_arm, values)
        array_type = self._types["HandCommandArray"]
        command_type = self._types["HandCommand"]
        if self._hand_publisher is None:
            self._hand_publisher = self._node.create_publisher(
                array_type, "/aima/hal/joint/hand/command", self._qos
            )
        message = array_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = "hand_command"
        message.left_hand_type.value = 1
        message.right_hand_type.value = 1
        prefix = "L" if side is Side.LEFT else "R"
        commands = []
        for suffix, position in zip(HAND_JOINT_SUFFIXES, positions_rad, strict=True):
            command = command_type()
            command.name = f"{prefix}_{suffix}"
            command.position = position
            command.velocity = 0.0
            command.acceleration = 0.0
            command.deceleration = 0.0
            command.effort = 0.0
            commands.append(command)
        if side is Side.LEFT:
            message.left_hands = commands
            message.right_hands = []
        else:
            message.left_hands = []
            message.right_hands = commands
        self._hand_publisher.publish(message)

    def set_motion_mode(self, mode: str) -> McState:
        if not self.config.control.enabled:
            raise SafetyInterlockError("Hardware writes are disabled in config (control.enabled=false)")
        if "SetMcAction" not in self._types:
            raise CapabilityUnavailableError("Installed aimdk_msgs has no SetMcAction service")
        assert self._node is not None
        service_type = self._types["SetMcAction"]
        client = self._node.create_client(service_type, "/aimdk_5Fmsgs/srv/SetMcAction")
        if not client.wait_for_service(timeout_sec=2.0):
            raise CapabilityUnavailableError("SetMcAction service is unavailable")
        request = service_type.Request()
        request.source = "lingxi_x2"
        request.command.action_desc = mode
        if hasattr(request, "header"):
            request.header.stamp = self._node.get_clock().now().to_msg()
        future = client.call_async(request)
        for _ in range(8):
            deadline = time.monotonic() + 0.25
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if future.done():
                break
            request.header.stamp = self._node.get_clock().now().to_msg()
            future = client.call_async(request)
        if not future.done() or future.result() is None:
            raise DataTimeoutError("SetMcAction timed out")
        response = future.result()
        validate_set_mc_action_response(response, mode)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            state = self.mc_state()
            if state.action == mode and state.action_status == MC_ACTION_RUNNING:
                return state
            time.sleep(0.05)
        state = self.mc_state()
        raise DataTimeoutError(
            f"SetMcAction({mode}) was accepted but MC did not reach RUNNING; "
            f"action={state.action}, status={state.action_status}"
        )
