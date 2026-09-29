from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Mapping, Sequence


ARM_JOINT_NAMES = tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in (
        "shoulder_pitch",
        "shoulder_roll",
        "shoulder_yaw",
        "elbow",
        "wrist_yaw",
        "wrist_pitch",
        "wrist_roll",
    )
)

HAND_JOINT_SUFFIXES = (
    "thumb_roll_joint",
    "thumb_abad_joint",
    "thumb_mcp_joint",
    "index_abad_joint",
    "index_pip_joint",
    "middle_pip_joint",
    "ring_abad_joint",
    "ring_pip_joint",
    "pinky_abad_joint",
    "pinky_pip_joint",
)


class ValueStrEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class Side(ValueStrEnum):
    LEFT = "left"
    RIGHT = "right"


class CameraName(ValueStrEnum):
    RGBD_FRONT_RGB = "rgbd_front_rgb"
    RGBD_FRONT_DEPTH = "rgbd_front_depth"
    STEREO_FRONT_LEFT = "stereo_front_left"
    STEREO_FRONT_RIGHT = "stereo_front_right"
    HEAD_REAR = "head_rear"
    HEAD_FRONT_CENTER = "head_front_center"


@dataclass(frozen=True, slots=True)
class Timestamp:
    sec: int
    nanosec: int
    monotonic_ns: int
    clock: str = "ros"

    @property
    def seconds(self) -> float:
        return self.sec + self.nanosec / 1_000_000_000


@dataclass(frozen=True, slots=True)
class JointSample:
    name: str
    position_rad: float
    velocity_rad_s: float
    effort_nm: float
    fault_code: int | None = None
    state: int | None = None


@dataclass(frozen=True, slots=True)
class ArmState:
    timestamp: Timestamp
    joints: tuple[JointSample, ...]
    domain_state: int | None = None
    source: str = "/aima/hal/joint/arm/state"


@dataclass(frozen=True, slots=True)
class HandState:
    timestamp: Timestamp
    side: Side
    hand_type: int
    joints: tuple[JointSample, ...]
    source: str = "/aima/hal/joint/hand/state"


@dataclass(frozen=True, slots=True)
class TactileSurface:
    name: str
    shape: tuple[int, ...]
    values: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class TactileFrame:
    timestamp: Timestamp
    side: Side
    palm: TactileSurface
    back_of_hand: TactileSurface
    fingertips: Mapping[str, TactileSurface]
    unit: str = "raw_uint8"
    source: str = "/aima/hal/joint/hand/state"


@dataclass(frozen=True, slots=True)
class CameraFrame:
    timestamp: Timestamp
    camera: CameraName
    width: int
    height: int
    encoding: str
    data: bytes
    frame_id: str = ""
    compressed: bool = False
    source: str = ""

    def metadata(self) -> dict[str, Any]:
        result = asdict(self)
        result.pop("data")
        result["size_bytes"] = len(self.data)
        return result


@dataclass(frozen=True, slots=True)
class ArmCommand:
    positions_rad: tuple[float, ...]
    duration_s: float = 1.0
    velocities_rad_s: tuple[float, ...] | None = None
    efforts_nm: tuple[float, ...] | None = None
    stiffness_nm_rad: float = 20.0
    damping_nm_s_rad: float = 2.0

    @classmethod
    def from_positions(cls, values: Sequence[float], duration_s: float = 1.0) -> "ArmCommand":
        return cls(tuple(float(value) for value in values), duration_s=float(duration_s))


@dataclass(frozen=True, slots=True)
class HandCommand:
    side: Side
    positions_rad: tuple[float, ...]
    duration_s: float = 0.5

    @classmethod
    def from_positions(
        cls, side: Side | str, values: Sequence[float], duration_s: float = 0.5
    ) -> "HandCommand":
        return cls(Side(side), tuple(float(value) for value in values), float(duration_s))


@dataclass(frozen=True, slots=True)
class Observation:
    captured_monotonic_ns: int
    arm: ArmState | None = None
    hands: Mapping[Side, HandState] = field(default_factory=dict)
    tactile: Mapping[Side, TactileFrame] = field(default_factory=dict)
    cameras: Mapping[CameraName, CameraFrame] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class Capability:
    available: bool
    verified: bool
    reason: str
    interface: str | None = None


@dataclass(frozen=True, slots=True)
class PlatformStatus:
    backend: str
    connected: bool
    firmware: str | None
    model: str | None
    capabilities: Mapping[str, Capability]
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class McState:
    """Safety-relevant fields from ``/aima/mc/common/state``."""

    timestamp: Timestamp
    input_source: str
    input_priority: int
    input_timeout_ms: int
    action: str
    action_status: int
    motion_player_state: int
    motion_control_area: int
    motion: str
    motion_type: int
    fsm_state: int
    body_state: int
    left_hand_type: int
    right_hand_type: int
    source: str = "/aima/mc/common/state"


@dataclass(frozen=True, slots=True)
class PreflightCheck:
    name: str
    passed: bool
    detail: str


@dataclass(frozen=True, slots=True)
class ControlPreflight:
    subsystem: str
    command_topic: str
    checked_monotonic_ns: int
    checks: tuple[PreflightCheck, ...]
    mc_state: McState | None = None

    @property
    def ready(self) -> bool:
        return all(check.passed for check in self.checks)

    def as_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["ready"] = self.ready
        return result
