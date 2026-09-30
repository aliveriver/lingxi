from __future__ import annotations

from collections.abc import Mapping
from io import BytesIO
import math
import threading
import time

from PIL import Image, ImageDraw

from ..config import PlatformConfig
from ..errors import DataTimeoutError, SafetyInterlockError
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


class MockBackend(Backend):
    """Deterministic simulator for policy, UI, and data-pipeline development."""

    def __init__(self, config: PlatformConfig | None = None, *, initial_arm_positions=None) -> None:
        self.config = config or PlatformConfig(backend="mock")
        self._lock = threading.RLock()
        self._started = False
        from ..safety import validate_arm_target
        self._arm = list(validate_arm_target(initial_arm_positions)) if initial_arm_positions is not None else [0.0] * 14
        self._hands = {Side.LEFT: [0.0] * 10, Side.RIGHT: [0.0] * 10}
        self._frame_index = 0
        self._motion_mode = "UPPERBODY_REMOTE_SPLIT"
        self._fsm_state = 4
        self._body_state = 1

    @staticmethod
    def _timestamp() -> Timestamp:
        now_ns = time.time_ns()
        return Timestamp(now_ns // 1_000_000_000, now_ns % 1_000_000_000, time.monotonic_ns(), "system")

    def start(self) -> None:
        self._started = True

    def close(self) -> None:
        self._started = False

    def _require_started(self) -> None:
        if not self._started:
            raise DataTimeoutError("Mock backend is not started")

    def status(self) -> PlatformStatus:
        capabilities = {
            name: Capability(True, True, "Deterministic mock; not hardware evidence", "mock")
            for name in ("arm", "hand", "camera", "tactile", "recording")
        }
        return PlatformStatus(
            backend="mock",
            connected=self._started,
            firmware=None,
            model="simulated X2 Ultra",
            capabilities=capabilities,
            warnings=("All samples and commands are simulated.",),
        )

    def arm_state(self, timeout_s: float = 1.0) -> ArmState:
        del timeout_s
        self._require_started()
        with self._lock:
            joints = tuple(
                JointSample(name, position, 0.0, 0.0, fault_code=0)
                for name, position in zip(ARM_JOINT_NAMES, self._arm, strict=True)
            )
        return ArmState(self._timestamp(), joints, domain_state=0, source="mock/arm")

    def hand_states(self, timeout_s: float = 1.0) -> Mapping[Side, HandState]:
        del timeout_s
        self._require_started()
        stamp = self._timestamp()
        with self._lock:
            return {
                side: HandState(
                    stamp,
                    side,
                    1,
                    tuple(
                        JointSample(f"{side.value[0].upper()}_{suffix}", value, 0.0, 0.0, 0, 0)
                        for suffix, value in zip(HAND_JOINT_SUFFIXES, values, strict=True)
                    ),
                    "mock/hand",
                )
                for side, values in self._hands.items()
            }

    def tactile_frames(self, timeout_s: float = 1.0) -> Mapping[Side, TactileFrame]:
        del timeout_s
        self._require_started()
        stamp = self._timestamp()
        result: dict[Side, TactileFrame] = {}
        for side in Side:
            phase = 15 if side is Side.LEFT else 40
            palm_values = tuple((phase + index + self._frame_index) % 256 for index in range(25))
            back_values = tuple((phase + index // 2) % 256 for index in range(36))
            tips = {
                finger: TactileSurface(finger, (4, 4), tuple((phase + i * 2) % 256 for i in range(16)))
                for finger in ("thumb", "index", "middle", "ring", "little")
            }
            result[side] = TactileFrame(
                stamp,
                side,
                TactileSurface("palm", (5, 5), palm_values),
                TactileSurface("back_of_hand", (6, 6), back_values),
                tips,
                source="mock/tactile",
            )
        return result

    def camera_frame(self, camera: CameraName, timeout_s: float = 2.0) -> CameraFrame:
        del timeout_s
        self._require_started()
        self._frame_index += 1
        image = Image.new("RGB", (640, 360), (21, 28, 35))
        draw = ImageDraw.Draw(image)
        x = 40 + (self._frame_index * 13) % 520
        draw.rectangle((x, 105, x + 80, 255), fill=(28, 176, 142), outline=(240, 243, 245), width=3)
        draw.text((24, 22), f"{camera.value}  frame {self._frame_index}", fill=(240, 243, 245))
        output = BytesIO()
        image.save(output, format="JPEG", quality=85)
        return CameraFrame(
            self._timestamp(),
            camera,
            640,
            360,
            "jpeg",
            output.getvalue(),
            frame_id="mock_camera",
            compressed=True,
            source=f"mock/{camera.value}",
        )

    def mc_state(self, timeout_s: float = 1.0) -> McState:
        del timeout_s
        self._require_started()
        return McState(
            self._timestamp(),
            "mock",
            0,
            1000,
            self._motion_mode,
            100,
            0,
            0,
            "",
            0,
            self._fsm_state,
            self._body_state,
            1,
            1,
            "mock/mc",
        )

    def publish_arm(
        self,
        positions_rad: tuple[float, ...],
        velocities_rad_s: tuple[float, ...],
        efforts_nm: tuple[float, ...],
        stiffness: float,
        damping: float,
    ) -> None:
        del velocities_rad_s, efforts_nm, stiffness, damping
        if len(positions_rad) != 14 or not all(math.isfinite(value) for value in positions_rad):
            raise ValueError("Arm command requires 14 finite positions")
        with self._lock:
            self._arm[:] = positions_rad

    def publish_hand(self, side: Side, positions_rad: tuple[float, ...]) -> None:
        if len(positions_rad) != 10 or not all(math.isfinite(value) for value in positions_rad):
            raise ValueError("Hand command requires 10 finite positions")
        with self._lock:
            self._hands[side][:] = positions_rad

    def control_preflight(self, subsystem: str, *, require_enabled: bool = True) -> ControlPreflight:
        if subsystem not in {"arm", "hand"}:
            raise ValueError(f"Unknown subsystem: {subsystem}")
        checks = (
            PreflightCheck(
                "control_enabled",
                self.config.control.enabled,
                f"control.enabled={str(self.config.control.enabled).lower()}",
            ),
            PreflightCheck("mock_backend", True, "Simulated result; not hardware evidence"),
        )
        report = ControlPreflight(subsystem, "mock", time.monotonic_ns(), checks, self.mc_state())
        if require_enabled and not report.ready:
            raise SafetyInterlockError("Control preflight failed: control.enabled=false")
        return report

    def set_motion_mode(self, mode: str) -> McState:
        if not mode:
            raise ValueError("Mode must be non-empty")
        self._motion_mode = mode
        return self.mc_state()
