from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping
from contextlib import contextmanager
from typing import Any

from ..models import (
    ArmState,
    CameraFrame,
    CameraName,
    ControlPreflight,
    HandState,
    McState,
    PlatformStatus,
    Side,
    TactileFrame,
)


class Backend(ABC):
    """Transport boundary. Public callers should use :class:`X2Client`."""

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def status(self) -> PlatformStatus: ...

    @abstractmethod
    def arm_state(self, timeout_s: float = 1.0) -> ArmState: ...

    @abstractmethod
    def hand_states(self, timeout_s: float = 1.0) -> Mapping[Side, HandState]: ...

    @abstractmethod
    def tactile_frames(self, timeout_s: float = 1.0) -> Mapping[Side, TactileFrame]: ...

    @abstractmethod
    def camera_frame(self, camera: CameraName, timeout_s: float = 2.0) -> CameraFrame: ...

    @abstractmethod
    def mc_state(self, timeout_s: float = 1.0) -> McState: ...

    @abstractmethod
    def publish_arm(
        self,
        positions_rad: tuple[float, ...],
        velocities_rad_s: tuple[float, ...],
        efforts_nm: tuple[float, ...],
        stiffness: float,
        damping: float,
    ) -> dict[str, Any] | None: ...

    @abstractmethod
    def publish_hand(self, side: Side, positions_rad: tuple[float, ...]) -> dict[str, Any] | None: ...

    @abstractmethod
    def control_preflight(self, subsystem: str, *, require_enabled: bool = True) -> ControlPreflight: ...

    @contextmanager
    def command_stream(self, subsystem: str):
        """Authorize one trajectory; hardware adapters revoke on exit."""
        self.control_preflight(subsystem)
        yield

    def stream_diagnostics(self) -> dict[str, Any]:
        return {}

    def set_motion_mode(self, mode: str) -> McState:
        raise NotImplementedError
