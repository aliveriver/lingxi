from __future__ import annotations

from collections.abc import Iterator, Mapping
from contextlib import AbstractContextManager
from dataclasses import asdict
from itertools import chain, repeat
import math
import time

from .backends.base import Backend
from .backends.mock import MockBackend
from .config import PlatformConfig, load_config
from .errors import BackendUnavailableError, DataTimeoutError, SafetyInterlockError
from .motion_feedback import arm_tracking_report
from .models import (
    ArmCommand,
    ArmState,
    CameraFrame,
    CameraName,
    ControlPreflight,
    HandCommand,
    HandState,
    McState,
    Observation,
    PlatformStatus,
    Side,
    TactileFrame,
)
from .safety import validate_acceptance_delta, validate_arm_target, validate_hand_target
from .publication import publish_points, summarize_frames


class X2Client(AbstractContextManager["X2Client"]):
    """Hardware-independent API used by policies, experiments, CLI, and Web UI."""

    def __init__(self, config: PlatformConfig | str | None = None, backend: Backend | None = None):
        self.config = config if isinstance(config, PlatformConfig) else load_config(config)
        self._backend = backend or self._make_backend()
        self._started = False
        self._publication_frames: list[dict[str, object]] = []

    def publication_stats(self) -> dict[str, object]:
        """Most recent stream, including successful calls before an exception."""
        return {**summarize_frames(self._publication_frames, self.config.control.publish_rate_hz),
                **self._backend.stream_diagnostics()}

    def _make_backend(self) -> Backend:
        if self.config.backend == "mock":
            return MockBackend(self.config)
        if self.config.backend in {"auto", "ros2"}:
            try:
                from .backends.ros2 import Ros2Backend

                return Ros2Backend(self.config)
            except BackendUnavailableError:
                if self.config.backend == "ros2":
                    raise
        return MockBackend(self.config)

    def start(self) -> "X2Client":
        if not self._started:
            self._backend.start()
            self._started = True
        return self

    connect = start

    def close(self) -> None:
        if self._started:
            self._backend.close()
            self._started = False

    def __enter__(self) -> "X2Client":
        return self.start()

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def status(self) -> PlatformStatus:
        return self._backend.status()

    def arm_state(self, timeout_s: float = 1.0) -> ArmState:
        return self._backend.arm_state(timeout_s)

    def hand_states(self, timeout_s: float = 1.0) -> Mapping[Side, HandState]:
        return self._backend.hand_states(timeout_s)

    def tactile_frames(self, timeout_s: float = 1.0) -> Mapping[Side, TactileFrame]:
        return self._backend.tactile_frames(timeout_s)

    def camera_frame(self, camera: CameraName | str, timeout_s: float = 2.0) -> CameraFrame:
        return self._backend.camera_frame(CameraName(camera), timeout_s)

    def mc_state(self, timeout_s: float = 1.0) -> McState:
        return self._backend.mc_state(timeout_s)

    def control_preflight(self, subsystem: str = "arm") -> ControlPreflight:
        """Run read-only checks. A failed report never publishes or calls a service."""
        return self._backend.control_preflight(subsystem, require_enabled=False)

    def observe(
        self,
        cameras: tuple[CameraName | str, ...] = (),
        include_tactile: bool = True,
        timeout_s: float = 2.0,
    ) -> Observation:
        arm = self.arm_state(timeout_s)
        hands = self.hand_states(timeout_s)
        tactile: Mapping[Side, TactileFrame] = {}
        if include_tactile:
            tactile = self.tactile_frames(timeout_s)
        frames = {CameraName(camera): self.camera_frame(camera, timeout_s) for camera in cameras}
        return Observation(time.monotonic_ns(), arm, hands, tactile, frames)

    def stream_observations(
        self,
        rate_hz: float = 10.0,
        cameras: tuple[CameraName | str, ...] = (),
        include_tactile: bool = True,
    ) -> Iterator[Observation]:
        if not 0 < rate_hz <= 200:
            raise ValueError("rate_hz must be in (0, 200]")
        period = 1.0 / rate_hz
        deadline = time.monotonic()
        while self._started:
            yield self.observe(cameras, include_tactile, max(1.0, period * 2))
            deadline = max(deadline + period, time.monotonic())
            # A slow consumer must not trigger a burst of backlogged observations.
            if deadline <= time.monotonic():
                deadline = time.monotonic() + period
            time.sleep(max(0.0, deadline - time.monotonic()))

    @staticmethod
    def _require_confirmation(confirm_hardware: bool) -> None:
        if not confirm_hardware:
            raise SafetyInterlockError(
                "Hardware command requires confirm_hardware=True after the operator verifies the physical safety state"
            )

    @staticmethod
    def _trajectory(current: tuple[float, ...], target: tuple[float, ...], duration_s: float, rate_hz: float, max_step: float):
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise SafetyInterlockError("duration_s must be positive")
        largest = max(abs(goal - start) for start, goal in zip(current, target, strict=True))
        steps = max(1, math.ceil(duration_s * rate_hz), math.ceil(largest / max_step))
        for step in range(1, steps + 1):
            alpha = step / steps
            yield tuple(start + (goal - start) * alpha for start, goal in zip(current, target, strict=True))

    def run_mock_compensated_session(self, urdf, request: dict, *, journal) -> dict:
        """Execute a fully prevalidated scenario on an actual MockBackend only."""
        from .compensation_runner import execute_mock_compensated_session
        return execute_mock_compensated_session(self, urdf, request, journal)

    def move_arm(self, command: ArmCommand, *, confirm_hardware: bool = False, hold_duration_s: float = 0.0) -> dict[str, object]:
        """Publish a trajectory and report final encoder error separately.

        Returning normally means publishing completed, not that the arm arrived.
        The feedback snapshot is not a settled-window acceptance verdict.
        """
        self._require_confirmation(confirm_hardware)
        self._validate_duration(command.duration_s, "duration_s")
        self._validate_duration(hold_duration_s, "hold_duration_s", allow_zero=True)
        target = validate_arm_target(command.positions_rad)
        current_state = self.arm_state()
        if len(current_state.joints) != 14:
            raise SafetyInterlockError(f"Expected 14 arm feedback joints, got {len(current_state.joints)}")
        current = tuple(joint.position_rad for joint in current_state.joints)
        rate = self.config.control.publish_rate_hz
        velocities = command.velocities_rad_s or (0.0,) * 14
        efforts = command.efforts_nm or (0.0,) * 14
        if len(velocities) != 14 or len(efforts) != 14:
            raise SafetyInterlockError("Arm velocity and effort vectors must contain 14 values")
        if self.config.control.authority == "upper_body_mc" and (
            any(float(v) != 0.0 for v in (*velocities, *efforts))
            or command.stiffness_nm_rad != 20.0 or command.damping_nm_s_rad != 2.0
        ):
            raise SafetyInterlockError(
                "upper_body_mc accepts position targets only; velocity, effort, stiffness and damping "
                "cannot be set through UpperBodyCommandArray. MC owns the actual gains; "
                "ArmCommand defaults are placeholders, not effective MC gains."
            )
        points = chain(
            (("trajectory", point) for point in self._trajectory(
                current, target, command.duration_s, rate, self.config.control.max_arm_step_rad
            )),
            repeat(("dwell", target), math.ceil(hold_duration_s * rate)),
        )
        def publish(point):
            return self._backend.publish_arm(
                point,
                tuple(float(value) for value in velocities),
                tuple(float(value) for value in efforts),
                command.stiffness_nm_rad,
                command.damping_nm_s_rad,
            )
        self._publication_frames = []
        with self._backend.command_stream("arm"):
            publish_points(points, publish, rate, self._publication_frames)
            # Read while the same stream is still owned, without waiting or
            # sending corrective targets. Missing feedback cannot imply arrival.
            try:
                feedback = self.arm_state(timeout_s=0.0)
                tracking = arm_tracking_report(
                    target, feedback, now_ns=time.monotonic_ns(),
                    not_before_ns=self._publication_frames[-1]["publish_return_monotonic_ns"],
                    max_age_ns=round(self.config.ros.state_timeout_s * 1e9),
                )
            except DataTimeoutError as exc:
                tracking = {"status": "unavailable", "motion_verified": False,
                            "reason": "feedback_timeout", "detail": str(exc)}
        return {**self.publication_stats(), "tracking": tracking}

    @staticmethod
    def _validate_duration(value: float, label: str, *, allow_zero: bool = False) -> None:
        if not math.isfinite(value) or value < 0 or (not allow_zero and value == 0):
            raise SafetyInterlockError(f"{label} must be finite and {'nonnegative' if allow_zero else 'positive'}")

    def move_hand(self, command: HandCommand, *, confirm_hardware: bool = False) -> dict[str, object]:
        self._require_confirmation(confirm_hardware)
        self._validate_duration(command.duration_s, "duration_s")
        target = validate_hand_target(command.positions_rad)
        state = self.hand_states()[command.side]
        if state.hand_type != 1:
            raise SafetyInterlockError(f"{command.side.value} hand type is {state.hand_type}, expected NIMBLE_HANDS=1")
        if len(state.joints) != 10:
            raise SafetyInterlockError(f"Expected 10 hand feedback joints, got {len(state.joints)}")
        current = tuple(joint.position_rad for joint in state.joints)
        rate = self.config.control.publish_rate_hz
        points = (("trajectory", point) for point in self._trajectory(
            current, target, command.duration_s, rate, self.config.control.max_hand_step_rad
        ))
        self._publication_frames = []
        with self._backend.command_stream("hand"):
            publish_points(points, lambda point: self._backend.publish_hand(command.side, point), rate, self._publication_frames)
        return self.publication_stats()

    def hold_upper_body(self, duration_s: float = 1.0, *, confirm_hardware: bool = False) -> dict[str, object]:
        """Continuously command the measured arm pose while preserving measured hand targets."""
        self._require_confirmation(confirm_hardware)
        if self.config.control.authority != "upper_body_mc":
            raise SafetyInterlockError("Upper-body hold requires control.authority=upper_body_mc")
        before = self.arm_state()
        target = tuple(joint.position_rad for joint in before.joints)
        mc_before = self.mc_state()
        started_ns = time.monotonic_ns()
        stats = self.move_arm(ArmCommand.from_positions(target, duration_s), confirm_hardware=True)
        after = self.arm_state()
        mc_after = self.mc_state()
        feedback = tuple(joint.position_rad for joint in after.joints)
        tracking_error = tuple(actual - commanded for commanded, actual in zip(target, feedback, strict=True))
        return {
            "kind": "upper_body_hold",
            "publication_stats": stats,
            "started_monotonic_ns": started_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "command_positions_rad": target,
            "feedback_positions_rad": feedback,
            "tracking_error_rad": tracking_error,
            "max_abs_tracking_error_rad": max(abs(value) for value in tracking_error),
            "mc_before": asdict(mc_before),
            "mc_after": asdict(mc_after),
            "publish_rate_hz": self.config.control.publish_rate_hz,
            "events": (
                {
                    "phase": "baseline",
                    "monotonic_ns": started_ns,
                    "arm": asdict(before),
                    "mc": asdict(mc_before),
                },
                {
                    "phase": "publishing_stopped",
                    "monotonic_ns": time.monotonic_ns(),
                    "arm": asdict(after),
                    "mc": asdict(mc_after),
                },
            ),
        }

    def test_arm_joint(
        self,
        joint_index: int,
        delta_rad: float,
        *,
        move_duration_s: float = 1.0,
        target_hold_s: float = 0.0,
        recovery_duration_s: float = 1.0,
        recovery_hold_s: float = 0.0,
        stop_observation_s: float = 0.25,
        confirm_hardware: bool = False,
    ) -> dict[str, object]:
        """Mock-only joint test within the configured acceptance input range.

        Hardware rejects this measured-baseline path before planning or motion.
        """
        self._require_confirmation(confirm_hardware)
        self._reject_hardware_measured_baseline_test()
        if self.config.control.authority != "upper_body_mc":
            raise SafetyInterlockError("Single-joint acceptance test requires control.authority=upper_body_mc")
        if not 0 <= joint_index < 14:
            raise SafetyInterlockError("joint_index must be in [0, 13]")
        delta = validate_acceptance_delta(delta_rad)
        for label, value in (("move_duration_s", move_duration_s), ("recovery_duration_s", recovery_duration_s)):
            self._validate_duration(value, label)
        for label, value in (("stop_observation_s", stop_observation_s), ("target_hold_s", target_hold_s), ("recovery_hold_s", recovery_hold_s)):
            self._validate_duration(value, label, allow_zero=True)

        baseline_state = self.arm_state()
        baseline = tuple(joint.position_rad for joint in baseline_state.joints)
        target = list(baseline)
        target[joint_index] += delta
        validate_arm_target(target)
        mc_before = self.mc_state()
        started_ns = time.monotonic_ns()
        events: list[dict[str, object]] = [
            {
                "phase": "baseline",
                "monotonic_ns": started_ns,
                "arm": asdict(baseline_state),
                "mc": asdict(mc_before),
            },
            {
                "phase": "target_command",
                "monotonic_ns": time.monotonic_ns(),
                "positions_rad": tuple(target),
            },
        ]

        target_stats = self.move_arm(ArmCommand.from_positions(target, move_duration_s), confirm_hardware=True, hold_duration_s=target_hold_s)
        target_feedback = self.arm_state()
        events.append(
            {
                "phase": "target_feedback",
                "monotonic_ns": time.monotonic_ns(),
                "arm": asdict(target_feedback),
                "mc": asdict(self.mc_state()),
            }
        )
        events.append(
            {
                "phase": "recovery_command",
                "monotonic_ns": time.monotonic_ns(),
                "positions_rad": baseline,
            }
        )
        recovery_stats = self.move_arm(ArmCommand.from_positions(baseline, recovery_duration_s), confirm_hardware=True, hold_duration_s=recovery_hold_s)
        recovery_feedback = self.arm_state()
        events.append(
            {
                "phase": "recovery_feedback",
                "monotonic_ns": time.monotonic_ns(),
                "arm": asdict(recovery_feedback),
                "mc": asdict(self.mc_state()),
            }
        )
        if stop_observation_s:
            time.sleep(stop_observation_s)
        stopped_feedback = self.arm_state()
        mc_after = self.mc_state()
        events.append(
            {
                "phase": "publishing_stopped",
                "monotonic_ns": time.monotonic_ns(),
                "arm": asdict(stopped_feedback),
                "mc": asdict(mc_after),
            }
        )
        return {
            "kind": "upper_body_single_joint_acceptance",
            "publication_stats": {"target": target_stats, "recovery": recovery_stats},
            "started_monotonic_ns": started_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "joint_index": joint_index,
            "joint_name": baseline_state.joints[joint_index].name,
            "delta_rad": delta,
            "baseline_positions_rad": baseline,
            "target_positions_rad": tuple(target),
            "target_feedback_rad": tuple(joint.position_rad for joint in target_feedback.joints),
            "recovery_feedback_rad": tuple(joint.position_rad for joint in recovery_feedback.joints),
            "post_stop_feedback_rad": tuple(joint.position_rad for joint in stopped_feedback.joints),
            "stop_observation_s": stop_observation_s,
            "target_hold_s": target_hold_s,
            "recovery_hold_s": recovery_hold_s,
            "mc_before": asdict(mc_before),
            "mc_after": asdict(mc_after),
            "publish_rate_hz": self.config.control.publish_rate_hz,
            "events": tuple(events),
        }

    def test_upper_body_hold_session(
        self,
        duration_s: float = 1.0,
        *,
        stop_observation_s: float = 0.5,
        confirm_hardware: bool = False,
    ) -> dict[str, object]:
        """Enter URS, hold measured pose, observe command loss, and restore standing mode."""
        self._require_confirmation(confirm_hardware)
        self._validate_duration(duration_s, "duration_s")
        self._validate_duration(stop_observation_s, "stop_observation_s", allow_zero=True)
        started_ns = time.monotonic_ns()
        mode_entry = self.set_motion_mode("UPPERBODY_REMOTE_SPLIT", confirm_hardware=True)
        hold: dict[str, object] | None = None
        post_stop: dict[str, object] | None = None
        try:
            hold = self.hold_upper_body(duration_s, confirm_hardware=True)
            if stop_observation_s:
                time.sleep(stop_observation_s)
            post_stop = {
                "monotonic_ns": time.monotonic_ns(),
                "arm": asdict(self.arm_state()),
                "mc": asdict(self.mc_state()),
            }
        finally:
            mode_restore = self.set_motion_mode("STAND_DEFAULT", confirm_hardware=True)
        return {
            "kind": "upper_body_hold_acceptance",
            "started_monotonic_ns": started_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "mode_entry": mode_entry,
            "hold": hold,
            "stop_observation_s": stop_observation_s,
            "post_stop": post_stop,
            "mode_restore": mode_restore,
        }

    def test_upper_body_joint_session(
        self,
        joint_index: int,
        delta_rad: float,
        *,
        settle_duration_s: float = 1.0,
        move_duration_s: float = 1.0,
        target_hold_s: float = 1.0,
        recovery_duration_s: float = 1.0,
        recovery_hold_s: float = 1.0,
        stop_observation_s: float = 0.5,
        confirm_hardware: bool = False,
    ) -> dict[str, object]:
        """Enter URS, settle, run one bounded joint test, and restore standing mode."""
        self._require_confirmation(confirm_hardware)
        self._reject_hardware_measured_baseline_test()
        if not 0 <= joint_index < 14:
            raise SafetyInterlockError("joint_index must be in [0, 13]")
        delta = validate_acceptance_delta(delta_rad)
        for label, value in (("settle_duration_s", settle_duration_s), ("move_duration_s", move_duration_s), ("recovery_duration_s", recovery_duration_s)):
            self._validate_duration(value, label)
        for label, value in (("target_hold_s", target_hold_s), ("recovery_hold_s", recovery_hold_s), ("stop_observation_s", stop_observation_s)):
            self._validate_duration(value, label, allow_zero=True)
        started_ns = time.monotonic_ns()
        mode_entry = self.set_motion_mode("UPPERBODY_REMOTE_SPLIT", confirm_hardware=True)
        qualification: dict[str, object] | None = None
        joint_test: dict[str, object] | None = None
        try:
            qualification = self.hold_upper_body(settle_duration_s, confirm_hardware=True)
            selected_error = abs(qualification["tracking_error_rad"][joint_index])
            maximum_qualification_error = abs(delta) / 2.0
            if selected_error > maximum_qualification_error:
                raise SafetyInterlockError(
                    f"Joint {joint_index} hold error {selected_error:.6f} rad exceeds "
                    f"qualification limit {maximum_qualification_error:.6f} rad"
                )
            joint_test = self.test_arm_joint(
                joint_index,
                delta,
                move_duration_s=move_duration_s,
                target_hold_s=target_hold_s,
                recovery_duration_s=recovery_duration_s,
                recovery_hold_s=recovery_hold_s,
                stop_observation_s=stop_observation_s,
                confirm_hardware=True,
            )
        finally:
            mode_restore = self.set_motion_mode("STAND_DEFAULT", confirm_hardware=True)
        return {
            "kind": "upper_body_single_joint_session",
            "started_monotonic_ns": started_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "mode_entry": mode_entry,
            "qualification_hold": qualification,
            "joint_test": joint_test,
            "mode_restore": mode_restore,
        }

    def _reject_hardware_measured_baseline_test(self) -> None:
        from .backends.ros2 import Ros2Backend

        if isinstance(self._backend, Ros2Backend):
            raise SafetyInterlockError(
                "Measured-feedback baseline acceptance is disabled on hardware: it rebases "
                "all joint targets onto tracking error. Use the operator-authorized "
                "scripts/command_baseline_session.py diagnostic with a recorded stable HAL baseline."
            )

    def set_motion_mode(self, mode: str, *, confirm_hardware: bool = False) -> dict[str, object]:
        self._require_confirmation(confirm_hardware)
        if not self.config.control.enabled:
            raise SafetyInterlockError("Hardware writes are disabled in config (control.enabled=false)")
        if self.config.control.authority != "upper_body_mc":
            raise SafetyInterlockError("MC mode changes require control.authority=upper_body_mc")
        allowed = {
            "PASSIVE_DEFAULT",
            "DAMPING_DEFAULT",
            "JOINT_DEFAULT",
            "STAND_DEFAULT",
            "LOCOMOTION_DEFAULT",
            "HEAD_ONLY",
            "UPPERBODY_REMOTE_SPLIT",
        }
        if mode not in allowed:
            raise SafetyInterlockError(f"Unsupported motion mode: {mode}")
        # A fresh DDS participant can need more than the normal 1 s state read
        # timeout to discover the transient-local MC publisher.
        before = self.mc_state(timeout_s=3.0)
        if mode == "UPPERBODY_REMOTE_SPLIT":
            prerequisites = (
                before.action == "STAND_DEFAULT"
                and before.action_status == 100
                and before.fsm_state == 4
                and before.body_state == 1
            )
            if not prerequisites:
                raise SafetyInterlockError(
                    "UPPERBODY_REMOTE_SPLIT requires STAND_DEFAULT/RUNNING, "
                    "FSM stable wire value 4, and body STAND(1); "
                    f"got action={before.action or '<empty>'}, status={before.action_status}, "
                    f"fsm={before.fsm_state}, body={before.body_state}"
                )
        after = self._backend.set_motion_mode(mode)
        return {"requested_mode": mode, "mc_before": asdict(before), "mc_after": asdict(after)}
