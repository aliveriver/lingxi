from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .client import X2Client
from .models import ArmCommand, CameraName, HandCommand, Observation


@dataclass(frozen=True, slots=True)
class PolicyAction:
    arm: ArmCommand | None = None
    hands: tuple[HandCommand, ...] = ()
    stop: bool = False


class Policy(Protocol):
    def reset(self) -> None: ...
    def act(self, observation: Observation) -> PolicyAction: ...


class ExperimentRunner:
    """Thin policy loop for VLA/WAM/WLA integrations."""

    def __init__(self, client: X2Client, policy: Policy):
        self.client = client
        self.policy = policy

    def run(
        self,
        *,
        max_steps: int,
        cameras: tuple[CameraName, ...] = (CameraName.RGBD_FRONT_RGB,),
        include_tactile: bool = True,
        allow_hardware_actions: bool = False,
    ) -> int:
        self.policy.reset()
        for step in range(max_steps):
            observation = self.client.observe(cameras, include_tactile)
            action = self.policy.act(observation)
            if action.stop:
                return step
            if action.arm is not None:
                self.client.move_arm(action.arm, confirm_hardware=allow_hardware_actions)
            for hand in action.hands:
                self.client.move_hand(hand, confirm_hardware=allow_hardware_actions)
        return max_steps
