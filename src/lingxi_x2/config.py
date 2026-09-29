from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field

from .errors import ConfigurationError


class RobotConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    model: str = "X2 Ultra"
    firmware: str | None = None
    pc1_host: str = "10.0.1.40"
    pc2_host: str = "10.0.1.41"


class RosConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    node_name: str = "lingxi_x2_platform"
    aimdk_prefix: str = "/agibot/software/common"
    camera_streams: list[str] = Field(default_factory=lambda: ["rgbd_front_rgb"])
    state_timeout_s: float = Field(default=0.5, gt=0)


class ControlConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    enabled: bool = False
    authority: Literal["observe_only", "hal_mc_stopped", "upper_body_mc"] = "observe_only"
    publish_rate_hz: float = Field(default=50.0, ge=20.0, le=200.0)
    command_echo: bool = False
    max_arm_step_rad: float = Field(default=0.05, gt=0, le=0.5)
    max_hand_step_rad: float = Field(default=0.15, gt=0, le=1.0)
    require_no_competing_publishers: bool = True
    allowed_competing_nodes: list[str] = Field(default_factory=list)


class WebConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    host: str = "127.0.0.1"
    port: int = Field(default=8080, ge=1, le=65535)


class PlatformConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")
    backend: Literal["auto", "mock", "ros2"] = "auto"
    robot: RobotConfig = Field(default_factory=RobotConfig)
    ros: RosConfig = Field(default_factory=RosConfig)
    control: ControlConfig = Field(default_factory=ControlConfig)
    web: WebConfig = Field(default_factory=WebConfig)


def load_config(path: str | Path | None = None) -> PlatformConfig:
    if path is None:
        return PlatformConfig()
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        return PlatformConfig.model_validate(raw)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise ConfigurationError(f"Cannot load config {config_path}: {exc}") from exc
