from __future__ import annotations

from pathlib import Path

import pytest

from lingxi_x2.config import PlatformConfig, load_config
from lingxi_x2.errors import ConfigurationError


def test_example_config_loads() -> None:
    config = load_config(Path("config/x2.example.yaml"))
    assert config.backend == "ros2"
    assert config.control.enabled is False
    assert config.control.authority == "upper_body_mc"
    assert config.control.publish_rate_hz == 50.0
    assert config.robot.firmware == "v1.1.4"
    assert config.robot.pc2_host == "10.0.1.41"


def test_unknown_config_key_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text("unknown: true\n", encoding="utf-8")
    with pytest.raises(ConfigurationError):
        load_config(path)


def test_defaults_are_observe_only() -> None:
    config = PlatformConfig()
    assert config.control.authority == "observe_only"
    assert not config.control.enabled
    assert config.control.publish_rate_hz == 50.0
