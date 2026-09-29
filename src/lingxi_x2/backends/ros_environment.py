from __future__ import annotations

import os
from pathlib import Path
import sys


def environment_updates(aimdk_prefix: str = "/agibot/software/common") -> dict[str, str]:
    prefix = Path(aimdk_prefix)
    python_path = prefix / "local/lib/python3.10/dist-packages"
    lib_path = prefix / "lib"
    updates: dict[str, str] = {}

    def prepend(name: str, value: Path) -> None:
        existing = os.environ.get(name, "")
        parts = [part for part in existing.split(":") if part]
        rendered = str(value)
        if rendered not in parts:
            parts.insert(0, rendered)
        updates[name] = ":".join(parts)

    if prefix.exists():
        prepend("AMENT_PREFIX_PATH", prefix)
    if python_path.exists():
        prepend("PYTHONPATH", python_path)
    if lib_path.exists():
        prepend("LD_LIBRARY_PATH", lib_path)
    return updates


def prepare_current_process(aimdk_prefix: str = "/agibot/software/common") -> None:
    updates = environment_updates(aimdk_prefix)
    os.environ.update(updates)
    for path in updates.get("PYTHONPATH", "").split(":"):
        if path and path not in sys.path:
            sys.path.insert(0, path)


def reexec_with_ros_environment(aimdk_prefix: str = "/agibot/software/common") -> None:
    """Restart once so the dynamic linker sees AimDK's library directory."""
    if os.environ.get("LINGXI_X2_ROS_ENV") == "1":
        prepare_current_process(aimdk_prefix)
        return
    updates = environment_updates(aimdk_prefix)
    if not updates:
        return
    environment = os.environ.copy()
    environment.update(updates)
    environment["LINGXI_X2_ROS_ENV"] = "1"
    os.execve(sys.executable, [sys.executable, *sys.argv], environment)

