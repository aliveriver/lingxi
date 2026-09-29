from __future__ import annotations

import os
from pathlib import Path
import sys


def environment_updates(aimdk_prefix: str = "/agibot/software/common",
                        ros_prefix: str = "/opt/ros/humble") -> dict[str, str]:
    prefix = Path(aimdk_prefix)
    updates: dict[str, str] = {}

    def prepend(name: str, value: Path) -> None:
        existing = updates.get(name, os.environ.get(name, ""))
        parts = [part for part in existing.split(":") if part]
        rendered = str(value)
        parts = [part for part in parts if part != rendered]
        parts.insert(0, rendered)
        updates[name] = ":".join(parts)

    # A fresh noninteractive PC2 SSH session has no ROS underlay in its env.
    # Add only existing paths from the supported Humble/Python 3.10 install;
    # keep the firmware's AimDK overlay first, and retain caller path entries.
    # Do not alter ordinary development hosts that lack an AimDK installation.
    if prefix.is_dir():
        for root in (Path(ros_prefix), prefix):
            if not root.is_dir():
                continue
            prepend("AMENT_PREFIX_PATH", root)
            for relative in ("lib/python3.10/site-packages", "local/lib/python3.10/dist-packages"):
                python_path = root / relative
                if python_path.is_dir():
                    prepend("PYTHONPATH", python_path)
            lib_path = root / "lib"
            if lib_path.is_dir():
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

