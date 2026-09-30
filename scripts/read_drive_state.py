"""Continuously record raw drive/control telemetry; no commands or services."""
import argparse
import json
import math
from pathlib import Path
import time

from lingxi_x2.backends.ros_environment import reexec_with_ros_environment
from lingxi_x2.client import X2Client
from lingxi_x2.control_trace import ControlTrace
from lingxi_x2.drive_trace import DriveTrace, review_drive_window


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/x2.yaml")
    parser.add_argument("--duration", type=float, default=4.)
    parser.add_argument("--output", type=Path, required=True, help="New JSON evidence file; never overwrite")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 0 < args.duration <= 60:
        parser.error("duration must be finite and in (0, 60]")
    if args.output.exists():
        parser.error("output already exists")
    reexec_with_ros_environment()
    result = {"kind": "drive_readonly_v2", "motion_commands_sent": 0, "mode_requests_sent": 0,
              "hardware_following_fixed": False}
    with args.output.open("x") as output:
        try:
            with X2Client(args.config) as client, ControlTrace(client._backend) as control, DriveTrace(client._backend) as drive:
                result["started_ns"] = time.monotonic_ns()
                try:
                    time.sleep(args.duration)
                finally:
                    result["stopped_ns"] = time.monotonic_ns()
                    result["control_trace"] = control.snapshot()
                    result["drive_trace"] = drive.snapshot()
                    result["drive_review"] = review_drive_window(result["drive_trace"], result["started_ns"], result["stopped_ns"])
        except BaseException as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            json.dump(result, output, allow_nan=False)
    print(json.dumps({"output": str(args.output), "drive_review": result["drive_review"]}), flush=True)
    return 0 if result["drive_review"]["raw_samples_available"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
