"""Read-only IMU/waist/arm snapshot. No command publishers or mode requests."""
import argparse
import json
from pathlib import Path

from lingxi_x2.backends.ros_environment import reexec_with_ros_environment
from lingxi_x2.gravity_capture import capture_readonly


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=3.)
    parser.add_argument("--output", type=Path, help="Write clean JSON separately from vendor DDS stdout")
    args = parser.parse_args()
    reexec_with_ros_environment()
    # Reserve the output before connecting; refuse to overwrite earlier evidence.
    if args.output is not None:
        with args.output.open("x", encoding="utf-8") as stream:
            result = capture_readonly(args.duration)
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.write("\n")
    else:
        print(json.dumps(capture_readonly(args.duration), indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
