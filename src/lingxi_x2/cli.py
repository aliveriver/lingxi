from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

from .backends.ros_environment import reexec_with_ros_environment
from .client import X2Client
from .models import ArmCommand, CameraName, HandCommand, Side
from .recording import JsonlRecorder, observation_to_dict


def _json(value: object) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2, default=str))


def _vector(text: str, count: int) -> tuple[float, ...]:
    values = tuple(float(value.strip()) for value in text.split(",") if value.strip())
    if len(values) != count:
        raise argparse.ArgumentTypeError(f"Expected {count} comma-separated values, got {len(values)}")
    return values


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="x2", description="AgiBot X2 embodied-AI platform")
    parser.add_argument("--config", help="YAML configuration path")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status", help="Show connection and capability status")
    analyze = sub.add_parser("analyze-log", help="Analyze existing acceptance stdout without connecting to ROS")
    analyze.add_argument("path", type=Path)
    doctor = sub.add_parser("doctor", help="Run non-mutating interface checks")
    doctor.add_argument("--sample", action="store_true", help="Wait for live state and camera samples")

    preflight = sub.add_parser("preflight", help="Run read-only control and MC safety checks")
    preflight.add_argument("subsystem", choices=("arm", "hand"), default="arm", nargs="?")

    observe = sub.add_parser("observe", help="Print one structured observation")
    observe.add_argument("--camera", action="append", choices=[item.value for item in CameraName], default=[])
    observe.add_argument("--tactile", action="store_true")

    capture = sub.add_parser("capture", help="Write one camera frame")
    capture.add_argument("camera", choices=[item.value for item in CameraName])
    capture.add_argument("output", type=Path)
    tactile = sub.add_parser("tactile-map", help="Save a read-only raw tactile map PNG and JSON")
    tactile.add_argument("output", type=Path)

    record = sub.add_parser("record", help="Record observations as JSONL")
    record.add_argument("output", type=Path)
    record.add_argument("--duration", type=float, required=True)
    record.add_argument("--rate", type=float, default=10.0)
    record.add_argument("--camera", action="append", choices=[item.value for item in CameraName], default=[])
    record.add_argument("--tactile", action="store_true")
    record.add_argument("--omit-images", action="store_true")

    arm = sub.add_parser("arm", help="Move 14 arm joints through a time-scaled trajectory")
    arm.add_argument("--positions", required=True)
    arm.add_argument("--duration", type=float, default=2.0)
    arm.add_argument("--confirm-hardware", action="store_true")

    hand = sub.add_parser("hand", help="Move one OmniHand through a time-scaled trajectory")
    hand.add_argument("side", choices=[side.value for side in Side])
    hand.add_argument("--positions", required=True)
    hand.add_argument("--duration", type=float, default=1.0)
    hand.add_argument("--confirm-hardware", action="store_true")

    mode = sub.add_parser("mode", help="Request an official MC motion mode")
    mode.add_argument("name")
    mode.add_argument("--confirm-hardware", action="store_true")

    hold = sub.add_parser("hold", help="Hold the current upper-body pose (hardware requires explicit confirmation)")
    hold.add_argument("--duration", type=float, default=1.0)
    hold.add_argument("--confirm-hardware", action="store_true")

    hold_session = sub.add_parser("test-hold-session", help="Enter URS, hold, observe stop, and restore standing")
    hold_session.add_argument("--duration", type=float, default=1.0)
    hold_session.add_argument("--stop-observation", type=float, default=0.5)
    hold_session.add_argument("--confirm-hardware", action="store_true")

    joint_test = sub.add_parser("test-arm-joint", help="Run the bounded single-joint movement/recovery test")
    joint_test.add_argument("joint_index", type=int)
    joint_test.add_argument("delta_rad", type=float)
    joint_test.add_argument("--move-duration", type=float, default=1.0)
    joint_test.add_argument("--target-hold", type=float, default=0.0)
    joint_test.add_argument("--recovery-duration", type=float, default=1.0)
    joint_test.add_argument("--recovery-hold", type=float, default=0.0)
    joint_test.add_argument("--stop-observation", type=float, default=0.25)
    joint_test.add_argument("--confirm-hardware", action="store_true")

    joint_session = sub.add_parser(
        "test-arm-joint-session", help="Enter URS, settle, test one arm joint, and restore standing"
    )
    joint_session.add_argument("joint_index", type=int)
    joint_session.add_argument("delta_rad", type=float)
    joint_session.add_argument("--settle-duration", type=float, default=1.0)
    joint_session.add_argument("--move-duration", type=float, default=1.0)
    joint_session.add_argument("--target-hold", type=float, default=1.0)
    joint_session.add_argument("--recovery-duration", type=float, default=1.0)
    joint_session.add_argument("--recovery-hold", type=float, default=1.0)
    joint_session.add_argument("--stop-observation", type=float, default=0.5)
    joint_session.add_argument("--confirm-hardware", action="store_true")

    web = sub.add_parser("web", help="Start the experiment console")
    web.add_argument("--host")
    web.add_argument("--port", type=int)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "analyze-log":
        from .log_analysis import analyze_session, read_session

        try:
            _json(analyze_session(read_session(args.path.read_text(encoding="utf-8"))))
            return 0
        except (OSError, ValueError) as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
    if args.command in {"status", "doctor", "preflight", "observe", "capture", "tactile-map", "record", "arm", "hand", "mode", "hold", "test-hold-session", "test-arm-joint", "test-arm-joint-session", "web"}:
        # No-op on non-X2 hosts; required on PC2 before importing generated messages.
        reexec_with_ros_environment()
    if args.command == "web":
        import uvicorn

        from .web.app import create_app

        config_client = X2Client(args.config)
        host = args.host or config_client.config.web.host
        port = args.port or config_client.config.web.port
        uvicorn.run(create_app(config_client), host=host, port=port)
        return 0

    try:
        with X2Client(args.config) as client:
            if args.command == "status":
                _json(client.status().as_dict())
            elif args.command == "doctor":
                result: dict[str, object] = {}
                if args.sample:
                    result["arm_joints"] = len(client.arm_state().joints)
                    result["hands"] = {side.value: len(state.joints) for side, state in client.hand_states().items()}
                    result["mc_state"] = asdict(client.mc_state())
                    configured = client.config.ros.camera_streams
                    if configured:
                        result["camera"] = client.camera_frame(configured[0]).metadata()
                result["status"] = client.status().as_dict()
                _json(result)
            elif args.command == "preflight":
                _json(client.control_preflight(args.subsystem).as_dict())
            elif args.command == "observe":
                observation = client.observe(tuple(args.camera), args.tactile)
                _json(observation_to_dict(observation, include_image_data=False))
            elif args.command == "capture":
                frame = client.camera_frame(args.camera)
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_bytes(frame.data)
                _json(frame.metadata())
            elif args.command == "record":
                count = JsonlRecorder(client, args.output).run(
                    args.duration,
                    args.rate,
                    tuple(args.camera),
                    args.tactile,
                    not args.omit_images,
                )
                _json({"samples": count, "output": str(args.output)})
            elif args.command == "tactile-map":
                from .tactile import save_tactile_map

                _json(save_tactile_map(client.tactile_frames(timeout_s=3.0), args.output))
            elif args.command == "arm":
                positions = _vector(args.positions, 14)
                _json(client.move_arm(
                    ArmCommand.from_positions(positions, args.duration),
                    confirm_hardware=args.confirm_hardware,
                ))
            elif args.command == "hand":
                positions = _vector(args.positions, 10)
                _json(client.move_hand(
                    HandCommand.from_positions(args.side, positions, args.duration),
                    confirm_hardware=args.confirm_hardware,
                ))
            elif args.command == "mode":
                _json(client.set_motion_mode(args.name, confirm_hardware=args.confirm_hardware))
            elif args.command == "hold":
                _json(client.hold_upper_body(args.duration, confirm_hardware=args.confirm_hardware))
            elif args.command == "test-hold-session":
                _json(
                    client.test_upper_body_hold_session(
                        args.duration,
                        stop_observation_s=args.stop_observation,
                        confirm_hardware=args.confirm_hardware,
                    )
                )
            elif args.command == "test-arm-joint":
                _json(
                    client.test_arm_joint(
                        args.joint_index,
                        args.delta_rad,
                        move_duration_s=args.move_duration,
                        target_hold_s=args.target_hold,
                        recovery_duration_s=args.recovery_duration,
                        recovery_hold_s=args.recovery_hold,
                        stop_observation_s=args.stop_observation,
                        confirm_hardware=args.confirm_hardware,
                    )
                )
            elif args.command == "test-arm-joint-session":
                _json(
                    client.test_upper_body_joint_session(
                        args.joint_index,
                        args.delta_rad,
                        settle_duration_s=args.settle_duration,
                        move_duration_s=args.move_duration,
                        target_hold_s=args.target_hold,
                        recovery_duration_s=args.recovery_duration,
                        recovery_hold_s=args.recovery_hold,
                        stop_observation_s=args.stop_observation,
                        confirm_hardware=args.confirm_hardware,
                    )
                )
        return 0
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        if "client" in locals() and client._publication_frames:
            _json({"kind": "failed_command", "error": str(exc), "publication_stats": client.publication_stats()})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
