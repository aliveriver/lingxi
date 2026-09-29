from types import SimpleNamespace as NS

from lingxi_x2.control_trace import ControlTrace, trace_sample


def test_hal_trace_preserves_gains_and_joint_names():
    msg = NS(header=NS(stamp=NS(sec=2, nanosec=3)), joints=[NS(
        name="left_shoulder_pitch_joint", position=.01, velocity=0., effort=.4, stiffness=20., damping=2.)])
    sample = trace_sample("hal_arm", msg)
    assert sample["stamp_ns"] == 2000000003
    assert sample["joints"][0]["stiffness"] == 20.
    assert sample["joints"][0]["name"] == "left_shoulder_pitch_joint"


def test_trace_reports_overflow_and_parse_failure():
    recorder = ControlTrace(None, max_samples=2)
    recorder.active = True
    msg = NS(header=NS(stamp=NS(sec=0, nanosec=0), sequence=0), source="test", arm_pos=[0.]*14,
             hand_pos=[0.]*20, hand_sub_mode=2)
    for seq in range(3):
        msg.header.sequence = seq
        recorder._receive("upper", msg)
    recorder._receive("mc", NS())
    snapshot = recorder.snapshot()
    assert [s["sequence"] for s in snapshot["samples"]["upper"]] == [1, 2]
    assert snapshot["dropped"]["upper"] == 1
    assert len(snapshot["errors"]) == 1
