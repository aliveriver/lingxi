from types import SimpleNamespace

import pytest

from lingxi_x2.publication import publish_points, summarize_frames
from lingxi_x2.log_analysis import analyze_session, read_session


def test_delayed_publisher_does_not_catch_up_in_bursts(monkeypatch):
    clock = SimpleNamespace(ns=0)
    def sleep(seconds):
        clock.ns += round(seconds * 1e9)
    monkeypatch.setattr("lingxi_x2.publication.time", SimpleNamespace(
        monotonic=lambda: clock.ns / 1e9, monotonic_ns=lambda: clock.ns, sleep=sleep,
    ))
    frames = []
    def publish(point):
        start = clock.ns
        clock.ns += 70_000_000 if point == (1,) else 1_000_000
        return {"monotonic_ns": start, "sequence": point[0]}
    report = publish_points((("trajectory", (i,)) for i in range(4)), publish, 50, frames)
    assert [f["monotonic_ns"] for f in frames] == [0, 20_000_000, 110_000_000, 130_000_000]
    assert report["published_count"] == 4
    assert report["actual_hz"] == pytest.approx(3 / .13)
    assert report["max_frame_interval_s"] == pytest.approx(.09)
    assert report["sequence_continuous"] is True


def test_sequence_wrap_gap_and_unavailable_are_distinct():
    def report(seqs):
        return summarize_frames([{"monotonic_ns": i * 20_000_000, "sequence": s} for i, s in enumerate(seqs)], 50)
    assert report([2**32 - 1, 0, 1])["sequence_continuous"] is True
    assert report([1, 3])["sequence_discontinuities"] == 1
    assert report([None, None])["sequence_continuous"] is None
    assert report([1])["actual_hz"] is None


def test_variable_pre_publish_work_does_not_shorten_actual_publish_intervals(monkeypatch):
    clock = SimpleNamespace(ns=0)
    def sleep(seconds):
        clock.ns += round(seconds * 1e9)
    monkeypatch.setattr("lingxi_x2.publication.time", SimpleNamespace(
        monotonic=lambda: clock.ns / 1e9, monotonic_ns=lambda: clock.ns, sleep=sleep,
    ))
    def publish(point):
        clock.ns += 15_000_000 if point == (0,) else 0
        return {"monotonic_ns": clock.ns, "sequence": point[0]}
    frames = []
    publish_points((("trajectory", (i,)) for i in range(2)), publish, 50, frames)
    assert [f["monotonic_ns"] for f in frames] == [15_000_000, 35_000_000]


def test_analysis_ignores_non_json_dds_prefix_and_suffix():
    session = read_session('DDS {not json}\n{"kind":"session", "events":['
        '{"phase":"target_command", "monotonic_ns":1000000000},'
        '{"phase":"target_feedback", "monotonic_ns":3007000000}]}\nDDS trailing')
    assert analyze_session(session)["sections"]["session"]["phase_intervals"][0]["elapsed_s"] == pytest.approx(2.007)
