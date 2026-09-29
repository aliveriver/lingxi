"""Offline verdicts for fixed-command single-axis traces; never grants actuation.

Tolerances are explicit engineering review criteria, not manufacturer guarantees.
Source/receive clocks are checked separately; this is not sensor synchronization.
"""
from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
from statistics import median

from .models import ARM_JOINT_NAMES
from .replay import strict_json


@dataclass(frozen=True)
class AcceptanceCriteria:
    window_s: float = .2
    min_dwell_s: float = .8
    min_window_samples: int = 10
    max_stream_gap_s: float = .05
    max_mc_gap_s: float = .25
    command_tolerance_rad: float = 1e-5
    following_tolerance_rad: float = .002
    absolute_position_tolerance_rad: float = .005
    return_tolerance_rad: float = .001
    other_joint_drift_rad: float = .005
    settled_spread_rad: float = .002
    repeatability_spread_rad: float = .001
    min_repetitions: int = 3

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"Invalid criterion {name}")
        if self.window_s > self.min_dwell_s:
            raise ValueError("Window must fit inside minimum dwell")
        if type(self.min_window_samples) is not int or self.min_window_samples < 2:
            raise ValueError("At least two window samples required")
        if type(self.min_repetitions) is not int or self.min_repetitions < 3:
            raise ValueError("Repeatability requires at least three separate sessions")


def _finite_vector(values, count):
    return (isinstance(values, (list, tuple)) and len(values) == count
            and all(type(v) in (int, float) and math.isfinite(v) for v in values))


def _standing(mc, action, *, event=False):
    return (mc.get("action") == action and mc.get("action_status") == 100
            and mc.get("fsm_state" if event else "fsm") == 4
            and mc.get("body_state" if event else "body") == 1)


def evaluate_trace(trace: dict, criteria: AcceptanceCriteria | None = None) -> dict:
    """Return fail/inconclusive/pass_under_criteria from source samples, not summaries."""
    c = criteria or AcceptanceCriteria()
    quality, checks, windows = [], [], {}
    report = {"kind": "fixed_baseline_acceptance_v1", "criteria": asdict(c),
              "criteria_authority": "engineering review defaults; not manufacturer acceptance limits",
              "verdict": "inconclusive", "quality_checks": quality, "motion_checks": checks,
              "windows": windows, "hardware_acceptance_complete": False, "execution_authorized": False,
              "scope": "one fixed-command single-axis session; not multi-axis, contact or model-loop acceptance"}
    def quality_check(name, passed, detail=""):
        quality.append({"name": name, "passed": bool(passed), "detail": detail})
    def motion_check(name, measured, maximum):
        checks.append({"name": name, "passed": measured <= maximum,
                       "measured_rad": measured, "maximum_rad": maximum})
    try:
        result = trace["result"]
        if trace.get("kind") != "control_path_trace" or result.get("kind") != "fixed_command_baseline_session":
            raise ValueError("Expected a fixed-command-baseline control trace")
        index, delta, baseline = result["joint_index"], result["delta_rad"], result["baseline_command_rad"]
        if type(index) is not int or not 0 <= index < 14 or not _finite_vector(baseline, 14):
            raise ValueError("Invalid selected joint/baseline")
        if type(delta) not in (int, float) or not math.isfinite(delta) or not .01 <= abs(delta) <= .02:
            raise ValueError("Trace delta is outside the diagnostic 0.01–0.02 rad range")
        profile, ramp = result.get("trajectory_profile", "linear"), result.get("ramp_duration_s", 1.)
        if (profile not in {"linear", "quintic"} or type(ramp) not in (int, float)
                or not math.isfinite(ramp) or not 1 <= ramp <= 3
                or result.get("gravity_compensation_enabled", False) is not False):
            raise ValueError("Unsupported trajectory profile, ramp duration or compensation")
        report.update(joint_index=index, joint_name=ARM_JOINT_NAMES[index], delta_rad=delta,
                      baseline_command_rad=baseline, trajectory_profile=profile, ramp_duration_s=ramp)
        events = result["events"]
        names = [e["phase"] for e in events]
        expected = ["before_mode", "baseline_hold", "target_ramp", "target_hold", "recovery_ramp",
                    "recovery_hold", "recovery_complete", "publishing_stopped", "final"]
        if names != expected or any(type(e["monotonic_ns"]) is not int for e in events):
            raise ValueError("Missing, duplicate or unexpected phase events")
        times = [e["monotonic_ns"] for e in events]
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError("Phase event times must increase")
        by_name = {e["phase"]: e for e in events}
        start, stop = times[1], times[6]
        report.update(start_monotonic_ns=start, stop_monotonic_ns=stop)
        quality_check("session_finished_without_error", not result.get("error") and not result.get("feedback_error"))
        quality_check("trace_parse_errors", trace.get("errors") == [])
        kinds = ("upper", "hal_arm", "arm_state", "mc")
        quality_check("trace_no_buffer_drops", all(trace.get("dropped", {}).get(k) == 0 for k in kinds))
        active = {}
        for kind in kinds:
            rows = trace["samples"][kind]
            if not rows or any(type(r["received_monotonic_ns"]) is not int or type(r["stamp_ns"]) is not int
                               or r["stamp_ns"] <= 0 for r in rows):
                raise ValueError(f"Missing or invalid clocks for {kind}")
            for clock in ("received_monotonic_ns", "stamp_ns"):
                quality_check(f"{kind}_{clock}_increasing", all(b[clock] > a[clock] for a, b in zip(rows, rows[1:])))
            active[kind] = [r for r in rows if start <= r["received_monotonic_ns"] < stop]
            selected = active[kind]
            if not selected:
                raise ValueError(f"No active samples for {kind}")
            receive = [start, *(r["received_monotonic_ns"] for r in selected), stop]
            gap = max(b-a for a, b in zip(receive, receive[1:])) / 1e9
            limit = c.max_mc_gap_s if kind == "mc" else c.max_stream_gap_s
            quality_check(f"{kind}_receive_coverage", gap <= limit, f"max gap incl. edges {gap:.9f} s; limit {limit:g} s")
        quality_check("urs_standing_throughout", all(_standing(r, "UPPERBODY_REMOTE_SPLIT") for r in active["mc"]))
        restored = result.get("mode_restore", {}).get("mc_after", {})
        final_mc = by_name["final"].get("mc", {})
        quality_check("restored_standing", _standing(restored, "STAND_DEFAULT", event=True)
                      and _standing(final_mc, "STAND_DEFAULT", event=True))
        final_time = by_name["final"]["monotonic_ns"]
        def final_fresh(state):
            stamp = state.get("timestamp", {}).get("monotonic_ns")
            return type(stamp) is int and abs(final_time-stamp)/1e9 <= c.max_mc_gap_s
        quality_check("restoration_feedback_fresh", final_fresh(restored) and final_fresh(final_mc))
        final_arm = by_name["final"].get("arm", {})
        quality_check("final_arm_faults_zero", final_fresh(final_arm)
                      and tuple(j.get("name") for j in final_arm.get("joints", [])) == ARM_JOINT_NAMES
                      and all(j.get("fault_code") == 0 for j in final_arm["joints"]))
        final_hands = result.get("final_hands", {})
        quality_check("final_hand_faults_zero", all(final_fresh(final_hands.get(side, {}))
                      and len(final_hands.get(side, {}).get("joints", [])) == 10
                      and all(j.get("fault_code") == 0 for j in final_hands[side]["joints"]) for side in ("left", "right")))
        for kind in ("hal_arm", "arm_state"):
            for row in active[kind]:
                if tuple(j.get("name") for j in row["joints"]) != ARM_JOINT_NAMES:
                    raise ValueError(f"{kind} must contain complete canonical joint order")
                if any(not _finite_vector([j.get("position"), j.get("velocity"), j.get("effort")], 3) for j in row["joints"]):
                    raise ValueError(f"Nonfinite or missing {kind} joint values")
        quality_check("arm_faults_zero", all(j.get("error_code") == 0 for row in active["arm_state"] for j in row["joints"]))
        upper = active["upper"]
        if any(not _finite_vector(r["arm_pos"], 14) or not _finite_vector(r["hand_pos"], 20) for r in upper):
            raise ValueError("Upper command axis count/values invalid")
        quality_check("upper_sequence", all(type(r["sequence"]) is int for r in upper)
                      and all(b["sequence"] == a["sequence"]+1 for a, b in zip(upper, upper[1:])))
        quality_check("hand_commands_held", all(r.get("hand_sub_mode") == 2 and
                      max(abs(a-b) for a, b in zip(r["hand_pos"], upper[0]["hand_pos"])) <= c.command_tolerance_rad for r in upper))
        for kind in ("upper", "hal_arm"):
            vectors = [r["arm_pos"] if kind == "upper" else [j["position"] for j in r["joints"]] for r in active[kind]]
            other_error = max(abs(q[i]-baseline[i]) for q in vectors for i in range(14) if i != index)
            lo, hi = sorted((baseline[index], baseline[index]+delta))
            quality_check(f"{kind}_fixed_baseline_single_axis", other_error <= c.command_tolerance_rad
                          and all(lo-c.command_tolerance_rad <= q[index] <= hi+c.command_tolerance_rad for q in vectors))
        for phase, next_phase in (("baseline_hold", "target_ramp"), ("target_hold", "recovery_ramp"), ("recovery_hold", "recovery_complete")):
            end = by_name[next_phase]["monotonic_ns"]
            begin = end-round(c.window_s*1e9)
            quality_check(f"{phase}_dwell_duration", (end-by_name[phase]["monotonic_ns"])/1e9 >= c.min_dwell_s)
            window = {"start_monotonic_ns": begin, "stop_monotonic_ns": end}
            for kind in ("hal_arm", "arm_state"):
                rows = [r for r in active[kind] if begin <= r["received_monotonic_ns"] < end]
                if not rows:
                    raise ValueError(f"Missing {phase}/{kind} window")
                quality_check(f"{phase}_{kind}_sample_count", len(rows) >= c.min_window_samples)
                columns = [[r["joints"][i]["position"] for r in rows] for i in range(14)]
                window[kind] = {"count": len(rows), "position_median_rad": [median(v) for v in columns],
                                "position_spread_rad": [max(v)-min(v) for v in columns]}
                if kind == "hal_arm":
                    expected_q = list(baseline)
                    if phase == "target_hold": expected_q[index] += delta
                    error = max(abs(v-expected_q[i]) for i, values in enumerate(columns) for v in values)
                    quality_check(f"{phase}_hal_target_delivered", error <= c.command_tolerance_rad,
                                  f"max command error {error:.9f} rad")
            windows[phase] = window
        base = windows["baseline_hold"]["arm_state"]["position_median_rad"]
        target = windows["target_hold"]["arm_state"]["position_median_rad"]
        recovery = windows["recovery_hold"]["arm_state"]["position_median_rad"]
        movement = target[index]-base[index]
        residual = recovery[index]-base[index]
        report["metrics"] = {"encoder_target_delta_rad": movement, "encoder_recovery_delta_rad": residual,
                             "tracking_fraction": movement/delta,
                             "absolute_target_error_rad": target[index]-(baseline[index]+delta)}
        motion_check("increment_following_error", abs(movement-delta), c.following_tolerance_rad)
        motion_check("selected_absolute_position_error", max(abs(base[index]-baseline[index]),
                     abs(target[index]-baseline[index]-delta), abs(recovery[index]-baseline[index])),
                     c.absolute_position_tolerance_rad)
        motion_check("return_residual", abs(residual), c.return_tolerance_rad)
        motion_check("nonselected_axis_drift", max(abs(row["joints"][i]["position"]-base[i])
                     for row in active["arm_state"] for i in range(14) if i != index), c.other_joint_drift_rad)
        motion_check("settled_window_spread", max(v for w in windows.values() for v in w["arm_state"]["position_spread_rad"]), c.settled_spread_rad)
        if all(q["passed"] for q in quality):
            report["verdict"] = "pass_under_criteria" if all(check["passed"] for check in checks) else "fail"
    except (KeyError, TypeError, ValueError, IndexError, AttributeError) as exc:
        quality_check("trace_structure", False, str(exc))
    return report


def evaluate_file(path: str | Path, criteria: AcceptanceCriteria | None = None) -> dict:
    path = Path(path)
    # Read once so the digest identifies exactly the bytes analyzed.
    raw = path.read_bytes()
    report = evaluate_trace(strict_json(raw.decode("utf-8")), criteria)
    return {"source_path": str(path), "source_sha256": hashlib.sha256(raw).hexdigest(), **report}


def evaluate_repeatability(reports: list[dict], criteria: AcceptanceCriteria | None = None) -> dict:
    c = criteria or AcceptanceCriteria()
    problems = []
    if len(reports) < c.min_repetitions:
        problems.append(f"Need at least {c.min_repetitions} separate sessions")
    hashes = [r.get("source_sha256") for r in reports]
    if any(not h for h in hashes) or len(set(hashes)) != len(hashes):
        problems.append("Missing or duplicate source hashes")
    if any(r.get("verdict") != "pass_under_criteria" for r in reports):
        problems.append("Every session must pass its single-axis criteria")
    if any(r.get("criteria") != asdict(c) for r in reports):
        problems.append("Criteria differ between sessions")
    if reports:
        first = reports[0]
        for r in reports:
            if (r.get("trajectory_profile") != first.get("trajectory_profile")
                    or r.get("ramp_duration_s") != first.get("ramp_duration_s")):
                problems.append("Repetitions must have the same trajectory profile and ramp duration")
                break
            if r.get("joint_index") != first.get("joint_index") or r.get("delta_rad") != first.get("delta_rad"):
                problems.append("Repetitions must have the same joint and signed delta")
                break
            a,b = r.get("baseline_command_rad"),first.get("baseline_command_rad")
            if not _finite_vector(a,14) or not _finite_vector(b,14) or max(abs(x-y) for x,y in zip(a,b)) > c.command_tolerance_rad:
                problems.append("Fixed command baselines differ")
                break
        spans = sorted((r.get("start_monotonic_ns", 0), r.get("stop_monotonic_ns", 0)) for r in reports)
        if any(a <= 0 or b <= a for a,b in spans) or any(b > next_a for (_,b),(next_a,_) in zip(spans,spans[1:])):
            problems.append("Sessions overlap or lack capture intervals; do not mix different PC2 boot clocks")
    spreads = {}
    if not problems:
        for metric in ("encoder_target_delta_rad", "encoder_recovery_delta_rad"):
            values = [r["metrics"][metric] for r in reports]
            spreads[metric] = max(values)-min(values)
    passed = not problems and all(v <= c.repeatability_spread_rad for v in spreads.values())
    return {"kind": "single_axis_repeatability_v1", "verdict": "pass_under_criteria" if passed else "not_passed",
            "criteria": asdict(c), "session_count": len(reports), "source_sha256": hashes,
            "problems": problems, "spread_rad": spreads, "execution_authorized": False,
            "hardware_acceptance_complete": False,
            "scope": "compatible numeric repeatability only; robot identity, controller configuration and setup require review"}
