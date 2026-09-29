"""Report settled-window command/encoder data, optionally export a plot."""
import argparse
import json
from pathlib import Path
from statistics import median


def analyze(trace):
    result = trace["result"]
    events = result["events"]
    index = result["joint_index"]
    phases = {}
    for a, b in zip(events, events[1:]):
        if a["phase"] not in {"baseline_hold", "target_hold", "recovery_hold"}:
            continue
        stop = b["monotonic_ns"]
        start = max(a["monotonic_ns"], stop-200_000_000)
        phase = {"start_monotonic_ns": start, "stop_monotonic_ns": stop}
        for kind in ("hal_arm", "arm_state"):
            samples = [s for s in trace["samples"][kind] if start <= s["received_monotonic_ns"] < stop]
            if not samples:
                raise ValueError(f"No samples for {a['phase']}/{kind}")
            phase[kind] = {
                "count": len(samples),
                "position_median_rad": [median(s["joints"][i]["position"] for s in samples) for i in range(14)],
                "selected_effort_median": median(s["joints"][index]["effort"] for s in samples),
            }
        phases[a["phase"]] = phase
    baseline = phases["baseline_hold"]
    target = phases["target_hold"]
    recovery = phases["recovery_hold"]
    def delta(phase, kind):
        return phase[kind]["position_median_rad"][index]-baseline[kind]["position_median_rad"][index]
    return {"kind": "fixed_baseline_trace_analysis", "window_s": .2,
            "phases": phases,
            "hal_target_delta_rad": delta(target, "hal_arm"),
            "encoder_target_delta_rad": delta(target, "arm_state"),
            "encoder_recovery_delta_rad": delta(recovery, "arm_state"),
            "note": "HAL target delivery is observable; this report does not infer the drive/mechanical cause."}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("--plot", type=Path)
    args = parser.parse_args()
    trace = json.loads(args.trace.read_text())
    print(json.dumps(analyze(trace), indent=2))
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        events = trace["result"]["events"]
        start = next(e["monotonic_ns"] for e in events if e["phase"] == "baseline_hold")
        stop = next(e["monotonic_ns"] for e in events if e["phase"] == "publishing_stopped")
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        for kind, label, color in (("upper", "Upper-body target", "#d97706"),
                                   ("hal_arm", "MC -> HAL target", "#2563eb"),
                                   ("arm_state", "Encoder", "#059669")):
            rows = [r for r in trace["samples"][kind] if start <= r["received_monotonic_ns"] <= stop]
            times = [(r["received_monotonic_ns"]-start)/1e9 for r in rows]
            positions = [r["arm_pos"][0] if kind == "upper" else r["joints"][0]["position"] for r in rows]
            axes[0].plot(times, positions, label=label, color=color, linewidth=1.3,
                         linestyle="--" if kind == "upper" else "-")
            if kind == "arm_state":
                axes[1].plot(times, [r["joints"][0]["effort"] for r in rows], color=color, linewidth=.7)
        for event in events:
            t = (event["monotonic_ns"]-start)/1e9
            if start <= event["monotonic_ns"] <= stop:
                for axis in axes:
                    axis.axvline(t, color="gray", alpha=.25)
        axes[0].set_ylabel("Left shoulder pitch (rad)")
        axes[0].legend(loc="upper left")
        axes[1].set_ylabel("Reported joint effort (N m)")
        axes[1].set_xlabel("PC2 receive time from baseline hold (s)")
        axes[0].set_title(f"X2 Ultra / v1.1.4: {trace['result']['delta_rad']:+g} rad fixed-command-baseline test")
        for axis in axes:
            axis.grid(alpha=.2)
        fig.tight_layout()
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.plot, dpi=160)


if __name__ == "__main__":
    main()
