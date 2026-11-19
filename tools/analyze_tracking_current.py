"""Offline audit of existing ROS/CAN tracking captures; never accesses hardware.

get_current[3:8] on G474 is one coherent native dq snapshot. Current-loop
errors below use that snapshot, not time-aligned samples from separate RPCs.
PD reconstruction is deliberately labelled approximate: host receive timestamps
and separate control/feedback reads are not a firmware-time coherent snapshot.
"""
import argparse
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CASES = (("j1-3deg-20261116", 101, 72),
         ("j2-modes-20261116", 102, 36),
         ("j2-minus10-plus10", 102, 36))


def nearest(source, target):
    idx = np.searchsorted(source, target)
    right = np.clip(idx, 0, len(source)-1)
    left = np.clip(idx-1, 0, len(source)-1)
    return np.where(abs(source[left]-target) <= abs(source[right]-target), left, right)


def audit(folder, node, gear, output):
    rows = [json.loads(line) for line in (folder / "can.jsonl").read_text().splitlines()]
    def series(kind):
        result = [r for r in rows if r.get("node") == node and r.get("kind") == kind]
        assert result, (folder, node, kind)
        times = np.array([r["t"] for r in result])
        assert np.all(np.diff(times) >= 0), (folder, kind, "unordered timestamps")
        return times, result
    tc, command = series("command")
    tf, feedback = series("feedback")
    td, diagnostic = series("control_diag")
    ti, current = series("get_current")
    amps = np.array([r["value"]["real32"]["value"] for r in current])
    ctrl = np.array([r["value"]["real32"]["value"] for r in diagnostic])
    # J1/J2: position_direction=+1, current_direction=-1. Native Iq sign
    # differs from public output-torque sign. Field 21 is read later outside
    # the snapshot lock, so it need not equal -field 4 sample-for-sample.
    target, measured = -amps[:, 5], -amps[:, 4]
    error = measured-target
    voltage = np.linalg.norm(amps[:, 6:8], axis=1)
    ci, fi = nearest(tc, ti), nearest(tf, ti)
    q_error = np.array([command[i]["q"] for i in ci])-np.array([feedback[i]["q"] for i in fi])
    v_error = np.array([command[i]["v"] for i in ci])-np.array([feedback[i]["v"] for i in fi])
    p = np.array([command[i]["kp"] for i in ci])*q_error / (.011*gear)
    d = np.array([command[i]["kd"] for i in ci])*v_error / (.011*gear)
    ff = np.array([command[i].get("torque", 0) for i in ci]) / (.011*gear)
    residual = target-p-d-ff
    slope, offset = np.linalg.lstsq(np.column_stack([p+d+ff, np.ones(len(p))]), target, rcond=None)[0]
    def rate(t):
        return {"median_dt_s": float(np.median(np.diff(t))), "max_dt_s": float(np.max(np.diff(t)))}
    metrics = {
        "folder": str(folder.relative_to(ROOT)), "node": node, "scope": "entire capture, including before/after action",
        "samples": len(current), "current_diagnostic_timing": rate(ti), "command_timing": rate(tc),
        "packet_gains": sorted({(r["kp"], r["kd"]) for r in command}),
        "mode_values": np.unique(ctrl[:, 0]).tolist(), "max_command_age_ms": float(max(ctrl[:, 6])),
        "live_field21_vs_snapshot_public_iq_rms_a": float(np.sqrt(np.mean((amps[:,21]-measured)**2))),
        "iq_target_peak_a": float(max(abs(target))), "iq_measured_peak_a": float(max(abs(measured))),
        "iq_error_rms_a": float(np.sqrt(np.mean(error**2))), "iq_error_p95_a": float(np.percentile(abs(error), 95)),
        "iq_error_max_a": float(max(abs(error))), "id_peak_a": float(max(abs(amps[:, 3]))),
        "vdq_norm_peak_v": float(max(voltage)), "samples_at_2v_ceiling": int(np.sum(voltage >= 1.999)),
        "current_limit_values_a": np.unique(amps[:, 17]).tolist(),
        "pwm_on_values": np.unique(amps[:, 19]).tolist(), "current_fault_values": np.unique(amps[:, 24]).tolist(),
        "approx_pd_residual_a_p0_p25_p50_p75_p100": np.percentile(residual, [0,25,50,75,100]).tolist(),
        "approx_pd_fit_slope": float(slope), "approx_pd_fit_offset_a": float(offset),
        "feedback_alignment_gap_s_p50_p95_max": np.percentile(abs(tf[fi]-ti), [50,95,100]).tolist(),
        "interpretation_limit": "Residual includes unlogged load compensation, stale native q/v between app updates, and receive-time skew; it is not a measured I-integrator.",
    }
    (output / (folder.name+".json")).write_text(json.dumps(metrics, indent=2)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    origin = tc[0]
    axes[0].plot(tc-origin, np.degrees([r["q"] for r in command]), label="CAN q_ref")
    axes[0].plot(td-origin, np.degrees(ctrl[:,1]), label="Firmware target", alpha=.65)
    axes[0].plot(tf-origin, np.degrees([r["q"] for r in feedback]), label="Fused measured q")
    axes[0].set_ylabel("Output angle, deg"); axes[0].legend()
    axes[1].plot(ti-origin, target, label="Iq target, public sign")
    axes[1].plot(ti-origin, measured, label="Iq measured, public sign", alpha=.7)
    axes[1].set_ylabel("Current, A"); axes[1].legend()
    axes[2].plot(ti-origin, error); axes[2].set_ylabel("Measured - target, A")
    axes[3].plot(ti-origin, voltage, label="sqrt(Vd²+Vq²)")
    axes[3].axhline(2, color="red", linestyle="--", label="2 V ceiling")
    axes[3].set_ylabel("Voltage vector, V"); axes[3].set_xlabel("Host receive time, s"); axes[3].legend()
    for ax in axes: ax.grid(alpha=.25)
    fig.suptitle(f"{folder.name}: coherent dq pairs, asynchronous position/commands")
    fig.tight_layout();fig.savefig(output/(folder.name+"-current.png"), dpi=130);plt.close(fig)
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"build/tracking-audit-20261116")
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    result = [audit(ROOT/"build"/name, node, gear, args.output) for name,node,gear in CASES]
    (args.output/"summary.json").write_text(json.dumps(result, indent=2)+"\n")
    for r in result:
        print(f'{r["folder"]}: Iq error RMS={r["iq_error_rms_a"]:.4f} A, p95={r["iq_error_p95_a"]:.4f} A; Vdq max={r["vdq_norm_peak_v"]:.3f} V')


if __name__ == "__main__":
    main()
