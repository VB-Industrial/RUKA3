"""Offline comparison of J4 encoder records; never connects to hardware."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "build/g474-joint4/diagnostics"


def load(name):
    rows = [json.loads(line) for line in (DEST / name).read_text().splitlines()
            if line.startswith("{")]
    values = np.array([r["value"]["real32"]["value"] for r in rows
                       if r.get("register") == "get_pos"])
    # Turns are maintained in the control ISR, not inferred from CAN samples.
    motor_turns = values[:, 3] + values[:, 0] / 16384
    output_turns = values[:, 4] + values[:, 1] / 16384
    return motor_turns, output_turns, values


def full_circle_plot(forward, reverse):
    continuation = load("plus04-5s.jsonl")
    m0, o0 = forward[0][0], forward[1][0]
    target = o0 + 1
    for i in range(1, len(continuation[0])):
        oa, ob = continuation[1][i-1:i+1]
        if oa <= target <= ob and ob > oa:
            fraction = (target - oa) / (ob - oa)
            m1 = continuation[0][i-1] + fraction * (continuation[0][i] - continuation[0][i-1])
            break
    else:
        raise ValueError("No complete external encoder cycle in the records")
    ratio = float(m1 - m0)
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
    residuals = []
    for data, label in [(forward, "Forward, 2 s"), (reverse, "Reverse, 1 s"),
                        (continuation, "Forward, 5 s")]:
        x = (data[0] - m0) * 360 / 36
        y = (data[1] - o0) * 360
        axes[0].plot(x, y, ".-", markersize=2, label=label)
        axes[1].plot(x, y - x, ".-", markersize=2, label=label)
        if data is not reverse:
            mask = (data[0] >= m0) & (data[0] <= m1)
            residuals.extend((y - x)[mask])
    xmax = float((continuation[0][-1] - m0) * 360 / 36)
    axes[0].plot([0, xmax], [0, xmax], "--", color="0.4", label="Configured ratio 36:1")
    axes[0].set_title(f"J4: complete external encoder cycle; measured motor turns = {ratio:.4f}")
    axes[0].set_ylabel("External encoder change, degrees")
    axes[1].set_ylabel("External minus motor/36, degrees")
    axes[1].set_xlabel("Motor encoder change / configured gear ratio 36, degrees")
    axes[1].axhline(0, color="black", linewidth=0.7)
    axes[1].axvline(ratio * 360 / 36, color="0.5", linestyle=":", label="One external cycle")
    for ax in axes:
        ax.grid(alpha=0.25)
        ax.legend(fontsize=9)
    metrics = {
        "motor_turns_per_complete_external_cycle": ratio,
        "configured_gear_ratio": 36,
        "relative_difference_from_36_percent": (ratio / 36 - 1) * 100,
        "residual_min_deg_on_cycle": float(min(residuals)),
        "residual_max_deg_on_cycle": float(max(residuals)),
        "residual_peak_to_peak_deg_on_cycle": float(max(residuals) - min(residuals)),
        "five_second_position_samples": len(continuation[0]),
        "note": "Cycle assembled from continuous MCU counters across separate runs without reset; no calibration applied.",
    }
    fig.savefig(DEST / "encoder-full-circle.png", dpi=160)
    fig.savefig(DEST / "encoder-full-circle.svg")
    (DEST / "encoder-full-circle.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))


def main():
    forward = load("plus04-2s.jsonl")
    reverse = load("reverse04-1s-thirds.jsonl")
    m0, o0 = forward[0][0], forward[1][0]
    # gear_ratio=36 is only the configured reference; also fit an unrestricted
    # line so a wrong constant ratio cannot masquerade as nonlinearity.
    x = (forward[0] - m0) * 360 / 36
    y = (forward[1] - o0) * 360
    xr = (reverse[0] - m0) * 360 / 36
    yr = (reverse[1] - o0) * 360
    slope, intercept = np.polyfit(x, y, 1)
    residual = y - (slope * x + intercept)
    residual_r = yr - (slope * xr + intercept)
    # Match actual motor coordinates, without shifting reverse independently.
    mask = (xr >= x.min()) & (xr <= x.max())
    order = np.argsort(x)
    difference = yr[mask] - np.interp(xr[mask], x[order], y[order])
    metrics = {
        "forward_samples": len(x), "reverse_samples": len(xr),
        "configured_gear_ratio": 36,
        "best_linear_fit_ratio_over_partial_arc": float(36 / slope),
        "fit_intercept_output_degrees": float(intercept),
        "forward_residual_min_deg": float(residual.min()),
        "forward_residual_max_deg": float(residual.max()),
        "forward_residual_peak_to_peak_deg": float(np.ptp(residual)),
        "forward_residual_rms_deg": float(np.sqrt(np.mean(residual**2))),
        "common_motor_arc_samples": int(mask.sum()),
        "reverse_minus_forward_median_deg": float(np.median(difference)),
        "reverse_minus_forward_rms_deg": float(np.sqrt(np.mean(difference**2))),
        "reverse_minus_forward_max_abs_deg": float(np.max(np.abs(difference))),
        "note": "Relative encoder disagreement on a partial arc; not absolute encoder calibration.",
    }
    (DEST / "encoder-linearity.json").write_text(json.dumps(metrics, indent=2) + "\n")

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
    axes[0].plot(x, y, ".-", markersize=3, label="Forward +0.4 A, 2 s")
    axes[0].plot(xr, yr, ".-", markersize=3, label="Reverse -0.4 A, 1 s")
    span = np.array([min(x.min(), xr.min()), max(x.max(), xr.max())])
    axes[0].plot(span, span, "--", color="0.5", label="Configured ratio 36:1")
    axes[0].plot(span, slope * span + intercept, ":", color="black",
                 label=f"Best straight line: ratio {36 / slope:.2f}:1 (partial arc)")
    axes[0].set_ylabel("External encoder change, degrees")
    axes[0].set_title("J4: relative encoder linearity, existing records only")
    axes[0].legend(fontsize=9)
    axes[1].plot(x, residual, ".-", markersize=3, label="Forward")
    axes[1].plot(xr, residual_r, ".-", markersize=3, label="Reverse, same reference")
    axes[1].axhline(0, color="black", linewidth=0.7)
    axes[1].set_ylabel("Residual from best straight line, degrees")
    axes[1].set_xlabel("Motor encoder change / configured gear ratio 36, degrees")
    axes[1].legend()
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.savefig(DEST / "encoder-linearity.png", dpi=160)
    fig.savefig(DEST / "encoder-linearity.svg")
    print(json.dumps(metrics, indent=2))
    if (DEST / "plus04-5s.jsonl").exists():
        full_circle_plot(forward, reverse)


if __name__ == "__main__":
    main()
