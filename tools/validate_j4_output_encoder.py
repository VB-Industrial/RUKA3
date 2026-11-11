"""Analyze independent J4 verification runs; no hardware access."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DATA = Path(__file__).resolve().parents[1] / "build/g474-joint4/diagnostics"


def main():
    metrics = {}
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), constrained_layout=True)
    for name in ("forward", "reverse"):
        rows = [json.loads(s) for s in (DATA / f"cal-validate-{name}.jsonl").read_text().splitlines()
                if s.startswith("{")]
        samples = [r["value"]["real32"]["value"] for r in rows if r.get("register") == "get_pos"]
        if not samples:
            metrics[name] = {"tested": False, "reason": "No recorded position samples"}
            continue
        p = np.array(samples)
        if p.shape[1] != 15 or not np.all(p[:, 14] == 1):
            raise ValueError("Output calibration must be active in all verification samples")
        raw = (p[:, 4] + p[:, 1] / 16384) * 2 * np.pi
        corrected = np.unwrap(p[:, 13])
        reference = p[:, 5]
        residual_raw = np.rad2deg(raw - reference)
        residual_corrected = np.rad2deg(corrected - reference)
        # Encoder and motor reference have different arbitrary coordinate zeros.
        residual_raw -= np.median(residual_raw)
        residual_corrected -= np.median(residual_corrected)
        motor_delta = (p[-1, 3] - p[0, 3]) + (p[-1, 0] - p[0, 0]) / 16384
        x = np.rad2deg(reference - reference[0])
        metrics[name] = {
            "samples": len(p), "motor_output_revolutions_at_gear36": float(motor_delta / 36),
            "raw_rms_deg": float(np.sqrt(np.mean(residual_raw**2))),
            "corrected_rms_deg": float(np.sqrt(np.mean(residual_corrected**2))),
            "corrected_p95_abs_deg": float(np.percentile(np.abs(residual_corrected), 95)),
            "corrected_max_abs_deg": float(np.max(np.abs(residual_corrected))),
            "raw_peak_to_peak_deg": float(np.ptp(residual_raw)),
            "corrected_peak_to_peak_deg": float(np.ptp(residual_corrected)),
        }
        axes[0].plot(x, residual_raw, label=f"{name}: raw")
        axes[0].plot(x, residual_corrected, label=f"{name}: corrected")
        axes[1].plot(x, residual_corrected, ".-", markersize=2, label=name)
    axes[0].set_title("J4: recorded independent verification at 0.4 A, RUKA2-style output LUT")
    axes[0].set_ylabel("Encoder disagreement, degrees")
    axes[1].set_ylabel("Corrected disagreement, degrees")
    for ax in axes:
        ax.set_xlabel("Motor-side output angle change, degrees")
        ax.grid(alpha=0.25)
        ax.legend()
        ax.axhline(0, color="black", linewidth=0.7)
    fig.savefig(DATA / "output-calibration-validation.png", dpi=160)
    fig.savefig(DATA / "output-calibration-validation.svg")
    metrics["note"] = "Actual firmware unfiltered corrected angle vs motor angle; each coordinate offset removed by median. Independent runs, not training data."
    (DATA / "output-calibration-validation.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
