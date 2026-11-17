#!/usr/bin/env python3
"""Analyze physical trial measurements; never substitute target distance for ground truth."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--measurements", required=True)
    p.add_argument("--logs-dir", required=True)
    p.add_argument("--output-dir", required=True)
    a = p.parse_args()
    output = Path(a.output_dir).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    with Path(a.measurements).expanduser().open(newline="") as f:
        measurements = list(csv.DictReader(f))
    for m in measurements:
        if not m["actual_m"].strip():
            print(f"Skipping trial {m['trial']}: actual_m missing")
            continue
        actual = float(m["actual_m"])
        path = Path(a.logs_dir).expanduser() / m["log_csv"]
        if not path.is_file():
            print(f"Skipping trial {m['trial']}: log not found {path}")
            continue
        with path.open(newline="") as f:
            log = list(csv.DictReader(f))
        completed = [r for r in log if r["mode"] == "DONE"]
        if not completed:
            print(f"Skipping trial {m['trial']}: no DONE record")
            continue
        last = completed[-1]
        odom = float(last["odom_remaining_m"])
        corrected = float(last["corrected_remaining_m"])
        rows.append(
            {
                "trial": int(m["trial"]),
                "actual_m": actual,
                "odom_m": odom,
                "corrected_m": corrected,
                "odom_error_m": odom - actual,
                "corrected_error_m": corrected - actual,
            }
        )
    if not rows:
        raise SystemExit(
            "No completed trials with independent physical measurements. No plots created."
        )
    rows.sort(key=lambda r: r["trial"])
    with (output / "trial_results.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    oe = np.array([r["odom_error_m"] for r in rows])
    ce = np.array([r["corrected_error_m"] for r in rows])
    x = np.array([r["trial"] for r in rows])

    def summary(e):
        return {
            "mean_error_m": float(np.mean(e)),
            "mae_m": float(np.mean(np.abs(e))),
            "rmse_m": float(np.sqrt(np.mean(e**2))),
            "sample_variance_m2": float(np.var(e, ddof=1))
            if len(e) > 1
            else float("nan"),
            "std_m": float(np.std(e, ddof=1)) if len(e) > 1 else float("nan"),
            "min_error_m": float(np.min(e)),
            "max_error_m": float(np.max(e)),
        }

    import json

    result = {
        "n": len(rows),
        "odom": summary(oe),
        "corrected": summary(ce),
        "sample_error_covariance_m2": float(np.cov(oe, ce, ddof=1)[0, 1])
        if len(rows) > 1
        else None,
        "note": "Covariance is across trials, not a Kalman covariance matrix.",
    }
    (output / "statistics.json").write_text(json.dumps(result, indent=2))
    fig, ax = plt.subplots()
    ax.scatter(x - 0.08, 100 * oe, label="Odometry only")
    ax.scatter(x + 0.08, 100 * ce, label="Fixed-gain corrected")
    ax.axhline(0, color="gray", linewidth=1)
    ax.set(
        xlabel="Trial",
        ylabel="Estimated minus measured distance (cm)",
        title="Final distance error by trial",
    )
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "trial_error_scatter.png", dpi=180)
    plt.close(fig)
    fig, ax = plt.subplots()
    ax.plot(x, 100 * oe, "o-", label="Odometry only")
    ax.plot(x, 100 * ce, "s-", label="Fixed-gain corrected")
    ax.axhline(0, color="gray", linewidth=1)
    ax.set(
        xlabel="Trial", ylabel="Error (cm)", title="Trial-to-trial estimation errors"
    )
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "trial_error_comparison.png", dpi=180)
    plt.close(fig)
    for m in measurements:
        path = Path(a.logs_dir).expanduser() / m["log_csv"]
        if not path.exists():
            continue
        with path.open(newline="") as f:
            log = list(csv.DictReader(f))
        if not log:
            continue
        t0 = float(log[0]["stamp_s"])
        ts = [float(r["stamp_s"]) - t0 for r in log]
        fig, ax = plt.subplots()
        for key, label in [
            ("odom_remaining_m", "Odometry"),
            ("predicted_remaining_m", "Prediction"),
            ("camera_remaining_m", "Camera"),
            ("corrected_remaining_m", "Corrected"),
        ]:
            ax.plot(ts, [float(r[key]) for r in log], label=label)
        ax.set(
            xlabel="Time (s)",
            ylabel="Remaining distance (m)",
            title=f"Trial {m['trial']}: prediction and correction",
        )
        ax.legend()
        fig.tight_layout()
        fig.savefig(output / f"trial_{int(m['trial']):02d}_history.png", dpi=180)
        plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f"Saved results to {output}")


if __name__ == "__main__":
    main()
