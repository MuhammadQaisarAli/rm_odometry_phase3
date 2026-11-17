#!/usr/bin/env python3
"""Recreate README endpoint figures from data/trial_endpoints.csv."""

import csv, json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "trial_endpoints.csv"
OUT = ROOT / "docs" / "figures"
OUT.mkdir(parents=True, exist_ok=True)
with DATA.open(newline="") as f:
    records = list(csv.DictReader(f))
t = np.array([int(r["trial"]) for r in records])
est = np.array([float(r["estimated_final_m"]) for r in records])
tape = np.array([float(r["tape_final_m"]) for r in records])
e = est - tape
assert np.allclose(e, [float(r["signed_error_m"]) for r in records])
stats = {
    "trials": len(t),
    "mean_signed_error_m": float(e.mean()),
    "mae_m": float(np.abs(e).mean()),
    "rmse_m": float(np.sqrt(np.mean(e**2))),
    "sample_variance_m2": float(np.var(e, ddof=1)),
    "sample_std_m": float(np.std(e, ddof=1)),
    "min_error_m": float(e.min()),
    "max_error_m": float(e.max()),
    "error_span_m": float(e.max() - e.min()),
    "sample_cov_estimate_tape_m2": float(np.cov(est, tape, ddof=1)[0, 1]),
}
(OUT / "statistics.json").write_text(json.dumps(stats, indent=2) + "\n")
plt.rcParams.update(
    {"font.size": 10, "axes.spines.top": False, "axes.spines.right": False}
)
fig, ax = plt.subplots(figsize=(8, 4.3))
ax.scatter(t, 100 * e, s=68, marker="o", color="#1d4ed8", zorder=3)
ax.axhline(0, color="#475569", ls="--", lw=1)
ax.axhline(
    100 * e.mean(),
    color="#dc2626",
    ls=":",
    lw=1.5,
    label=f"Mean = {100 * e.mean():.2f} cm",
)
ax.set(
    xticks=t,
    xlabel="Trial",
    ylabel="Estimated - tape (cm)",
    title="Final distance estimation error",
)
ax.legend()
ax.grid(alpha=0.17)
fig.tight_layout()
fig.savefig(OUT / "error_scatter.png", dpi=180)
plt.close(fig)
fig, ax = plt.subplots(figsize=(8, 4.3))
ax.plot(t, est, "o-", label="Estimated final distance", color="#1d4ed8", lw=1.7)
ax.plot(t, tape, "s--", label="Tape final distance", color="#ea580c", lw=1.7)
ax.set(
    xticks=t,
    xlabel="Trial",
    ylabel="Final distance (m)",
    title="Estimated versus measured final distance",
)
ax.grid(alpha=0.17)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "final_distance_comparison.png", dpi=180)
plt.close(fig)
fig, axs = plt.subplots(2, 4, figsize=(12, 5.5), sharey=True)
for ax, i, a, b in zip(axs.flat, t, est, tape):
    ax.scatter([0, 1], [a, b], s=65, c=["#1d4ed8", "#ea580c"])
    ax.plot([0, 1], [a, b], color="#94a3b8", lw=1)
    ax.set(
        xticks=[0, 1],
        xticklabels=["Est.", "Tape"],
        title=f"Trial {i} | error {a - b:+.3f} m",
        ylim=(0, 0.57),
    )
    ax.grid(axis="y", alpha=0.2)
for ax in axs[:, 0]:
    ax.set_ylabel("Distance (m)")
fig.suptitle("Final estimates and tape measurements by trial")
fig.tight_layout()
fig.savefig(OUT / "individual_trials.png", dpi=180)
plt.close(fig)
fig, ax = plt.subplots(figsize=(6, 5))
ax.scatter(tape, est, s=70, color="#1d4ed8")
ax.plot([0, 0.55], [0, 0.55], "--", color="#475569", label="Perfect agreement")
for i, x, y in zip(t, tape, est):
    ax.annotate(str(i), (x, y), xytext=(5, 5), textcoords="offset points", fontsize=9)
ax.set(
    xlim=(0, 0.56),
    ylim=(0, 0.56),
    xlabel="Tape final distance (m)",
    ylabel="Estimated final distance (m)",
    title="Estimate vs tape: agreement",
)
ax.grid(alpha=0.17)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "agreement.png", dpi=180)
plt.close(fig)
print(json.dumps(stats, indent=2))
