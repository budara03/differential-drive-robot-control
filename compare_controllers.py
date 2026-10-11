#compare controllers for the report 
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

import ddr_core as core

OUT = "results"
os.makedirs(OUT, exist_ok=True)

# --- plot style -------------------------------------------------------------
# Each controller gets its own colour AND its own line style / marker, so the
# curves are still distinguishable in a black-and-white printout.
COLORS = {"P": "#e11d48", "PD": "#7c3aed", "PID": "#059669"}      # rose / violet / emerald
STYLES = {"P": "--", "PD": "-.", "PID": "-"}
LABELS = {"P": "P  (Kp=3)", "PD": "PD (Kp=3, Kd=1)", "PID": "PD + PID heading"}
SWEEP_CMAP = matplotlib.colors.LinearSegmentedColormap.from_list(
    "sweep", ["#0ea5e9", "#6366f1", "#c026d3", "#f97316"])         # sky -> indigo -> magenta -> orange
WIND_COLORS = ("#f97316", "#0369a1")                               # orange / deep blue

plt.rcParams.update({
    "figure.facecolor": "white",
    "axes.facecolor": "#f8fafc",
    "axes.edgecolor": "#94a3b8",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "grid.color": "#cbd5e1",
    "grid.linestyle": ":",
    "font.family": "DejaVu Sans",
})

SCENARIOS = {
    "A": ((100.0, 300.0, 0.0), (400.0, 400.0)), 
    "B": ((400.0, 300.0, 0.0), (150.0, 150.0)), 
    "C": ((100.0, 300.0, 0.0), (600.0, 400.0)),
}

def run_with(mode, scenario, kd=None, ki=None, i_zone_deg=None):
    old_pd = core.PDDistance.__init__.__defaults__
    old_pid = core.PIDHeading.__init__.__defaults__
    try:
        if kd is not None:
            kp, _, vmax, fa = old_pd
            core.PDDistance.__init__.__defaults__ = (kp, kd, vmax, fa)
        if ki is not None or i_zone_deg is not None:
            kp, ki0, kd0, il, om, zone0 = old_pid
            zone = math.radians(i_zone_deg) if i_zone_deg is not None else zone0
            core.PIDHeading.__init__.__defaults__ = (kp, ki if ki is not None else ki0,
                                                     kd0, il, om, zone)
        start, target = SCENARIOS[scenario]
        return core.run_simulation(mode, start, target, t_max=20.0)
    finally:
        core.PDDistance.__init__.__defaults__ = old_pd
        core.PIDHeading.__init__.__defaults__ = old_pid


def style(ax, title, xlabel, ylabel):
    ax.set_title(title, fontsize=10, fontweight="bold", loc="left", color="#0f172a")
    ax.set_xlabel(xlabel, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.grid(alpha=0.8)
    ax.tick_params(labelsize=8)

def figure_comparison(scn, fname, title):
    fig, axs = plt.subplots(2, 2, figsize=(9.5, 6.4))
    results = {}
    for mode in ("P", "PD", "PID"):
        log, met = run_with(mode, scn)
        results[mode] = (log, met)
        c = COLORS[mode]
        axs[0, 0].plot(log["t"], log["dist"], color=c, ls=STYLES[mode], lw=2.2, label=LABELS[mode])
        axs[0, 1].plot(log["t"], [math.degrees(h) for h in log["head"]], color=c, ls=STYLES[mode], lw=2.2, label=LABELS[mode])
        axs[1, 0].plot(log["t"], log["v"], color=c, ls=STYLES[mode], lw=2.2, label=LABELS[mode])
        axs[1, 1].plot(log["x"], log["y"], color=c, ls=STYLES[mode], lw=2.2, label=LABELS[mode])
    start, target = SCENARIOS[scn]
    axs[0, 0].axhline(core.STOP_DISTANCE, color="#64748b", ls=":", lw=1.2)
    style(axs[0, 0], "(a) Distance error", "time (s)", "distance error (px)")
    style(axs[0, 1], "(b) Heading error", "time (s)", "heading error (deg)")
    style(axs[1, 0], "(c) Linear velocity", "time (s)", "v (px/s)")
    style(axs[1, 1], "(d) Trajectory", "x (px)", "y (px)")
    axs[1, 1].plot(*target, "*", color="#f59e0b", mec="#0f172a", ms=15, mew=0.8, label="target")
    axs[1, 1].plot(start[0], start[1], "o", color="#0f172a", ms=7, label="start")
    axs[1, 1].invert_yaxis()           # pygame y axis points down
    axs[1, 1].set_aspect("equal", adjustable="datalim")
    axs[0, 0].legend(fontsize=8)
    axs[1, 1].legend(fontsize=7, loc="best")
    fig.suptitle(title, fontsize=11, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, fname), dpi=170)
    plt.close(fig)
    return results

def figure_tuning():
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.5))
    cmap = SWEEP_CMAP
    kds = [0.0, 0.3, 0.6, 1.0, 1.5]
    rows = []
    for i, kd in enumerate(kds):
        mode = "P" if kd == 0.0 else "PD"
        log, met = run_with("PD", "A", kd=kd)
        axs[0].plot(log["t"], log["dist"], color=cmap(i / (len(kds) - 1)), lw=2.2, label=f"Kd={kd}")
        rows.append(("Kd sweep (Kp=3)", f"Kd={kd}", met))
    style(axs[0], "(a) Distance: effect of Kd", "time (s)", "distance error (px)")
    axs[0].set_ylim(-5, 120)
    axs[0].set_xlim(1.0, 5.0)
    axs[0].legend(fontsize=7)

    kis = [0.0, 0.2, 0.8, 1.5]
    for i, ki in enumerate(kis):
        log, met = run_with("PID", "A", ki=ki)
        axs[1].plot(log["t"], [math.degrees(h) for h in log["head"]], color=cmap(i / (len(kis) - 1)), lw=2.2, label=f"Ki={ki}")
        rows.append(("Ki sweep (Kp=2, Kd=0.25)", f"Ki={ki}", met))
    style(axs[1], "(b) Heading: effect of Ki (scenario A)", "time (s)", "heading error (deg)")
    axs[1].set_ylim(-12, 25)
    axs[1].set_xlim(0, 3.2)
    axs[1].legend(fontsize=7)

    for i, ki in enumerate(kis):
        log, met = run_with("PID", "C", ki=ki)
        axs[2].plot(log["t"], [math.degrees(h) for h in log["head"]], color=cmap(i / (len(kis) - 1)), lw=2.2, label=f"Ki={ki}")
    style(axs[2], "(c) Heading: effect of Ki (scenario C)", "time (s)", "heading error (deg)")
    axs[2].set_ylim(-5, 14)
    axs[2].set_xlim(0, 4.2)
    axs[2].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_gain_tuning.png"), dpi=170)
    plt.close(fig)
    return rows

def figure_windup():
    fig, axs = plt.subplots(1, 2, figsize=(9.5, 3.6))
    cases = [("No integral zone (zone = 180 deg)", 180.0, WIND_COLORS[0], "--"),
             ("Integral zone = 25 deg", 25.0, WIND_COLORS[1], "-")]
    out = {}
    for label, zone, c, ls in cases:
        log, met = run_with("PID", "B", i_zone_deg=zone)
        axs[0].plot(log["t"], [math.degrees(h) for h in log["head"]], color=c, ls=ls, lw=2.2, label=label)
        axs[1].plot(log["x"], log["y"], color=c, ls=ls, lw=2.2, label=label)
        out[label] = met
    start, target = SCENARIOS["B"]
    axs[1].plot(*target, "*", color="#f59e0b", mec="#0f172a", ms=15, mew=0.8)
    axs[1].plot(start[0], start[1], "o", color="#0f172a", ms=7)
    axs[1].invert_yaxis()
    axs[1].set_aspect("equal", adjustable="datalim")
    style(axs[0], "(a) Heading error, target behind robot", "time (s)", "heading error (deg)")
    style(axs[1], "(b) Trajectory", "x (px)", "y (px)")
    axs[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_windup_demo.png"), dpi=170)
    plt.close(fig)
    return out

def render_frame(mode, scn, fname):
    log, met = run_with(mode, scn)
    start, target = SCENARIOS[scn]
    img = Image.new("RGB", (800, 600), (15, 23, 42))                # dark slate background
    d = ImageDraw.Draw(img)
    pts = list(zip(log["x"], log["y"]))
    rgb = tuple(int(COLORS[mode][i:i + 2], 16) for i in (1, 3, 5))  # trajectory = controller colour
    d.line(pts, fill=rgb, width=4)
    d.ellipse([target[0] - 10, target[1] - 10, target[0] + 10, target[1] + 10], fill=(245, 158, 11))
    d.ellipse([start[0] - 5, start[1] - 5, start[0] + 5, start[1] + 5], outline=(148, 163, 184), width=2)
    fx, fy = pts[-1]
    try:
        sprite = Image.open(os.path.join("asset", "robot.png")).convert("RGBA").resize((70, 70))
        th = math.atan2(log["y"][-1] - log["y"][-2], log["x"][-1] - log["x"][-2])
        sprite = sprite.rotate(-math.degrees(th), expand=True)
        img.paste(sprite, (int(fx - sprite.width / 2), int(fy - sprite.height / 2)), sprite)
    except Exception:
        d.ellipse([fx - 18, fy - 18, fx + 18, fy + 18], outline=(80, 160, 255), width=3)
    d.line([fx - 10, fy - 10, fx + 10, fy + 10], fill=(255, 220, 0), width=3)
    d.line([fx - 10, fy + 10, fx + 10, fy - 10], fill=(255, 220, 0), width=3)
    txt = (f"{core.CONTROLLER_MODES[mode][0]}   |   arrived t = {met['arrival_time']:.2f} s   "
           f"final error = {met['final_error']:.1f} px   overshoot = {met['overshoot']:.1f} px")
    d.text((14, 12), txt, fill=(235, 235, 235))
    img.save(os.path.join(OUT, fname))

if __name__ == "__main__":
    rows = []
    for scn, fname, title in (("A", "fig_controller_comparison.png",
                               "Scenario A: start (100,300), target (400,400)"),
                              ("B", "fig_turnaround_scenario.png",
                               "Scenario B: target behind the robot, start (400,300) -> target (150,150)"),
                              ("C", "fig_scenario_C.png",
                               "Scenario C: start (100,300), target (600,400)")):
        res = figure_comparison(scn, fname, title)
        for mode, (_, met) in res.items():
            rows.append((f"Scenario {scn}", mode, met))
        for mode in ("P", "PD", "PID"):
            if scn == "A":
                render_frame(mode, scn, f"frame_{mode}.png")

    rows += figure_tuning()
    wind = figure_windup()
    for k, met in wind.items():
        rows.append(("Windup demo (scenario B)", k, met))

    keys = ["arrival_time", "final_error", "overshoot", "peak_speed",
            "path_length", "max_cross_track", "mean_heading_deg"]
    with open(os.path.join(OUT, "metrics.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["group", "case"] + keys)
        for g, c, m in rows:
            w.writerow([g, c] + [f"{m[k]:.2f}" for k in keys])
    print(f"{'group':28s} {'case':34s}" + "".join(f"{k[:11]:>12s}" for k in keys))
    for g, c, m in rows:
        print(f"{g:28s} {c:34s}" + "".join(f"{m[k]:12.2f}" for k in keys))
