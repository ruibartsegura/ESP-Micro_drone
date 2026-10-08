#!/usr/bin/env python3
"""
Made by Rui B.S.

Analyzes the output of `idf.py monitor` (log_monitor.txt).

  - Removes the ANSI colour codes.
  - Separates the log by TAG (ATTITUDE, MICRO_ROS, SYSTEM, ...) into
    <out>/<TAG>.log, plus <out>/events.log with only the important lines.
  - Extracts every "name = value" / "name=value" of each TAG into
    <out>/<TAG>.csv (one row per control cycle).
  - Splits the run in phases (before / after the TAKEOFF service) and prints
    a summary per variable with min/max/mean and a sparkline.
  - Reconstructs the motor power with the same mix as attitude_controller.c
    (if the log has m0..m3 it uses those instead) and reports the saturation.
  - If matplotlib is installed it also saves <out>/plots.png.

Usage:
  python3 tools/analyze_log.py log_monitor.txt
  python3 tools/analyze_log.py log_monitor.txt -o log_analysis --from 19000 --to 30000
"""

import argparse
import csv
import math
import os
import re
import sys
from collections import defaultdict

# ------------------------------------------------------------
# Constants copied from attitude_controller.c / motors.c (sim)
# ------------------------------------------------------------
DEF_KP = 85.0
DEF_THROTTLE_BASE = 2387.0
DEF_MAX_POWER = 3000.0

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
LINE_RE = re.compile(r"^([IWEDV]) \((\d+)\) ([^:]+): ?(.*)$")
KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|nan|-?inf)")

EVENT_PATTERNS = [
    r"TAKEOFF", r"state", r"STATE", r"Fin test", r"Failed", r"ERROR",
    r"arm", r"Executor ready", r"Guru", r"abort", r"panic", r"rst:",
]
EVENT_RE = re.compile("|".join(EVENT_PATTERNS))

SPARK = "▁▂▃▄▅▆▇█"


class C:
    on = sys.stdout.isatty()
    @staticmethod
    def c(code, s):
        return f"\x1b[{code}m{s}\x1b[0m" if C.on else s
    red = staticmethod(lambda s: C.c("31", s))
    green = staticmethod(lambda s: C.c("32", s))
    yellow = staticmethod(lambda s: C.c("33", s))
    blue = staticmethod(lambda s: C.c("34", s))
    bold = staticmethod(lambda s: C.c("1", s))
    dim = staticmethod(lambda s: C.c("2", s))


# ------------------------------------------------------------
#                         Parsing
# ------------------------------------------------------------
def parse(path, t_from=None, t_to=None):
    """Returns (lines_by_tag, rows_by_tag, events, other_lines)."""
    lines_by_tag = defaultdict(list)
    rows_by_tag = defaultdict(list)   # tag -> list of dict(t=..., key=val...)
    events = []
    other = []

    with open(path, errors="replace") as f:
        for raw in f:
            line = ANSI_RE.sub("", raw).rstrip("\n")
            m = LINE_RE.match(line)
            if not m:
                if line.strip():
                    other.append(line)
                    if EVENT_RE.search(line):
                        events.append((None, "-", line))
                continue

            level, t, tag, msg = m.group(1), int(m.group(2)), m.group(3).strip(), m.group(4)
            if (t_from is not None and t < t_from) or (t_to is not None and t > t_to):
                continue

            lines_by_tag[tag].append(line)
            if level in "WE" or tag == "TUNING" or (EVENT_RE.search(msg) and tag != "ATTITUDE"):
                events.append((t, tag, line))

            kvs = KV_RE.findall(msg)
            if not kvs:
                continue
            rows = rows_by_tag[tag]
            # A new row starts when a key repeats (= next control cycle). The
            # timestamp is not used because the UART makes the lines of one
            # cycle spread over several ms.
            cur = rows[-1] if rows else None
            for k, v in kvs:
                if cur is None or k in cur:
                    cur = {"t": t}
                    rows.append(cur)
                cur[k] = float(v)

    return lines_by_tag, rows_by_tag, events, other


# ------------------------------------------------------------
#                       Motor model
# ------------------------------------------------------------
def add_motor_model(rows, kp, base, max_power, takeoff_t):
    """Adds pow_* and m0..m3 (before and after the clamp) to the ATTITUDE rows.

    If the firmware already logs m0..m3 these are respected. pow_h is only
    known if the log has pow_h or (h_t and z); otherwise it is marked as
    unknown and taken as 0 (it is small anyway: KP * 0.5 m = 42).
    """
    pow_h_known = False
    for r in rows:
        if not all(k in r for k in ("err_roll_rate", "err_pitch_rate", "err_yaw_rate")):
            continue
        r.setdefault("pow_roll", kp * r.get("err_roll_rate", 0.0))
        r.setdefault("pow_pitch", kp * r.get("err_pitch_rate", 0.0))
        r.setdefault("pow_yaw", kp * r.get("err_yaw_rate", 0.0))
        if "pow_h" not in r and "h_t" in r and "z" in r:
            r["pow_h"] = kp * (r["h_t"] - r["z"])
        if "pow_h" in r:
            pow_h_known = True
        ph = r.get("pow_h", 0.0)
        pr, pp, py = r["pow_roll"], r["pow_pitch"], r["pow_yaw"]
        raw = [
            base + ph + pr - pp - py,
            base + ph - pr - pp + py,
            base + ph + pr + pp + py,
            base + ph - pr + pp - py,
        ]
        armed = takeoff_t is not None and r["t"] >= takeoff_t
        for i, v in enumerate(raw):
            r.setdefault(f"m{i}_raw", v)
            if f"m{i}" not in r:
                if not armed:
                    r[f"m{i}"] = 0.0
                elif not (v > 0.0):
                    r[f"m{i}"] = 0.0
                else:
                    r[f"m{i}"] = min(v, max_power)
        r["m_sum"] = sum(r[f"m{i}"] for i in range(4))
    return pow_h_known


# ------------------------------------------------------------
#                        Reporting
# ------------------------------------------------------------
def sparkline(values, width=60):
    vals = [v for v in values if not math.isnan(v)]
    if not vals:
        return ""
    if len(values) > width:   # bucket average
        step = len(values) / width
        values = [
            sum(values[int(i * step):int((i + 1) * step)]) / max(1, len(values[int(i * step):int((i + 1) * step)]))
            for i in range(width)
        ]
    lo, hi = min(values), max(values)
    span = hi - lo or 1.0
    return "".join(SPARK[min(7, int((v - lo) / span * 7.999))] for v in values)


def stats(values):
    n = len(values)
    if n == 0:
        return None
    mean = sum(values) / n
    return min(values), max(values), mean, values[-1]


def fmt(v):
    if abs(v) >= 1000:
        return f"{v:10.0f}"
    return f"{v:10.3f}"


def print_phase(name, rows, keys, max_power):
    if not rows:
        return
    t0, t1 = rows[0]["t"], rows[-1]["t"]
    print()
    per = (t1 - t0) / (len(rows) - 1) if len(rows) > 1 else 0
    print(C.bold(C.blue(f"── {name}  [{t0} ms → {t1} ms, {len(rows)} muestras, "
                        f"periodo medio {per:.1f} ms] " + "─" * 10)))
    print(C.dim(f"  {'variable':<16}{'min':>10}{'max':>10}{'media':>10}{'último':>10}   evolución"))
    for k in keys:
        vals = [r[k] for r in rows if k in r]
        st = stats(vals)
        if st is None:
            continue
        lo, hi, mean, last = st
        line = f"  {k:<16}{fmt(lo)}{fmt(hi)}{fmt(mean)}{fmt(last)}   {sparkline(vals)}"
        if k.startswith("m") and k[1:].isdigit() and (hi >= max_power or lo <= 0):
            line = C.yellow(line)
        print(line)

    if any("m0" in r for r in rows):
        rows = [r for r in rows if "m0" in r]
        n = len(rows)
        print()
        for i in range(4):
            sat_hi = sum(1 for r in rows if r.get(f"m{i}", 0) >= max_power)
            sat_lo = sum(1 for r in rows if r.get(f"m{i}", 0) <= 0)
            msg = f"  motor {i}: saturado a MAX {100*sat_hi/n:5.1f}%   a 0 {100*sat_lo/n:5.1f}%"
            print(C.red(msg) if sat_hi + sat_lo > 0.3 * n else msg)


def diagnose(rows, takeoff_t, max_power, base, pow_h_known):
    print()
    print(C.bold("── Diagnóstico automático " + "─" * 40))
    post = [r for r in rows if takeoff_t is not None and r["t"] >= takeoff_t]
    pre = [r for r in rows if takeoff_t is None or r["t"] < takeoff_t]

    if takeoff_t is None:
        print(C.yellow("  · No se encontró TAKEOFF en el log → los motores nunca se armaron."))

    if pre:
        e = [abs(r.get("err_roll", 0)) for r in pre]
        if max(e) > 2:
            print(C.red(f"  · Antes del despegue (en el suelo) |err_roll| llega a {max(e):.1f}°: "
                        "la estimación de roll/pitch deriva sin moverse el dron."))

    if post:
        for k in ("pow_roll", "pow_pitch", "pow_yaw"):
            v = max(abs(r.get(k, 0)) for r in post)
            if v > 0.25 * base:
                print(C.red(f"  · max |{k}| = {v:.0f}, comparable o mayor que throttle_base ({base:.0f}) "
                            "→ el lazo de actitud satura los motores."))
        post = [r for r in post if "m0" in r]
        n = max(1, len(post))
        diff = sum(1 for r in post
                   if max(r.get(f"m{i}", 0) for i in range(4)) >= max_power
                   and min(r.get(f"m{i}", 0) for i in range(4)) <= 0)
        if diff:
            print(C.red(f"  · En {100*diff/n:.0f}% de los ciclos tras el despegue hay motores a MAX y "
                        "otros a 0 a la vez: el empuje total no es el de hover."))
        sums = [r["m_sum"] for r in post if "m_sum" in r]
        mean_sum = sum(sums) / max(1, len(sums))
        print(f"  · Empuje total medio tras el despegue: {mean_sum:.0f} "
              f"(4·throttle_base = {4*base:.0f})")
        if not pow_h_known:
            print(C.yellow("  · pow_h no está en el log (se ha supuesto 0). Añade la línea CTRL "
                           "sugerida al firmware para ver altura y motores reales."))


# ------------------------------------------------------------
#                          Output
# ------------------------------------------------------------
def write_outputs(out, lines_by_tag, rows_by_tag, events, other):
    os.makedirs(out, exist_ok=True)
    for tag, lines in lines_by_tag.items():
        safe = re.sub(r"[^A-Za-z0-9_.-]", "_", tag)
        with open(os.path.join(out, f"{safe}.log"), "w") as f:
            f.write("\n".join(lines) + "\n")
    if other:
        with open(os.path.join(out, "_other.log"), "w") as f:
            f.write("\n".join(other) + "\n")
    with open(os.path.join(out, "events.log"), "w") as f:
        for _, _, line in events:
            f.write(line + "\n")
    for tag, rows in rows_by_tag.items():
        if not rows:
            continue
        keys = ["t"] + sorted({k for r in rows for k in r if k != "t"})
        safe = re.sub(r"[^A-Za-z0-9_.-]", "_", tag)
        with open(os.path.join(out, f"{safe}.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)


def plot(out, rows, events, max_power):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(C.dim("\n(matplotlib no instalado: sin gráficas. `pip install matplotlib`)"))
        return

    t = [r["t"] / 1000.0 for r in rows]
    groups = [
        ("Ángulo (err, °)", ["err_roll", "err_pitch"]),
        ("Rate (err, °/s)", ["err_roll_rate", "err_pitch_rate", "err_yaw_rate"]),
        ("Contribución PID", ["pow_roll", "pow_pitch", "pow_yaw", "pow_h"]),
        ("Motores (tras clamp)", ["m0", "m1", "m2", "m3"]),
    ]
    groups = [(n, [k for k in ks if any(k in r for r in rows)]) for n, ks in groups]
    groups = [g for g in groups if g[1]]
    fig, axes = plt.subplots(len(groups), 1, sharex=True, figsize=(14, 3 * len(groups)))
    if len(groups) == 1:
        axes = [axes]
    for ax, (name, keys) in zip(axes, groups):
        for k in keys:
            ax.plot(t, [r.get(k, float("nan")) for r in rows], label=k, lw=0.8)
        ax.set_ylabel(name)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
        for et, tag, line in events:
            if et is not None:
                ax.axvline(et / 1000.0, color="k", ls="--", lw=0.6, alpha=0.5)
        if name.startswith("Motores"):
            ax.axhline(max_power, color="r", ls=":", lw=0.8)
    axes[-1].set_xlabel("t (s)")
    fig.tight_layout()
    p = os.path.join(out, "plots.png")
    fig.savefig(p, dpi=110)
    print(C.green(f"\nGráfica: {p}"))


# ------------------------------------------------------------
#                           Main
# ------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="log de idf.py monitor")
    ap.add_argument("-o", "--out", default=None, help="carpeta de salida (por defecto <log>_analysis)")
    ap.add_argument("--from", dest="t_from", type=int, help="ms inicial")
    ap.add_argument("--to", dest="t_to", type=int, help="ms final")
    ap.add_argument("--kp", type=float, default=DEF_KP)
    ap.add_argument("--base", type=float, default=DEF_THROTTLE_BASE, help="throttle_base")
    ap.add_argument("--max-power", type=float, default=DEF_MAX_POWER)
    ap.add_argument("--no-color", action="store_true")
    a = ap.parse_args()
    if a.no_color:
        C.on = False

    out = a.out or os.path.splitext(a.log)[0] + "_analysis"
    lines_by_tag, rows_by_tag, events, other = parse(a.log, a.t_from, a.t_to)

    takeoff_t = next((t for t, tag, l in events if t is not None and "TAKEOFF: accepted" in l), None)
    att = rows_by_tag.get("ATTITUDE", [])
    pow_h_known = add_motor_model(att, a.kp, a.base, a.max_power, takeoff_t)

    write_outputs(out, lines_by_tag, rows_by_tag, events, other)

    # ---- header
    print(C.bold(f"Log: {a.log}"))
    print("Líneas por TAG: " + ", ".join(f"{k}={len(v)}" for k, v in
                                         sorted(lines_by_tag.items(), key=lambda kv: -len(kv[1]))))
    if len(att) > 1:
        dts = [b["t"] - a_["t"] for a_, b in zip(att, att[1:])]
        print(f"Ciclos ATTITUDE: {len(att)}  periodo medio {sum(dts)/len(dts):.1f} ms")

    # ---- events
    print()
    print(C.bold("── Eventos " + "─" * 55))
    for t, tag, line in events:
        col = C.red if (" E " in line[:3] or "Failed" in line or "ERROR" in line) else C.green
        print("  " + col(line))

    # ---- other tags with numeric values (e.g. MICRO_ROS imu_recv)
    for tag, rows in rows_by_tag.items():
        if tag == "ATTITUDE" or not rows:
            continue
        keys = sorted({k for r in rows for k in r if k != "t"})
        print_phase(f"{tag}", rows, keys, a.max_power)
        imu_rows = [r for r in rows if "imu_recv" in r]
        if len(imu_rows) > 1:
            r0, r1 = imu_rows[0], imu_rows[-1]
            hz = (r1["imu_recv"] - r0["imu_recv"]) / ((r1["t"] - r0["t"]) / 1000.0)
            msg = f"  frecuencia IMU recibida ≈ {hz:.1f} Hz"
            print(C.red(msg + "  (muy baja para un lazo de 100 Hz)") if hz < 50 else msg)

    # ---- attitude per phase
    if att:
        keys = [k for k in ["err_roll", "err_pitch", "err_roll_rate", "err_pitch_rate", "err_yaw_rate",
                            "h_t", "z", "pow_h", "pow_roll", "pow_pitch", "pow_yaw",
                            "m0", "m1", "m2", "m3", "m_sum"]
                if any(k in r for r in att)]
        extra = sorted({k for r in att for k in r} - set(keys) - {"t"} - {f"m{i}_raw" for i in range(4)})
        keys += extra
        if takeoff_t is None:
            print_phase("ATTITUDE (todo)", att, keys, a.max_power)
        else:
            print_phase("ATTITUDE antes del TAKEOFF (motores desarmados)",
                        [r for r in att if r["t"] < takeoff_t], keys, a.max_power)
            post = [r for r in att if r["t"] >= takeoff_t]
            print_phase("ATTITUDE primer segundo tras TAKEOFF",
                        [r for r in post if r["t"] < takeoff_t + 1000], keys, a.max_power)
            print_phase("ATTITUDE tras TAKEOFF (completo)", post, keys, a.max_power)
        diagnose(att, takeoff_t, a.max_power, a.base, pow_h_known)
        plot(out, att, events, a.max_power)

    print(C.dim(f"\nFicheros separados en: {out}/  (<TAG>.log, <TAG>.csv, events.log)"))


if __name__ == "__main__":
    main()
