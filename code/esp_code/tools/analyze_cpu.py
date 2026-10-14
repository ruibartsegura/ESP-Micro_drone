#!/usr/bin/env python3
"""
Made by Rui B.S.

Analyzes the CPU log of the drone (CONFIG_CPU_STATS_ON, cpu_stats.c) after
the flight, not live. It reads the lines of the monitor:

  DIAG: CPU,<t_ms>,<window_us>,<task>,<prio>,<run_us>,<stack_free_B>
  DIAG: PERIOD,<t_ms>,<task>,<target_ms>,<n>,<avg_us>,<max_us>,<late>

and makes:
  - Table of CPU usage of every task (IDLE = free CPU) and its minimum
    free stack.
  - Table of the real period of the drone tasks (iterations lost =
    starvation: a task with more priority did not let it run).
  - Warnings: CPU almost full, stack almost full, late tasks, reboots and
    crashes of the log (Guru Meditation, abort, watchdog, stack overflow).
  - Graphs (with matplotlib): CPU of every task over time, real period vs
    target, iterations lost and free stack.
  - Everything in <log>_cpu/: cpu.csv, periods.csv, events.csv,
    summary.txt and the png.

Get the log (the menuconfig option "Diagnostics -> CPU usage and task
period log" has to be on):
  idf.py monitor | tee ~/uni/TFG/cpu_log.txt

Usage:
  python3 tools/analyze_cpu.py ~/uni/TFG/cpu_log.txt
  python3 tools/analyze_cpu.py log.txt --from 20 --to 60    # only 20-60 s
  python3 tools/analyze_cpu.py log.txt --boot 2             # only the 2nd boot
Graphs: sudo apt install python3-matplotlib
"""

import argparse
import csv
import os
import re

ANSI = re.compile(r"\x1b\[[0-9;]*m")
DIAG = re.compile(r"DIAG: (CPU|PERIOD),(.*)$")
ESP_TIME = re.compile(r"^[IWEDV] \((\d+)\)")

# Lines of the monitor that mean that something went wrong
EVENTS = [
    ("reinicio", re.compile(r"^rst:|ets_main\.c|ESP-ROM:")),
    ("panic", re.compile(r"Guru Meditation|abort\(\) was called|Backtrace:")),
    ("stack overflow", re.compile(r"[Ss]tack (overflow|canary)|stack_chk")),
    ("watchdog", re.compile(r"task_wdt|Interrupt wdt|TG\dWDT")),
    ("brownout", re.compile(r"[Bb]rownout")),
    ("micro-ROS", re.compile(r"spin_some error|session|agent", re.I)),
]

# Thresholds of the warnings
FREE_CPU_LOW = 10.0   # % of IDLE
STACK_LOW = 300       # bytes
RATE_LOW = 0.90       # real frequency / target frequency


def percentile(values, p):
    if not values:
        return 0.0
    v = sorted(values)
    k = (len(v) - 1) * p / 100.0
    i = int(k)
    return v[i] if i + 1 >= len(v) else v[i] + (v[i + 1] - v[i]) * (k - i)


def read_log(path):
    """Returns the CPU rows, the PERIOD rows and the events. A boot ends when
    the time of the drone goes back (or there is a reset line)."""
    cpu, periods, events = [], [], []
    boot, last_t = 1, -1   # last_t = -1: no data yet in this boot

    with open(path, errors="replace") as f:
        for n_line, raw in enumerate(f, 1):
            line = ANSI.sub("", raw).rstrip("\r\n")

            for kind, rx in EVENTS:
                if rx.search(line) and "DIAG:" not in line:
                    if kind == "reinicio" and last_t >= 0:
                        boot += 1     # the ESP32 starts again
                        last_t = -1
                    m = ESP_TIME.match(line)
                    t = int(m.group(1)) / 1000 if m else max(last_t, 0) / 1000
                    events.append({"boot": boot, "t": t, "line": n_line, "kind": kind,
                                   "text": line.strip()[:120]})
                    break

            m = DIAG.search(line)
            if not m:
                continue
            f_ = m.group(2).split(",")
            try:
                t_ms = int(f_[0])
            except ValueError:
                continue
            if 0 <= t_ms < last_t:
                boot += 1   # the time went back without a reset line
            last_t = t_ms

            try:
                if m.group(1) == "CPU" and len(f_) == 6:
                    window, run = int(f_[1]), int(f_[4])
                    cpu.append({"boot": boot, "t": t_ms / 1000, "task": f_[2], "prio": int(f_[3]),
                                "run_us": run, "window_us": window,
                                "cpu": 100.0 * run / window if window else 0.0,
                                "stack": int(f_[5])})
                elif m.group(1) == "PERIOD" and len(f_) == 7:
                    n, avg, mx, late = int(f_[3]), int(f_[4]), int(f_[5]), int(f_[6])
                    periods.append({"boot": boot, "t": t_ms / 1000, "task": f_[1],
                                    "target_ms": int(f_[2]), "n": n, "avg_ms": avg / 1000,
                                    "max_ms": mx / 1000, "late": late,
                                    "late_pct": 100.0 * late / n if n else 0.0})
            except ValueError:
                continue   # line cut by the monitor
    return cpu, periods, events


def add_rates(periods, cpu):
    # Real frequency of each window: iterations / length of the window
    windows = {}
    for r in cpu:
        windows[(r["boot"], r["t"])] = r["window_us"] / 1e6
    for r in periods:
        w = windows.get((r["boot"], r["t"]))
        r["rate_hz"] = r["n"] / w if w else (1000.0 / r["avg_ms"] if r["avg_ms"] else 0.0)
        r["target_hz"] = 1000.0 / r["target_ms"] if r["target_ms"] else 0.0


def group(rows, key="task"):
    g = {}
    for r in rows:
        g.setdefault(r[key], []).append(r)
    return g


def summary(cpu, periods, events, out_lines):
    def p(s=""):
        print(s)
        out_lines.append(s)

    if cpu:
        t0, t1 = min(r["t"] for r in cpu), max(r["t"] for r in cpu)
        n_win = len({r["t"] for r in cpu})
        p(f"Ventanas: {n_win}  ({t0:.1f} s - {t1:.1f} s)")

    # ---- CPU table
    p("\n── Uso de CPU por tarea " + "─" * 56)
    p("  tarea              prio |  media %   p95 %   máx % | stack libre mín (B)")
    rows = []
    for task, rs in group(cpu).items():
        c = [r["cpu"] for r in rs]
        rows.append((task, rs[-1]["prio"], sum(c) / len(c), percentile(c, 95), max(c),
                     min(r["stack"] for r in rs)))
    rows.sort(key=lambda x: (x[0] != "IDLE", -x[2]))
    for task, prio, mean, p95, mx, stack in rows:
        name = task + (" (libre)" if task == "IDLE" else "")
        p(f"  {name:<18} {prio:4d} | {mean:7.1f} {p95:7.1f} {mx:7.1f} | {stack:8d}")
    busy = sum(m for t, _, m, _, _, _ in rows if t != "IDLE")
    p(f"  {'total ocupada':<18}      | {busy:7.1f}")

    # ---- Period table
    if periods:
        p("\n── Periodo real de las tareas del dron " + "─" * 41)
        p("  tarea          objetivo | frec. real (Hz)  media  mín | periodo (ms) medio   máx"
          " | tarde (>2x)")
        for task, rs in group(periods).items():
            tgt = rs[0]["target_ms"]
            rates = [r["rate_hz"] for r in rs if r["n"]]
            avgs = [r["avg_ms"] for r in rs if r["n"]]
            late = sum(r["late"] for r in rs)
            n = sum(r["n"] for r in rs)
            p(f"  {task:<14} {tgt:5d} ms | {1000 / tgt:6.1f} Hz  {sum(rates) / max(len(rates), 1):6.1f}"
              f" {min(rates, default=0):5.1f} | {sum(avgs) / max(len(avgs), 1):13.1f}"
              f" {max(r['max_ms'] for r in rs):6.1f} | {late:5d} ({100 * late / max(n, 1):.1f} %)")

    # ---- Warnings
    p("\n── Avisos " + "─" * 70)
    warns = []
    idle = [r for r in cpu if r["task"] == "IDLE"]
    if idle:
        low = [r for r in idle if r["cpu"] < FREE_CPU_LOW]
        worst = min(idle, key=lambda r: r["cpu"])
        if low:
            warns.append(f"CPU casi llena: libre < {FREE_CPU_LOW:.0f} % en {len(low)} ventanas "
                         f"(mínimo {worst['cpu']:.1f} % a t={worst['t']:.1f} s)")
        else:
            warns.append(f"OK: CPU libre mínima {worst['cpu']:.1f} % (t={worst['t']:.1f} s)")
    for task, prio, mean, p95, mx, stack in rows:
        if stack < STACK_LOW:
            warns.append(f"Stack casi lleno: {task} con {stack} B libres (sube su stack en menuconfig)")
        if task != "IDLE" and mx > 50:
            warns.append(f"{task} llega a usar el {mx:.0f} % de la CPU (¿bucle sin bloquearse?)")
    for task, rs in group(periods).items():
        late = [r for r in rs if r["late"]]
        # uros_task: its period is the timeout of spin_some, going faster is normal
        slow = [r for r in rs if r["n"] and r["rate_hz"] < RATE_LOW * r["target_hz"]
                and task != "uros_task"]
        if late or slow:
            bad = sorted({r["t"] for r in late + slow})
            txt = (f"Hambruna: {task} en {len(bad)} ventanas (primera a t={bad[0]:.1f} s): "
                   f"periodo máx {max(r['max_ms'] for r in rs):.0f} ms para {rs[0]['target_ms']} ms")
            if slow:
                txt += f", frecuencia mín {min(r['rate_hz'] for r in slow):.1f} de {rs[0]['target_hz']:.1f} Hz"
            warns.append(txt)
    boots = sorted({r["boot"] for r in cpu} | {e["boot"] for e in events})
    if len(boots) > 1:
        warns.append(f"El log tiene {len(boots)} arranques del ESP32 (se ha reiniciado)")
    if cpu:
        t_end = max(r["t"] for r in cpu)
        warns.append(f"Último dato de CPU a t={t_end:.1f} s (si el dron siguió encendido "
                     "y no hay más, el ESP32 se colgó o se cortó el log)")
    for w in warns:
        p("  - " + w)

    if events:
        p("\n── Eventos del log " + "─" * 61)
        for e in events[:40]:
            p(f"  línea {e['line']:6d}  arranque {e['boot']}  t={e['t']:7.1f} s  [{e['kind']}] {e['text']}")
        if len(events) > 40:
            p(f"  ... {len(events) - 40} más en events.csv")


def plots(cpu, periods, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.ticker import ScalarFormatter
    except ImportError:
        print("\nSin gráficas: falta matplotlib (sudo apt install python3-matplotlib)")
        return []

    files = []
    by_task = group(cpu)
    times = sorted({r["t"] for r in cpu})

    # 1. CPU of every task over time (stacked), small tasks together
    if cpu:
        means = {t: sum(r["cpu"] for r in rs) / len(rs) for t, rs in by_task.items()}
        main = [t for t in sorted(means, key=means.get, reverse=True) if t != "IDLE" and means[t] >= 1.0]
        series = {t: {r["t"]: r["cpu"] for r in by_task[t]} for t in by_task}
        stack_y = [[series[t].get(x, 0.0) for x in times] for t in main]
        others = [sum(series[t].get(x, 0.0) for t in by_task if t not in main and t != "IDLE")
                  for x in times]
        fig, ax = plt.subplots(figsize=(13, 6))
        ax.stackplot(times, *stack_y, others, labels=main + ["otras (<1 %)"], alpha=0.85)
        if "IDLE" in series:
            ax.plot(times, [100 - series["IDLE"].get(x, 0.0) for x in times], "k--", lw=1,
                    label="ocupada total (100 - IDLE)")
        ax.axhline(100 - FREE_CPU_LOW, color="red", lw=0.8, ls=":", label=f"{FREE_CPU_LOW:.0f} % libre")
        ax.set_ylim(0, 105)
        ax.set_xlabel("t (s desde el arranque)")
        ax.set_ylabel("CPU (%)")
        ax.set_title("Uso de CPU por tarea (apilado)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.01, 1))
        fig.tight_layout()
        files.append(os.path.join(out, "cpu_tareas.png"))
        fig.savefig(files[-1], dpi=120)
        plt.close(fig)

        # 2. Mean and max CPU of every task (bars)
        order = sorted(means, key=means.get, reverse=True)
        fig, ax = plt.subplots(figsize=(13, 5))
        x = range(len(order))
        ax.bar(x, [max(r["cpu"] for r in by_task[t]) for t in order], color="tab:red", alpha=0.35,
               label="máx")
        ax.bar(x, [means[t] for t in order], color="tab:blue", label="media")
        ax.set_xticks(list(x))
        ax.set_xticklabels([f"{t}\n(p{by_task[t][-1]['prio']})" for t in order], rotation=45,
                           ha="right", fontsize=7)
        ax.set_ylabel("CPU (%)")
        ax.set_title("CPU media y máxima por tarea (IDLE = libre, pN = prioridad)")
        ax.grid(alpha=0.3, axis="y")
        ax.legend()
        fig.tight_layout()
        files.append(os.path.join(out, "cpu_resumen.png"))
        fig.savefig(files[-1], dpi=120)
        plt.close(fig)

        # 3. Free stack over time
        fig, ax = plt.subplots(figsize=(13, 5))
        for t in order:
            rs = by_task[t]
            ax.plot([r["t"] for r in rs], [r["stack"] for r in rs], ".-", ms=3, label=t)
        ax.axhline(STACK_LOW, color="red", lw=0.8, ls=":", label=f"{STACK_LOW} B")
        ax.set_yscale("log")
        ax.yaxis.set_major_formatter(ScalarFormatter())
        ax.set_xlabel("t (s)")
        ax.set_ylabel("stack libre mínimo (B)")
        ax.set_title("Stack libre de cada tarea (mínimo histórico)")
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.01, 1))
        fig.tight_layout()
        files.append(os.path.join(out, "stack.png"))
        fig.savefig(files[-1], dpi=120)
        plt.close(fig)

    # 4. Real period of the drone tasks: one row per task
    if periods:
        by_p = group(periods)
        fig, axes = plt.subplots(len(by_p), 1, figsize=(13, 2.6 * len(by_p)), sharex=True,
                                 squeeze=False)
        for ax, (task, rs) in zip(axes[:, 0], by_p.items()):
            t = [r["t"] for r in rs]
            tgt = rs[0]["target_ms"]
            ax.plot(t, [r["max_ms"] for r in rs], ".-", color="tab:red", label="máx")
            ax.plot(t, [r["avg_ms"] for r in rs], ".-", color="tab:blue", label="medio")
            ax.axhline(tgt, color="green", lw=1, label=f"objetivo {tgt} ms")
            ax.axhline(2 * tgt, color="red", lw=0.8, ls=":", label="2 x objetivo (tarde)")
            late = [(r["t"], r["late"]) for r in rs if r["late"]]
            if late:
                ax2 = ax.twinx()
                ax2.bar([a for a, _ in late], [b for _, b in late], width=0.6, color="orange",
                        alpha=0.5)
                ax2.set_ylabel("iter. tarde", fontsize=7, color="orange")
            ax.set_title(f"{task}", fontsize=9, loc="left")
            ax.set_ylabel("periodo (ms)")
            ax.set_ylim(0, 1.1 * max(2 * tgt, max(r["max_ms"] for r in rs)))
            ax.grid(alpha=0.3)
            ax.legend(fontsize=6, loc="upper right")
        axes[-1, 0].set_xlabel("t (s)")
        fig.suptitle("Periodo real entre iteraciones (barras naranjas = iteraciones que llegan tarde)")
        fig.tight_layout()
        files.append(os.path.join(out, "periodos.png"))
        fig.savefig(files[-1], dpi=120)
        plt.close(fig)
    return files


def write_csv(path, rows, keys):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="salida del monitor guardada (idf.py monitor | tee log.txt)")
    ap.add_argument("--boot", type=int, help="analizar solo este arranque (1, 2, ...)")
    ap.add_argument("--from", dest="t_from", type=float, help="desde t (s)")
    ap.add_argument("--to", dest="t_to", type=float, help="hasta t (s)")
    a = ap.parse_args()

    log = os.path.expanduser(a.log)
    cpu, periods, events = read_log(log)
    if not cpu and not periods:
        print("No hay líneas 'DIAG: CPU' ni 'DIAG: PERIOD' en el log.\n"
              "Activa menuconfig -> X-WING Config -> Diagnostics -> CPU usage and task period log")
        return
    add_rates(periods, cpu)

    boots = sorted({r["boot"] for r in cpu})
    if a.boot is None and len(boots) > 1:
        a.boot = boots[-1]
        print(f"El log tiene {len(boots)} arranques: analizo el último ({a.boot}), usa --boot N para otro")

    def keep(r):
        return ((a.boot is None or r["boot"] == a.boot)
                and (a.t_from is None or r["t"] >= a.t_from)
                and (a.t_to is None or r["t"] <= a.t_to))
    cpu = [r for r in cpu if keep(r)]
    periods = [r for r in periods if keep(r)]
    events = [e for e in events if a.boot is None or e["boot"] == a.boot]

    out = os.path.splitext(log)[0] + "_cpu"
    os.makedirs(out, exist_ok=True)
    lines = []
    summary(cpu, periods, events, lines)

    write_csv(os.path.join(out, "cpu.csv"), cpu,
              ["boot", "t", "task", "prio", "cpu", "run_us", "window_us", "stack"])
    write_csv(os.path.join(out, "periods.csv"), periods,
              ["boot", "t", "task", "target_ms", "target_hz", "rate_hz", "n", "avg_ms", "max_ms",
               "late", "late_pct"])
    write_csv(os.path.join(out, "events.csv"), events, ["boot", "t", "line", "kind", "text"])
    with open(os.path.join(out, "summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")

    files = plots(cpu, periods, out)
    print(f"\nResultados en {out}/: summary.txt, cpu.csv, periods.csv, events.csv"
          + "".join(f", {os.path.basename(x)}" for x in files))


if __name__ == "__main__":
    main()
