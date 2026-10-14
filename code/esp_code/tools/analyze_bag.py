#!/usr/bin/env python3
"""
Made by Rui B.S.

Analyzes a rosbag of the simulation (/imu/data, /barometer/data,
/drone/command/motor_speed, /drone/state, /drone/cmd_vel and /ground_truth if
recorded).

  - Real rate of each topic and the gaps (time without messages).
  - Real roll/pitch (Gazebo IMU orientation quaternion) next to the roll/pitch
    computed only from the accelerometer (what the firmware estimates).
  - Motor commands, saturation and thrust compared with hover (thrust ~ w^2).
  - Height from the barometer (same formula as the firmware).
  - Position, velocity and orientation sent by the drone (/drone/state)
    compared with the real ones. The real ones come from /ground_truth
    (Gazebo odometry) if it is in the bag; if not, they are rebuilt from the
    Gazebo IMU (true orientation, no noise) integrating its acceleration in
    the world frame, and z from the barometer. x, y are aligned with the drone
    at the first message (the firmware starts at 0,0 when it boots).
  - Velocity commands (/drone/cmd_vel): segments, what the drone really did
    during each one and how long it was in EXTERNAL_CONTROL.
  - Saves everything in <bag>_analysis/bag.csv
  - With --plot, saves the drone state vs real graphs in
    <bag>_analysis/state_vs_real.png (needs: sudo apt install python3-matplotlib)

Usage (with ROS sourced):
  source /opt/ros/jazzy/setup.bash
  python3 tools/analyze_bag.py ~/uni/TFG/rosbag2_2026_10_08-19_07_19
  python3 tools/analyze_bag.py <bag> --step 0.25
  python3 tools/analyze_bag.py <bag> --plot

To have the true position, record /ground_truth too:
  ros2 bag record -s mcap /imu/data /barometer/data /drone/command/motor_speed \
      /drone/state /drone/cmd_vel /ground_truth
"""

import argparse
import bisect
import csv
import math
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMU = "/imu/data"
BARO = "/barometer/data"
MOT = "/drone/command/motor_speed"
STATE = "/drone/state"
GT = "/ground_truth"
CMD = "/drone/cmd_vel"

THROTTLE_BASE = 2387.0   # attitude_controller.c
MAX_POWER = 3000.0       # motors.c (simulation)
P0 = 101325.0
G = 9.81
CMD_EXPIRE = 0.5         # CMD_VEL_EXPIRE_TIME in system.c (s)

SM_STATES = ["INIT", "CHECKING", "ARMING", "TAKING_OFF", "HOVERING",
             "EXTERNAL_CONTROL", "LANDING", "DISARMING", "ERROR"]   # state.h


def read_bag(uri):
    r = rosbag2_py.SequentialReader()
    storage = "mcap" if any(f.endswith(".mcap") for f in os.listdir(uri)) else "sqlite3"
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    data = {k: [] for k in types}
    while r.has_next():
        topic, raw, t = r.read_next()
        data[topic].append((t / 1e9, deserialize_message(raw, types[topic])))
    return data


def quat_to_rp(q):
    roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
    return roll, pitch


def quat_to_yaw(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def rotate(q, v):
    # body -> world (same matrix as estimate_odom() in odometry_estimator.c)
    x, y, z, w = q.x, q.y, q.z, q.w
    return ((1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] + 2 * (x * z + w * y) * v[2],
            2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - w * x) * v[2],
            2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2])


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def at(msgs, t):
    # Last message received before t (or the first one)
    ts = [x[0] for x in msgs]
    i = bisect.bisect_right(ts, t) - 1
    return msgs[max(i, 0)][1] if msgs else None


def gt_reference(gt):
    # Gazebo odometry: the twist is in the body frame (child_frame_id)
    ref = []
    for t, m in gt:
        q, p, v = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist.linear
        ref.append((t, (p.x, p.y, p.z), rotate(q, (v.x, v.y, v.z)), q))
    return ref


def imu_reference(imu, baro):
    # Integrates the Gazebo IMU acceleration with its true orientation.
    # Assumes the drone is still at the start of the bag (vel = 0).
    acc_w = [rotate(m.orientation, (m.linear_acceleration.x, m.linear_acceleration.y,
                                    m.linear_acceleration.z)) for _, m in imu]
    # Gravity of the simulation (Gazebo uses 9.8, not 9.81): mean of the
    # first ~1 s with the drone still. 0.01 m/s^2 of error = 1 m/s in 100 s
    rest = acc_w[:30]
    g_sim = sum(a[2] for a in rest) / len(rest) if rest else G
    ref = []
    p, v = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    last = None
    for (t, m), acc in zip(imu, acc_w):
        st = stamp(m)
        dt = st - last if last is not None else 0.0
        last = st
        if not 0.0 < dt <= 0.1:
            dt = 0.0
        acc = [acc[0], acc[1], acc[2] - g_sim]
        for i in range(3):
            p[i] += v[i] * dt + 0.5 * acc[i] * dt * dt
            v[i] += acc[i] * dt
        b = at(baro, t)
        z = baro_to_h(b.fluid_pressure) if b else p[2]
        ref.append((t, (p[0], p[1], z), tuple(v), m.orientation))
    return ref


def merge_z(ref, baro, win=0.2):
    # z from the barometer and vz as its derivative (central difference of
    # +-win/2 s). Integrating the IMU is not good here: the 30 Hz IMU misses
    # the impacts (spawn, landing) and vz keeps an offset afterwards.
    bt = [x[0] for x in baro]
    bh = [baro_to_h(m.fluid_pressure) for _, m in baro]
    bs = [stamp(m) for _, m in baro]

    def idx(t):
        return min(max(bisect.bisect_right(bt, t) - 1, 0), len(bt) - 1)

    out = []
    for t, p, v, q in ref:
        i, j0, j1 = idx(t), idx(t - win / 2), idx(t + win / 2)
        dt = bs[j1] - bs[j0]
        vz = (bh[j1] - bh[j0]) / dt if dt > 0 else 0.0
        out.append((t, (p[0], p[1], bh[i]), (v[0], v[1], vz), q))
    return out


def compare_state(state, ref):
    # One row per /drone/state message with the real value at the same time
    ts = [x[0] for x in ref]
    rows = []
    off = None
    for t, m in state:
        i = max(bisect.bisect_right(ts, t) - 1, 0)
        _, p, v, q = ref[i]
        if off is None:
            off = (m.pos.x - p[0], m.pos.y - p[1])
        r = {"t_wall": t, "sm": m.sm_state,
             "x_est": m.pos.x, "y_est": m.pos.y, "z_est": m.pos.z,
             "x_real": p[0] + off[0], "y_real": p[1] + off[1], "z_real": p[2],
             "vx_est": m.vel_lin.x, "vy_est": m.vel_lin.y, "vz_est": m.vel_lin.z,
             "vx_real": v[0], "vy_real": v[1], "vz_real": v[2]}
        r["roll_est"], r["pitch_est"] = quat_to_rp(m.q)
        r["yaw_est"] = quat_to_yaw(m.q)
        r["roll_q_real"], r["pitch_q_real"] = quat_to_rp(q)
        r["yaw_real"] = quat_to_yaw(q)
        for k in ("x", "y", "z", "vx", "vy", "vz"):
            r[f"e_{k}"] = r[f"{k}_est"] - r[f"{k}_real"]
        r["e_xy"] = math.hypot(r["e_x"], r["e_y"])
        for k in ("roll", "pitch", "yaw"):
            real = r["roll_q_real"] if k == "roll" else r["pitch_q_real"] if k == "pitch" else r["yaw_real"]
            r[f"e_{k}"] = (r[f"{k}_est"] - real + 180) % 360 - 180
        rows.append(r)
    return rows


def cmd_segments(cmd):
    # Groups the consecutive equal commands. A segment ends CMD_EXPIRE after its
    # last message (the firmware drops the command after that time).
    segs = []
    for t, m in cmd:
        l, w = m.twist.linear, m.twist.angular
        val = (l.x, l.y, l.z, w.z)
        if segs and segs[-1]["val"] == val and t - segs[-1]["last"] < 2.0:
            s = segs[-1]
            s["gaps"].append(t - s["last"])
            s["last"] = t
            s["n"] += 1
        else:
            segs.append({"val": val, "start": t, "last": t, "n": 1, "gaps": []})
    for s in segs:
        s["end"] = s["last"] + CMD_EXPIRE
    return segs


def cmd_at(segs, t):
    for s in segs:
        if s["start"] <= t <= s["end"]:
            return s["val"]
    return (0.0, 0.0, 0.0, 0.0)


def print_commands(segs, rows, t0):
    print("\n── Comandos de velocidad (" + CMD + ") " + "─" * 31)
    print("  inicio   fin  | vx    vy    wz   | msgs periodo | EXT_CTRL | real: Δx    Δy    v media | est: Δx    Δy")
    slow = False
    for s in segs:
        rs = [r for r in rows if s["start"] <= r["t_wall"] <= s["end"]]
        per = sum(s["gaps"]) / len(s["gaps"]) if s["gaps"] else 0.0
        slow |= per > CMD_EXPIRE
        line = (f"  {s['start'] - t0:6.1f} {s['end'] - t0:6.1f} | {s['val'][0]:5.1f} {s['val'][1]:5.1f} "
                f"{s['val'][3]:5.1f} | {s['n']:4d} {per:5.2f} s  |")
        if len(rs) < 2:
            print(line + "  (sin /drone/state en el tramo)")
            continue
        ext = 100 * sum(r["sm"] == 5 for r in rs) / len(rs)
        a, b = rs[0], rs[-1]
        dt = b["t_wall"] - a["t_wall"]
        dx, dy = b["x_real"] - a["x_real"], b["y_real"] - a["y_real"]
        v = math.hypot(dx, dy) / dt if dt > 0 else 0.0
        print(line + f"   {ext:4.0f}%  | {dx:6.2f} {dy:6.2f} {v:5.2f} m/s | "
              f"{b['x_est'] - a['x_est']:6.2f} {b['y_est'] - a['y_est']:6.2f}")
    if slow:
        print(f"  AVISO: los comandos llegan más despacio que CMD_VEL_EXPIRE_TIME ({CMD_EXPIRE*1000:.0f} ms):"
              " el dron vuelve a HOVERING entre mensajes. Publica más rápido, p. ej. ros2 topic pub -r 10 ...")


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else 0.0


def print_comparison(rows, source, t0, step):
    keys = ("e_x", "e_y", "e_z", "e_vx", "e_vy", "e_vz", "e_roll", "e_pitch", "e_yaw")
    print("\n── Estado del dron vs real " + "─" * 36)
    print(f"  referencia real: {source}")
    print("  RMS del error por estado (est - real):")
    print("  estado              n |   x(m)   y(m)   z(m) | vx(m/s) vy    vz    | roll(°) pitch  yaw")
    groups = {}
    for r in rows:
        groups.setdefault(r["sm"], []).append(r)
    for sm, rs in sorted(groups.items()) + [(None, rows)]:
        name = "TOTAL" if sm is None else SM_STATES[sm] if sm < len(SM_STATES) else str(sm)
        e = [rms([r[k] for r in rs]) for k in keys]
        print(f"  {name:<17} {len(rs):4d} | {e[0]:6.2f} {e[1]:6.2f} {e[2]:6.2f} | "
              f"{e[3]:6.2f} {e[4]:5.2f} {e[5]:5.2f} | {e[6]:6.2f} {e[7]:6.2f} {e[8]:6.2f}")
    last = rows[-1]
    print(f"  deriva final en xy: {last['e_xy']:.2f} m   máx: {max(r['e_xy'] for r in rows):.2f} m")

    print("\n     t | estado           | x est   real  | y est   real  | z est  real  | vx est  real | vy est  real")
    next_t = -1.0
    for r in rows:
        t = r["t_wall"] - t0
        if t < next_t:
            continue
        next_t = t + step
        name = SM_STATES[r["sm"]] if r["sm"] < len(SM_STATES) else str(r["sm"])
        print(f"{t:6.1f} | {name:<16} | {r['x_est']:7.2f} {r['x_real']:6.2f} | {r['y_est']:7.2f} {r['y_real']:6.2f} | "
              f"{r['z_est']:6.2f} {r['z_real']:5.2f} | {r['vx_est']:6.2f} {r['vx_real']:5.2f} | "
              f"{r['vy_est']:6.2f} {r['vy_real']:5.2f}")


def plot_comparison(rows, source, path, segs=(), t0=0.0):
    import matplotlib
    matplotlib.use("Agg")  # no window, only the png
    import matplotlib.pyplot as plt

    t = [r["t"] for r in rows]
    fig, ax = plt.subplots(3, 3, figsize=(16, 10))
    fig.suptitle(f"/drone/state (estimado) vs real — {source}", fontsize=10)

    panels = [("x", "x (m)"), ("y", "y (m)"), ("z", "z (m)"),
              ("vx", "vx (m/s)"), ("vy", "vy (m/s)"), ("vz", "vz (m/s)")]
    for a, (k, label) in zip(ax[:2].flat, panels):
        a.plot(t, [r[f"{k}_est"] for r in rows], ".-", label="estimado")
        a.plot(t, [r[f"{k}_real"] for r in rows], ".-", label="real")
        a.set_ylabel(label)

    for k in ("roll", "pitch", "yaw"):
        ax[2][0].plot(t, [r[f"e_{k}"] for r in rows], ".-", label=k)
    ax[2][0].set_ylabel("error orientación (°)")

    ax[2][1].plot(t, [r["e_xy"] for r in rows], ".-", color="tab:red")
    ax[2][1].set_ylabel("error xy (m)")

    # State changes as vertical lines on the time graphs. Changes that last
    # less than 1 s (HOVERING <-> EXTERNAL_CONTROL between commands) are joined
    changes = [(r["t"], r["sm"]) for i, r in enumerate(rows) if i == 0 or r["sm"] != rows[i - 1]["sm"]]
    kept = []
    for i, (t, sm) in enumerate(changes):
        t_next = changes[i + 1][0] if i + 1 < len(changes) else rows[-1]["t"]
        if (t_next - t >= 1.0 or i + 1 == len(changes)) and (not kept or kept[-1][1] != sm):
            kept.append((t, sm))
    for t, sm in kept:
        name = SM_STATES[sm] if sm < len(SM_STATES) else str(sm)
        for a in list(ax[:2].flat) + [ax[2][0], ax[2][1]]:
            a.axvline(t, color="gray", ls="--", lw=0.7)
        ax[0][0].text(t, 1.02, name, transform=ax[0][0].get_xaxis_transform(),
                      rotation=45, fontsize=7, ha="left", va="bottom")
    for a in list(ax[:2].flat) + [ax[2][0], ax[2][1]]:
        a.set_xlabel("t (s)")
        a.grid(alpha=0.3)
    for a in list(ax[:2].flat) + [ax[2][0]]:
        a.legend(fontsize=7)

    # Velocity commands as shaded segments
    for s in segs:
        a0, a1 = s["start"] - t0, s["end"] - t0
        vx, vy, _, wz = s["val"]
        color = "tab:green" if vx + vy >= 0 else "tab:purple"
        for a in list(ax[:2].flat) + [ax[2][0], ax[2][1]]:
            a.axvspan(a0, a1, color=color, alpha=0.12, lw=0)
        label = " ".join(f"{k}={v:+g}" for k, v in (("vx", vx), ("vy", vy), ("wz", wz)) if v)
        ax[0][1].text((a0 + a1) / 2, 1.02, label or "0", transform=ax[0][1].get_xaxis_transform(),
                      fontsize=7, ha="center", va="bottom")

    xy = ax[2][2]
    xy.plot([r["x_est"] for r in rows], [r["y_est"] for r in rows], ".-", label="estimado")
    xy.plot([r["x_real"] for r in rows], [r["y_real"] for r in rows], ".-", label="real")
    xy.plot(rows[0]["x_real"], rows[0]["y_real"], "ko", label="inicio")
    xy.set_xlabel("x (m)")
    xy.set_ylabel("y (m)")
    xy.set_aspect("equal", adjustable="datalim")
    xy.grid(alpha=0.3)
    xy.legend(fontsize=7)

    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def acc_to_rp(a):
    # Same formula as get_roll_pitch() in attitude_controller.c
    roll = math.degrees(math.atan2(a.y, a.z))
    pitch = math.degrees(math.atan2(-a.x, math.hypot(a.y, a.z)))
    return roll, pitch


def baro_to_h(p):
    return 44330.0 * (1.0 - (p / P0) ** (1 / 5.255))


def topic_timing(name, msgs):
    if len(msgs) < 2:
        print(f"  {name:<30} {len(msgs)} mensajes")
        return
    t = [x[0] for x in msgs]
    gaps = [b - a for a, b in zip(t, t[1:])]
    hz = (len(t) - 1) / (t[-1] - t[0])
    big = sum(g > 0.1 for g in gaps)
    line = f"  {name:<30} {len(t):5d} msg  {hz:6.1f} Hz  hueco máx {max(gaps)*1000:6.0f} ms  huecos>100ms: {big}"
    print(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--step", type=float, default=0.5, help="paso de la tabla en s")
    ap.add_argument("--base", type=float, default=THROTTLE_BASE)
    ap.add_argument("--max-power", type=float, default=MAX_POWER)
    ap.add_argument("--plot", action="store_true", help="guarda las gráficas de estado vs real en png")
    a = ap.parse_args()

    bag = os.path.expanduser(a.bag.rstrip("/"))
    data = read_bag(bag)
    imu, baro, mot = data.get(IMU, []), data.get(BARO, []), data.get(MOT, [])
    state, gt, cmd = data.get(STATE, []), data.get(GT, []), data.get(CMD, [])
    t0 = min(m[0][0] for m in (imu, baro, mot, state, gt, cmd) if m)

    print("── Frecuencias (tiempo de pared) " + "─" * 30)
    for name, msgs in ((IMU, imu), (BARO, baro), (MOT, mot), (STATE, state), (GT, gt), (CMD, cmd)):
        if msgs:
            topic_timing(name, msgs)
    if len(imu) > 1:
        st = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in imu]
        dt = (st[-1] - st[0]) / (len(st) - 1)
        rtf = (st[-1] - st[0]) / (imu[-1][0] - imu[0][0])
        print(f"  IMU en tiempo de simulación: {1/dt:.1f} Hz (dt={dt*1000:.1f} ms), real-time factor ≈ {rtf:.2f}")

    # ---- motors
    if mot:
        n = len(mot)
        both = sum(1 for _, m in mot if max(m.velocity) >= a.max_power and min(m.velocity) <= 0)
        ratio = [sum(v * v for v in m.velocity) / (4 * a.base ** 2) for _, m in mot]
        print("\n── Motores " + "─" * 52)
        for i in range(len(mot[0][1].velocity)):
            hi = sum(1 for _, m in mot if m.velocity[i] >= a.max_power)
            lo = sum(1 for _, m in mot if m.velocity[i] <= 0)
            print(f"  motor {i}: a MAX {100*hi/n:5.1f}%   a 0 {100*lo/n:5.1f}%")
        print(f"  mensajes con un motor a MAX y otro a 0 a la vez: {100*both/n:.0f}%")
        print(f"  empuje / empuje de hover (∝ w²): medio {sum(ratio)/n:.2f}  máx {max(ratio):.2f}"
              + ("   → NUNCA llega a 1, no puede despegar" if max(ratio) < 1 else ""))

    # ---- drone state vs real
    cmp_rows = []
    if state:
        if gt:
            ref, source = gt_reference(gt), GT + " (Gazebo)"
            if all(abs(r[1][2]) < 1e-6 for r in ref):
                # 2D odometry (OdometryPublisher without <dimensions>3</dimensions>):
                # z and vz are always 0, take them from the barometer
                if baro:
                    ref = merge_z(ref, baro)
                    source += ", z y vz del barómetro (odometría 2D)"
        else:
            ref = imu_reference(imu, baro)
            source = ("IMU de Gazebo integrada + barómetro (no hay " + GT + " en el bag;"
                      " x, y, v son una aproximación)")
        if ref:
            if state[0][1].sm_state > 2:
                print("\n  AVISO: el bag empieza con el dron ya despegado, la referencia supone v=0 al inicio")
            cmp_rows = compare_state(state, ref)
            print_comparison(cmp_rows, source, t0, a.step * 10)
    segs = cmd_segments(cmd)
    if segs and cmp_rows:
        print_commands(segs, cmp_rows, t0)

    # ---- time table
    rows = []
    t_end = max(m[-1][0] for m in (imu, baro, mot) if m)
    t = t0
    while t <= t_end:
        r = {"t": round(t - t0, 3)}
        m = at(imu, t)
        if m:
            r["roll_real"], r["pitch_real"] = quat_to_rp(m.orientation)
            r["roll_acc"], r["pitch_acc"] = acc_to_rp(m.linear_acceleration)
            r["gx"], r["gy"], r["gz"] = m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z
        m = at(mot, t)
        if m:
            for i, v in enumerate(m.velocity):
                r[f"m{i}"] = v
        m = at(baro, t)
        if m:
            r["h_baro"] = baro_to_h(m.fluid_pressure)
        rows.append(r)
        t += a.step

    print("\n── Evolución " + "─" * 50)
    print("     t | roll_real pitch_real | roll_acc pitch_acc | gx(rad/s) gy   | motores                  | h_baro")
    for r in rows:
        ms = " ".join(f"{r.get(f'm{i}', 0):5.0f}" for i in range(4))
        print(f"{r['t']:6.1f} | {r.get('roll_real', 0):8.2f} {r.get('pitch_real', 0):9.2f} | "
              f"{r.get('roll_acc', 0):8.2f} {r.get('pitch_acc', 0):8.2f} | "
              f"{r.get('gx', 0):8.3f} {r.get('gy', 0):6.3f} | {ms} | {r.get('h_baro', 0):6.3f}")

    out = bag + "_analysis"
    os.makedirs(out, exist_ok=True)
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k != "t", k))
    with open(os.path.join(out, "bag.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f"\nCSV: {out}/bag.csv")
    if cmp_rows:
        for r in cmp_rows:
            c = cmd_at(segs, r["t_wall"])
            r["cmd_vx"], r["cmd_vy"], r["cmd_vz"], r["cmd_wz"] = c
            r["t"] = round(r.pop("t_wall") - t0, 3)
        keys = ["t"] + [k for k in cmp_rows[0] if k != "t"]
        with open(os.path.join(out, "state_vs_real.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(cmp_rows)
        print(f"CSV: {out}/state_vs_real.csv")
        if a.plot:
            try:
                plot_comparison(cmp_rows, source, os.path.join(out, "state_vs_real.png"), segs, t0)
                print(f"PNG: {out}/state_vs_real.png")
            except ImportError:
                print("Falta matplotlib: sudo apt install python3-matplotlib")
    elif a.plot:
        print("No hay /drone/state en el bag, no hay nada que graficar")


if __name__ == "__main__":
    main()
