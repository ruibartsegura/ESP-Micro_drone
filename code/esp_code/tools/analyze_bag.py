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
  - With --plot, saves 3 graphs of the drone (/drone/state) vs real in
    <bag>_analysis/ (needs: sudo apt install python3-matplotlib):
      posicion.png     x, y, z, their velocity and acceleration (world axes)
      orientacion.png  roll, pitch, yaw, angular velocity and acceleration
      recorrido.png    path from above (x up, y to the left)

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


def smooth(v, n):
    # Centered moving average of n samples (the derivatives amplify the noise)
    if n <= 1 or len(v) < n:
        return list(v)
    h = n // 2
    out = []
    for i in range(len(v)):
        w = v[max(0, i - h):i + h + 1]
        out.append(sum(w) / len(w))
    return out


def deriv(t, v):
    # Central difference (forward / backward at the ends)
    n = len(t)
    if n < 2:
        return [0.0] * n
    out = []
    for i in range(n):
        a, b = max(i - 1, 0), min(i + 1, n - 1)
        dt = t[b] - t[a]
        out.append((v[b] - v[a]) / dt if dt > 0 else 0.0)
    return out


def unwrap(deg):
    # Yaw without the jumps of +-180
    out = []
    for d in deg:
        if out:
            d += round((out[-1] - d) / 360) * 360
        out.append(d)
    return out


def est_series(state, t0):
    # What the drone sends in /drone/state. acc_lin is the accelerometer in
    # body axes (m/s^2, with gravity): it is turned to world axes with the q
    # of the drone and the gravity is subtracted (what the firmware "sees")
    s = {"t": [t - t0 for t, _ in state]}
    for k in ("x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az",
              "roll", "pitch", "yaw", "wroll", "wpitch", "wyaw"):
        s[k] = []
    for _, m in state:
        s["x"].append(m.pos.x); s["y"].append(m.pos.y); s["z"].append(m.pos.z)
        s["vx"].append(m.vel_lin.x); s["vy"].append(m.vel_lin.y); s["vz"].append(m.vel_lin.z)
        a = rotate(m.q, (m.acc_lin.x, m.acc_lin.y, m.acc_lin.z))
        s["ax"].append(a[0]); s["ay"].append(a[1]); s["az"].append(a[2] - G)
        r, p = quat_to_rp(m.q)
        s["roll"].append(r); s["pitch"].append(p); s["yaw"].append(quat_to_yaw(m.q))
        s["wroll"].append(math.degrees(m.vel_ang.x))
        s["wpitch"].append(math.degrees(m.vel_ang.y))
        s["wyaw"].append(math.degrees(m.vel_ang.z))
    s["yaw"] = unwrap(s["yaw"])
    for k in ("roll", "pitch", "yaw"):
        s["d" + k] = deriv(s["t"], s["w" + k])
    return s


def real_series(ref, imu, state, t0):
    # Position / velocity from the reference (ground truth or IMU integrated),
    # with x, y aligned with the drone at the first /drone/state message.
    # Acceleration and orientation from the Gazebo IMU (true orientation, no
    # noise): its acceleration turned with the true q is the real one
    s = {"t": [t - t0 for t, *_ in ref]}
    off = (0.0, 0.0)
    if state and ref:
        ts = [x[0] for x in ref]
        p = ref[max(bisect.bisect_right(ts, state[0][0]) - 1, 0)][1]
        off = (state[0][1].pos.x - p[0], state[0][1].pos.y - p[1])
    s["x"] = [p[0] + off[0] for _, p, _, _ in ref]
    s["y"] = [p[1] + off[1] for _, p, _, _ in ref]
    s["z"] = [p[2] for _, p, _, _ in ref]
    s["vx"] = [v[0] for _, _, v, _ in ref]
    s["vy"] = [v[1] for _, _, v, _ in ref]
    s["vz"] = [v[2] for _, _, v, _ in ref]

    s["t_imu"] = [t - t0 for t, _ in imu]
    acc = [rotate(m.orientation, (m.linear_acceleration.x, m.linear_acceleration.y,
                                  m.linear_acceleration.z)) for _, m in imu]
    # Gravity of the simulation (Gazebo uses 9.8): median of the first ~1 s
    # with the drone still (the median does not take the impact of the spawn)
    rest = sorted(a[2] for a in acc[:60])
    g_sim = rest[len(rest) // 2] if rest else G
    n = max(1, int(round(0.1 / ((s["t_imu"][-1] - s["t_imu"][0]) / len(imu))))) if len(imu) > 1 else 1
    s["ax"] = smooth([a[0] for a in acc], n)
    s["ay"] = smooth([a[1] for a in acc], n)
    s["az"] = smooth([a[2] - g_sim for a in acc], n)
    rp = [quat_to_rp(m.orientation) for _, m in imu]
    s["roll"] = [r for r, _ in rp]
    s["pitch"] = [p for _, p in rp]
    s["yaw"] = unwrap([quat_to_yaw(m.orientation) for _, m in imu])
    s["wroll"] = [math.degrees(m.angular_velocity.x) for _, m in imu]
    s["wpitch"] = [math.degrees(m.angular_velocity.y) for _, m in imu]
    s["wyaw"] = [math.degrees(m.angular_velocity.z) for _, m in imu]
    for k in ("roll", "pitch", "yaw"):
        s["d" + k] = smooth(deriv(s["t_imu"], s["w" + k]), n)
    return s


def robust_ylim(a, *series, q=0.01):
    # Y limits without the impacts (spawn, landings): the values between the
    # 1 % and the 99 % of the samples, with a margin. Only if the spikes are
    # much bigger than the rest
    v = sorted(x for s in series for x in s)
    if len(v) < 20:
        return False
    lo, hi = v[int(q * len(v))], v[int((1 - q) * len(v)) - 1]
    if v[-1] - v[0] < 3 * (hi - lo):
        return False
    m = max(0.25 * (hi - lo), 1e-3)
    a.set_ylim(lo - m, hi + m)
    return True


def mark_time(axes, est, segs, t0):
    # States of the drone as vertical lines and velocity commands as shaded
    # segments. State changes of less than 1 s (HOVERING <-> EXTERNAL_CONTROL
    # between commands) are joined
    t, sm = est["t"], est["sm"]
    changes = [(t[i], sm[i]) for i in range(len(t)) if i == 0 or sm[i] != sm[i - 1]]
    kept = []
    for i, (tc, s) in enumerate(changes):
        t_next = changes[i + 1][0] if i + 1 < len(changes) else t[-1]
        if (t_next - tc >= 1.0 or i + 1 == len(changes)) and (not kept or kept[-1][1] != s):
            kept.append((tc, s))
    for tc, s in kept:
        for a in axes.flat:
            a.axvline(tc, color="gray", ls="--", lw=0.7)
        name = SM_STATES[s] if s < len(SM_STATES) else str(s)
        axes[0][0].text(tc, 1.02, name, transform=axes[0][0].get_xaxis_transform(),
                        rotation=45, fontsize=7, ha="left", va="bottom")
    for n, sg in enumerate(segs):
        a0, a1 = sg["start"] - t0, sg["end"] - t0
        vx, vy, _, wz = sg["val"]
        color = "tab:green" if vx + vy + wz >= 0 else "tab:purple"
        for a in axes.flat:
            a.axvspan(a0, a1, color=color, alpha=0.12, lw=0)
        label = " ".join(f"{k}={v:+g}" for k, v in (("vx", vx), ("vy", vy), ("wz", wz)) if v)
        # Labels in two heights so the short commands do not overlap
        axes[0][1].text((a0 + a1) / 2, 1.02 + 0.07 * (n % 2), label or "0",
                        transform=axes[0][1].get_xaxis_transform(),
                        fontsize=7, ha="center", va="bottom")


def grid_figure(plt, title, est, real, rows, t_real_key, segs, t0, path):
    # 3x3: one row per magnitude, one column per axis. Estimated = /drone/state,
    # real = Gazebo
    fig, ax = plt.subplots(3, 3, figsize=(16, 10), sharex=True)
    fig.suptitle(title, fontsize=10)
    for i, (keys, unit) in enumerate(rows):
        for j, (k, name) in enumerate(keys):
            a = ax[i][j]
            tr = real[t_real_key[i]]
            a.plot(tr, real[k], "-", lw=1, color="tab:orange", label="real")
            a.plot(est["t"], est[k], ".-", ms=3, lw=1, color="tab:blue", label="medido (/drone/state)")
            a.set_ylabel(f"{name} ({unit})")
            a.grid(alpha=0.3)
            if i > 0 and robust_ylim(a, real[k], est[k]):
                a.text(0.01, 0.97, "picos recortados", transform=a.transAxes,
                       fontsize=6, color="gray", va="top")
    for a in ax[2]:
        a.set_xlabel("t (s)")
    ax[0][2].legend(fontsize=7, loc="upper right")
    mark_time(ax, est, segs, t0)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def plot_figures(state, ref, imu, source, out, segs=(), t0=0.0):
    import matplotlib
    matplotlib.use("Agg")  # no window, only the png
    import matplotlib.pyplot as plt

    est = est_series(state, t0)
    est["sm"] = [m.sm_state for _, m in state]
    real = real_series(ref, imu, state, t0)
    paths = []

    # 1. Position, velocity and acceleration
    p = os.path.join(out, "posicion.png")
    grid_figure(plt, f"Posición, velocidad y aceleración (ejes mundo): medido vs real — {source}",
                est, real,
                [((("x", "x"), ("y", "y"), ("z", "z")), "m"),
                 ((("vx", "vx"), ("vy", "vy"), ("vz", "vz")), "m/s"),
                 ((("ax", "ax"), ("ay", "ay"), ("az", "az")), "m/s²")],
                ["t", "t", "t_imu"], segs, t0, p)
    paths.append(p)

    # 2. Orientation, angular velocity and angular acceleration
    p = os.path.join(out, "orientacion.png")
    grid_figure(plt, "Orientación, velocidad y aceleración angular: medido (q de Madgwick, giroscopio) vs real (IMU de Gazebo)",
                est, real,
                [((("roll", "roll"), ("pitch", "pitch"), ("yaw", "yaw")), "°"),
                 ((("wroll", "ω roll"), ("wpitch", "ω pitch"), ("wyaw", "ω yaw")), "°/s"),
                 ((("droll", "α roll"), ("dpitch", "α pitch"), ("dyaw", "α yaw")), "°/s²")],
                ["t_imu", "t_imu", "t_imu"], segs, t0, p)
    paths.append(p)

    # 3. Path seen from above: x forward (up) and y to the left, as in ENU
    p = os.path.join(out, "recorrido.png")
    fig, a = plt.subplots(figsize=(9, 9))
    a.plot(real["y"], real["x"], "-", lw=1.2, color="tab:orange", label="real")
    a.plot(est["y"], est["x"], ".-", ms=3, lw=1, color="tab:blue", label="medido (/drone/state)")
    a.plot(real["y"][0], real["x"][0], "ko", label="inicio")
    a.plot(real["y"][-1], real["x"][-1], "kx", ms=9, label="fin (real)")
    a.invert_xaxis()  # +y to the left: same view as from behind the drone
    a.set_xlabel("y (m)  ← izquierda")
    a.set_ylabel("x (m)  ↑ adelante")
    a.set_title(f"Recorrido visto desde arriba — {source}", fontsize=10)
    a.set_aspect("equal", adjustable="datalim")
    a.grid(alpha=0.3)
    a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(p, dpi=120)
    plt.close(fig)
    paths.append(p)
    return paths


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
    ap.add_argument("--plot", action="store_true", help="guarda las 3 gráficas (posición, orientación, recorrido) en png")
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
                for p in plot_figures(state, ref, imu, source, out, segs, t0):
                    print(f"PNG: {p}")
            except ImportError:
                print("Falta matplotlib: sudo apt install python3-matplotlib")
    elif a.plot:
        print("No hay /drone/state en el bag, no hay nada que graficar")


if __name__ == "__main__":
    main()
