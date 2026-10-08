#!/usr/bin/env python3
"""
Made by Rui B.S.

Analyzes a rosbag of the simulation (/imu/data, /barometer/data,
/drone/command/motor_speed).

  - Real rate of each topic and the gaps (time without messages).
  - Real roll/pitch (Gazebo IMU orientation quaternion) next to the roll/pitch
    computed only from the accelerometer (what the firmware estimates).
  - Motor commands, saturation and thrust compared with hover (thrust ~ w^2).
  - Height from the barometer (same formula as the firmware).
  - Saves everything in <bag>_analysis/bag.csv

Usage (with ROS sourced):
  source /opt/ros/jazzy/setup.bash
  python3 tools/analyze_bag.py ~/uni/TFG/rosbag2_2026_10_08-19_07_19
  python3 tools/analyze_bag.py <bag> --step 0.25
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

THROTTLE_BASE = 2387.0   # attitude_controller.c
MAX_POWER = 3000.0       # motors.c (simulation)
P0 = 101325.0


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
    a = ap.parse_args()

    bag = os.path.expanduser(a.bag.rstrip("/"))
    data = read_bag(bag)
    imu, baro, mot = data.get(IMU, []), data.get(BARO, []), data.get(MOT, [])
    t0 = min(m[0][0] for m in (imu, baro, mot) if m)

    print("── Frecuencias (tiempo de pared) " + "─" * 30)
    for name, msgs in ((IMU, imu), (BARO, baro), (MOT, mot)):
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

    # ---- time table
    def at(msgs, t):
        ts = [x[0] for x in msgs]
        i = bisect.bisect_right(ts, t) - 1
        return msgs[max(i, 0)][1] if msgs else None

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


if __name__ == "__main__":
    main()
