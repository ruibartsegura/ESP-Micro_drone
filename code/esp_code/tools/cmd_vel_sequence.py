#!/usr/bin/env python3
"""
Sends a sequence of velocity commands to the drone (drone/cmd_vel).

Each command is published at 10 Hz during HOLD_S seconds. Between two commands
nothing is published during GAP_S seconds, so the drone goes back to HOVERING
by itself (the firmware discards a cmd_vel after 500 ms).

The drone must be flying (HOVERING) before running it. Ctrl+C stops the
sequence: the drone stays in HOVERING.

Usage (with ROS 2 sourced):
    ./cmd_vel_sequence.py
"""

import time

import rclpy
from geometry_msgs.msg import TwistStamped

TOPIC = "/drone/cmd_vel"
RATE_HZ = 10.0
HOLD_S = 7.0
GAP_S = 4.0
WAIT_SUB_S = 10.0 # max time waiting for the drone

# (name, linear.x m/s, linear.y m/s, angular.z rad/s)
SEQUENCE = [
    ("Y +0.3",    0.0,  0.3,  0.0),
    ("Y -0.3",    0.0, -0.3,  0.0),
    ("X +0.3",    0.3,  0.0,  0.0),
    ("X -0.3",   -0.3,  0.0,  0.0),
    ("Yaw +0.2",  0.0,  0.0,  0.2),
    ("Yaw -0.2",  0.0,  0.0, -0.2),
]


def wait_subscriber(pub):
    """Waits until the drone is subscribed, so the first command is complete."""
    t_end = time.monotonic() + WAIT_SUB_S
    while pub.get_subscription_count() == 0:
        if time.monotonic() > t_end:
            return False
        time.sleep(0.1)
    return True


def hold(pub, x, y, wz):
    """Publishes the command at RATE_HZ during HOLD_S."""
    msg = TwistStamped()
    msg.twist.linear.x = x
    msg.twist.linear.y = y
    msg.twist.angular.z = wz

    t0 = time.monotonic()
    for i in range(round(HOLD_S * RATE_HZ)):
        pub.publish(msg)
        # Absolute deadlines: the period does not accumulate delays
        time.sleep(max(0.0, t0 + (i + 1) / RATE_HZ - time.monotonic()))


def main():
    rclpy.init()
    node = rclpy.create_node("cmd_vel_sequence")
    pub = node.create_publisher(TwistStamped, TOPIC, 10)

    try:
        print(f"Waiting for a subscriber on {TOPIC}...")
        if not wait_subscriber(pub):
            print("No subscriber found: is the drone connected to the agent?")
            return 1

        for i, (name, x, y, wz) in enumerate(SEQUENCE):
            if i > 0:
                print(f"  pause {GAP_S:g} s")
                time.sleep(GAP_S)
            print(f"[{i + 1}/{len(SEQUENCE)}] {name} during {HOLD_S:g} s")
            hold(pub, x, y, wz)

        print("Sequence finished")
        return 0
    except KeyboardInterrupt:
        print("\nStopped: the drone goes back to HOVERING")
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
