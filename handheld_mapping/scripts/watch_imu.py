#!/usr/bin/env python3
"""Live IMU readout: what the Mid-360 is actually reporting, right now.

Built for "put the rig down, do not touch it, is the IMU sane?". It prints a
refreshing block with the instantaneous accel/gyro, the gravity-derived
roll/pitch, how fast samples are arriving, and windowed statistics -- the
standard deviations are the stillness test: at rest on a desk they should sit
around a few mg and well under 0.01 rad/s.

    ros2 run handheld_mapping watch_imu.py                # /livox/imu
    ros2 run handheld_mapping watch_imu.py --topic /livox/imu_sync
    ros2 run handheld_mapping watch_imu.py --plain        # one block/s, no ANSI

When stdout is not a terminal it falls back to --plain on its own, so the
output survives being piped into a log.
"""
import argparse
import math
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

G = 9.80665


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class WatchImu(Node):
    def __init__(self, topic, window):
        super().__init__('watch_imu')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=200)
        self.stamps = deque(maxlen=window)
        self.acc = deque(maxlen=window)
        self.gyr = deque(maxlen=window)
        self.arrivals = deque(maxlen=window)
        self.latest = None
        self.latest_lag = 0.0
        self.count = 0
        self.create_subscription(Imu, topic, self.on_imu, qos)

    def on_imu(self, msg):
        now = time.time()
        self.count += 1
        self.stamps.append(stamp_of(msg))
        self.arrivals.append(now)
        self.acc.append((msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z))
        self.gyr.append((msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z))
        self.latest = msg
        self.latest_lag = now - self.stamps[-1]

    def report(self):
        if self.latest is None:
            return None
        acc = np.asarray(self.acc)
        gyr = np.asarray(self.gyr)
        a = acc[-1]
        w = gyr[-1]
        norm = float(np.linalg.norm(a))
        # Gravity direction -> roll/pitch of the body frame. Yaw is unobservable
        # from an accelerometer; that is not a defect of this readout.
        roll = math.degrees(math.atan2(a[1], a[2]))
        pitch = math.degrees(math.atan2(-a[0], math.hypot(a[1], a[2])))

        t = np.asarray(self.arrivals)
        span = t[-1] - t[0] if len(t) > 1 else 0.0
        rate = (len(t) - 1) / span if span > 0 else 0.0
        gaps = np.diff(np.asarray(self.stamps))
        max_gap = float(gaps.max() * 1e3) if len(gaps) else 0.0

        acc_mean = acc.mean(axis=0)
        acc_std = acc.std(axis=0) if len(acc) > 1 else np.zeros(3)
        gyr_std = gyr.std(axis=0) if len(gyr) > 1 else np.zeros(3)
        still = np.linalg.norm(gyr_std) < 0.01 and abs(np.linalg.norm(acc, axis=1).std()) < 0.05

        return dict(
            count=self.count, window=len(t), rate=rate, max_gap=max_gap,
            acc=a, norm=norm, gyr=w, gyr_norm=float(np.linalg.norm(w)),
            roll=roll, pitch=pitch, lag=self.latest_lag,
            acc_mean=acc_mean, acc_std=acc_std, gyr_std=gyr_std, still=still,
        )


def render(r):
    if r is None:
        return '等待 IMU 数据…（检查 /livox/imu 的发布者，以及 Zenoh router 是否起了）'
    a, w = r['acc'], r['gyr']
    lines = [
        '=== Livox Mid-360 IMU  实时读数 ===',
        f"采样        {r['count']:6d} 条    窗口 {r['window']} 条"
        f"    频率 {r['rate']:6.1f} Hz    最大间隔 {r['max_gap']:5.1f} ms",
        '',
        '加速度 (m/s²)          x          y          z',
        f'  瞬时          {a[0]:+9.3f}  {a[1]:+9.3f}  {a[2]:+9.3f}',
        f'  窗口均值      {r["acc_mean"][0]:+9.3f}  {r["acc_mean"][1]:+9.3f}'
        f'  {r["acc_mean"][2]:+9.3f}',
        f'  窗口标准差    {r["acc_std"][0]:9.3f}  {r["acc_std"][1]:9.3f}'
        f'  {r["acc_std"][2]:9.3f}',
        f'  模长 |a| = {r["norm"]:.3f} m/s²  (重力 {G:.3f}，偏差 {r["norm"] - G:+.3f})',
        '',
        '角速度 (rad/s)         x          y          z',
        f'  瞬时          {w[0]:+9.4f}  {w[1]:+9.4f}  {w[2]:+9.4f}',
        f'  窗口标准差    {r["gyr_std"][0]:9.4f}  {r["gyr_std"][1]:9.4f}'
        f'  {r["gyr_std"][2]:9.4f}',
        f'  模长 |ω| = {r["gyr_norm"]:.4f} rad/s',
        '',
        f'由重力反算姿态  roll {r["roll"]:+7.2f}°   pitch {r["pitch"]:+7.2f}°'
        '   (yaw 加速度计测不到，正常)',
        f'到达延迟        {r["lag"] * 1e3:+7.1f} ms',
        f'静止判定        {"✅ 静止（σ 很小）" if r["still"] else "⚠️ 有抖动 / 还在动"}',
        '',
        'Ctrl-C 退出',
    ]
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/livox/imu',
                    help='IMU topic (default /livox/imu, the raw driver output)')
    ap.add_argument('--window', type=int, default=400,
                    help='samples kept for the statistics (default 400 = 2 s at 200 Hz)')
    ap.add_argument('--period', type=float, default=0.25,
                    help='screen refresh interval in s (default 0.25)')
    ap.add_argument('--plain', action='store_true',
                    help='no ANSI redraw: print one block per refresh')
    args = ap.parse_args()
    ansi = (not args.plain) and sys.stdout.isatty()

    rclpy.init()
    node = WatchImu(args.topic, args.window)
    try:
        while rclpy.ok():
            deadline = time.time() + args.period
            while rclpy.ok() and time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            text = render(node.report())
            if ansi:
                sys.stdout.write('\x1b[2J\x1b[H' + text + '\n')
            else:
                sys.stdout.write(text + '\n\n')
            sys.stdout.flush()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or SIGTERM from `timeout`/systemd: not worth a traceback.
        pass
    finally:
        if ansi:
            sys.stdout.write('\x1b[0m\n')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
