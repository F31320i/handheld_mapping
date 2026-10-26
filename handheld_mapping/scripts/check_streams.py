#!/usr/bin/env python3
"""One-shot health check for the handheld rig's streams.

Reports, over a short window:
  * arrival rate of each topic,
  * the image size and encoding,
  * how each stream's header stamp compares with the host clock -- this is the
    number that tells you whether the time bases agree.

Read the last column like this:
  lidar/imu  ~ -1.79e9 s   raw driver output: device clock, NOT host time
                            -> livox_livo2_adapter must be running
  lidar_livo ~ -0.15 s (network/driver latency plus the 100 ms sweep)
  image      ~ -0.04 s with v4l2_capture; -0.1 to -1.0 s with usb_cam, which
             varies per start (see README 6.2b). Either way it is constant.
If image and lidar_livo agree to within a second, LIVO2 can associate them.

Usage:
  ros2 run handheld_mapping check_streams.py [--seconds 6]
"""
import argparse
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:
    CustomMsg = None


def qos():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=20,
    )


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Checker(Node):
    def __init__(self):
        super().__init__('check_streams')
        self.count = {}
        self.diff = {}       # topic -> list of (stamp - host_at_callback), seconds
        self.meta = {}
        self.t0 = time.time()

        def wrap(topic, kind):
            self.count[topic] = 0
            self.diff[topic] = []

            def cb(msg):
                self.count[topic] += 1
                now = time.time()
                self.diff[topic].append(stamp_s(msg.header) - now)
                if kind == 'image':
                    self.meta[topic] = f'{msg.width}x{msg.height} {msg.encoding}'
                elif kind == 'custom':
                    self.meta[topic] = f'{len(msg.points)} points'
                elif kind == 'pc2':
                    self.meta[topic] = f'{msg.width} points'
            return cb

        if CustomMsg is not None:
            self.create_subscription(CustomMsg, '/livox/lidar', wrap('/livox/lidar', 'custom'), qos())
        self.create_subscription(PointCloud2, '/livox/lidar_livo',
                                 wrap('/livox/lidar_livo', 'pc2'), qos())
        self.create_subscription(Image, '/image_raw', wrap('/image_raw', 'image'), qos())
        self.create_subscription(CameraInfo, '/camera_info', wrap('/camera_info', 'info'), qos())
        self.create_subscription(Imu, '/livox/imu_sync', wrap('/livox/imu_sync', 'imu'), qos())

    def report(self):
        elapsed = time.time() - self.t0
        print(f'\n=== streams over {elapsed:.1f} s ===')
        print(f'{"topic":<22}{"rate":>9}  {"stamp-host (median)":>22}  info')
        for topic in sorted(self.count):
            n = self.count[topic]
            rate = f'{n / elapsed:7.1f} Hz' if n else '   none'
            diffs = self.diff[topic]
            if diffs:
                med = statistics.median(diffs)
                spread = (max(diffs) - min(diffs)) * 1000
                when = f'{med:+12.3f} s' if abs(med) < 1000 else f'{med / 1e9:+.3f}e9 s'
                info = self.meta.get(topic, '')
                print(f'{topic:<22}{rate:>9}  {when:>22}  {info}  (spread {spread:.1f} ms)')
            elif topic == '/camera_info':
                # Expected on the mapping stack: v4l2_capture publishes only
                # /image_raw and the calibration launches are the ones that use
                # usb_cam. Not a fault.
                print(f'{topic:<22}{rate:>9}  {"-":>22}  '
                      'not published by v4l2_capture (calibration only)')
            else:
                print(f'{topic:<22}{rate:>9}  {"-":>22}  NO MESSAGE')
        print('')

    def verdict(self):
        led = self.count.get('/livox/lidar', 0)
        imu = self.count.get('/livox/imu_sync', 0)
        img = self.count.get('/image_raw', 0)
        sync = self.count.get('/livox/lidar_livo', 0)
        print('=== checks ===')
        print(f'{"OK  " if led > 0 else "FAIL"} LiDAR driver       ({led} sweeps)')
        print(f'{"OK  " if sync > 0 else "FAIL"} lidar adapter       ({sync} PointCloud2 sweeps)')
        print(f'{"OK  " if imu > 0 else "FAIL"} IMU on /livox/imu_sync ({imu} samples)')
        print(f'{"OK  " if img > 0 else "FAIL"} camera /image_raw   ({img} frames)')
        if '/image_raw' in self.diff and self.diff['/image_raw'] and sync:
            img_med = statistics.median(self.diff['/image_raw'])
            if self.diff['/livox/lidar_livo']:
                lid_med = statistics.median(self.diff['/livox/lidar_livo'])
                print(f'image vs adapted LiDAR stamp base differs by '
                      f'{(img_med - lid_med) * 1e3:+.1f} ms '
                      f'(both should be within ~a few hundred ms of each other)')
        print('')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=6.0)
    args, _ = parser.parse_known_args()

    if CustomMsg is None:
        print('warning: livox_ros_driver2.msg not importable; LiDAR rows will be missing.\n'
              'source the workspace (install/setup.bash) before running this.', file=sys.stderr)

    rclpy.init()
    node = Checker()
    end = time.time() + args.seconds
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.report()
    node.verdict()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
