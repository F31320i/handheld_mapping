#!/usr/bin/env python3
"""Save one (or a few) camera frames to PNG.

Needed because FAST-Calib wants a still image file next to the bag, not a
stream: it detects the ArUco markers in a single picture and estimates the
target's pose from that one image.

  ros2 run handheld_mapping save_frame.py --out ~/calib/scene_a.png
  ros2 run handheld_mapping save_frame.py --out ~/calib/s --count 3 --delay 2

Why not `ros2 run image_view image_saver`: on Humble it ignores the `filename`
override (measured 2026-10-26 -- it accepted the parameter, then wrote
left0000.jpg, left0001.jpg, ... into the current directory, 175 files in 6 s).
PNG by default: JPEG would blur exactly the corner locations ArUco detection
depends on.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Saver(Node):
    def __init__(self, topic, out, count, delay, encoding):
        super().__init__('save_frame')
        self.bridge = CvBridge()
        self.out, self.count, self.delay = out, count, delay
        self.encoding = encoding
        self.saved = []
        self.started = None
        self.create_subscription(Image, topic, self.on_image, 1)

    def on_image(self, msg):
        now = self.get_clock().now()
        if self.started is None:
            self.started = now
        # The delay runs from the first frame, so the operator has time to let
        # go of the rig after starting the node.
        if (now - self.started).nanoseconds < self.delay * 1e9:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        except Exception as exc:                     # noqa: BLE001
            self.get_logger().error(f'cv_bridge: {exc}')
            return
        if self.count == 1:
            path = self.out if self.out.lower().endswith('.png') else self.out + '.png'
        else:
            path = f'{self.out}{len(self.saved):02d}.png'
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        if not cv2.imwrite(path, img):
            self.get_logger().error(f'could not write {path}')
            return
        self.saved.append(path)
        self.get_logger().info(
            f'saved {path}  {img.shape[1]}x{img.shape[0]} {img.dtype} '
            f'mean {float(np.mean(img)):.1f}')
        if len(self.saved) >= self.count:
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True,
                    help='file (count=1) or a prefix (count>1; gets 00.png ...)')
    ap.add_argument('--topic', default='/image_raw')
    ap.add_argument('--count', type=int, default=1)
    ap.add_argument('--delay', type=float, default=0.0,
                    help='seconds to wait after the first frame arrives')
    ap.add_argument('--encoding', default='bgr8',
                    help='cv_bridge target encoding; bgr8 is what cv2.imwrite '
                         'and FAST-Calib both want')
    args = ap.parse_args()

    rclpy.init()
    node = Saver(args.topic, args.out, args.count, args.delay, args.encoding)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    saved = list(node.saved)
    node.destroy_node()
    rclpy.shutdown()
    for path in saved:
        print(path)
    if not saved:
        print(f'no frame arrived on {args.topic}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
