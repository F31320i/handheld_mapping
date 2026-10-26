"""Bring up the Mid-360 and the USB camera only (no mapping).

Use this to check wiring/network before touching FAST-LIVO2:

  ros2 launch handheld_mapping sensors.py
  ros2 run handheld_mapping check_streams.py --seconds 8

Then either
  ros2 run rqt_image_view rqt_image_view /image_raw
or point RViz's fixed frame at livox_frame and add /livox/lidar_livo.

The topics you want to see: /livox/lidar_livo (PointCloud2, 10 Hz, the format
FAST-LIVO2 wants), /livox/imu_sync (200 Hz), /image_raw (Image, 30 Hz). The raw
/livox/lidar (CustomMsg) and /livox/imu are still there and are what the adapter
consumes; nothing else should use them. There is no /camera_info here: the
camera node is v4l2_capture (see config/v4l2_capture.yaml), and only the
camera_calibration launches use usb_cam, which publishes it.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import livox_adapter, livox_driver, share_path, v4l2_capture  # noqa: E402


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        # 需要你自己的 IP 时先跑 set_mid360_ips.py，再把路径传进来。
        DeclareLaunchArgument('mid360_config',
                              default_value=share_path('config', 'MID360_config.json')),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('pixel_format', default_value='yuyv'),
    ]

    ld = LaunchDescription(args)
    ld.add_action(GroupAction([
        livox_driver(LaunchConfiguration('mid360_config')),
        livox_adapter(),
    ], condition=IfCondition(LaunchConfiguration('use_lidar'))))
    ld.add_action(GroupAction([
        v4l2_capture(LaunchConfiguration('video_device'),
                     LaunchConfiguration('image_width'),
                     LaunchConfiguration('image_height'),
                     LaunchConfiguration('framerate'),
                     LaunchConfiguration('pixel_format')),
    ], condition=IfCondition(LaunchConfiguration('use_camera'))))
    return ld
