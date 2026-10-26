"""Record a mapping session (or a calibration session) to a rosbag2.

  ros2 launch handheld_mapping record.py bag_out:=$HOME/bags/room1

What is recorded is what FAST-LIVO2 consumes, already time-shifted into host
time, plus tf. The raw device-clock topics are deliberately NOT recorded: a bag
that mixes the two bases is a trap later.

Replay:
  ros2 launch handheld_mapping mapping_livo2.py use_lidar:=false use_camera:=false
  ros2 bag play <bag> --clock        # use_sim_time:=true on the mapping launch

(Pass use_sim_time:=true to mapping_livo2.py when replaying: every node then
follows /clock from the bag instead of the wall clock.)
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

TOPICS = [
    '/livox/lidar_livo',    # PointCloud2, 10 Hz, absolute-ns point stamps
    '/livox/imu_sync',      # 200 Hz, host-clock stamps
    '/image_raw',           # 30 Hz
    '/camera_info',
    '/tf',
    '/tf_static',
]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('bag_out',
                              default_value=os.path.expanduser('~/bags/handheld'),
                              description='bag directory (rosbag2 adds a timestamp suffix)'),
    ]
    # Need more topics? Run `ros2 bag record` by hand with them added; launch
    # arguments cannot be split into several argv entries cleanly.
    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_out'), *TOPICS],
        output='screen',
    )
    return LaunchDescription(args + [record])
