"""fast_limo bring-up run: LiDAR + IMU -> front end -> map -> (PGO) -> RViz.

No camera and no calibration board. This is the run that answers "does the rig
itself work" before the LiDAR-camera extrinsic exists: driver, timestamps, IMU,
factory extrinsic, Zenoh, TF -- all of it except vision.

Two maps come out of it, and they are meant to be compared:

  /handheld/map_raw   front-end output accumulated as-is, odom frame, no loop
                      closure: a path that doubles back stays doubled.
  /pgo/map            the same scans, but placed by the pose graph, map frame,
                      with loop-closure constraints. Kept by the back end.

use_pgo:=false drops the back end and leaves the raw accumulation only (that
was the original smoke test).

  # first time on a new rig / network (host IP, LiDAR IP)
  ros2 run handheld_mapping set_mid360_ips.py --host-ip 192.168.1.2 --lidar-ip 192.168.1.109

  # Zenoh router must already be up (README 2.5)
  ros2 run rmw_zenoh_cpp rmw_zenohd

  ros2 launch handheld_mapping mapping_limo.py \\
      mid360_config:=$HOME/.config/handheld_mapping/MID360_config.json

  # Hold the rig STILL for the first ~5 s (calibration.time): that window is
  # where gravity and both IMU biases are measured. Moving through it bakes the
  # error into every pose that follows.

Topic flow (see src/livox_livo2_adapter.cpp):

  /livox/lidar (CustomMsg) --adapter--> /livox/lidar_limo (PointCloud2) --+
  /livox/imu               --adapter--> /livox/imu_sync                 --+--> fast_limo

  /livox/lidar_limo is the same sweeps as /livox/lidar_livo with the point
  `timestamp` field in seconds since the sweep start instead of absolute
  nanoseconds: fast_limo adds it to the cloud stamp, LIVO2 subtracts the header
  from it. Using the wrong one diverges without printing an error.

  fast_limo -> /fast_limo/state        (nav_msgs/Odometry, odom -> base_link TF)
            -> /fast_limo/final_raw    (world frame, full density, debug on)
            -> /fast_limo/pointcloud   (world frame, 0.1 m voxel)

  /fast_limo/final_raw + /fast_limo/state -> map_accumulator -> /handheld/map_raw
                                                            -> /handheld/trajectory_raw
  /fast_limo/final_raw + /fast_limo/state -> pgo_backend     -> /pgo/map
                                                            -> /pgo/trajectory
                                                            -> /pgo/loops

  TF   map --static identity--> odom --fast_limo--> base_link --static--> livox_frame
       (the map -> odom arrow belongs to pgo_backend when use_pgo:=true)

  base_link is the LiDAR centre with the LiDAR's axes, and map -> odom is the
  identity, so at startup the triad in RViz stands exactly at the map origin
  and from then on shows where the sensor is. livox_frame coincides with it by
  construction; it is only there so the raw /livox/lidar_limo cloud can be
  displayed without an unknown-frame error.

Differences from mapping_livo2.py, and why:

  * No /odometry/lio remap: pgo_limo.yaml points odom_topic straight at
    /fast_limo/state. Nothing in this chain answers to /odometry/lio.
  * fast_limo broadcasts odom -> base_link itself (frames.tf_pub: true), so
    there is no livo2_odom_bridge here: there is no second frame to reconcile.
  * map -> odom has exactly one publisher, and which one depends on use_pgo.
    The back end owns it (correction transform) or the static identity does,
    never both -- two publishers of one edge is jitter at best and a
    contradictory frame tree at worst. Until the back end's first keyframe
    there is no map -> odom at all, so RViz complains for the first few
    seconds; that is expected, not a fault.
  * The map has no colour. Nothing in this chain sees the image.
  * RViz uses rviz/mapping_limo.rviz (this file's displays), not mapping.rviz.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import (fast_limo_front_end, livox_adapter, livox_driver,  # noqa: E402
                     map_accumulator, pgo_backend, rviz, share_path, static_tf)


def _default_mid360_config():
    """Prefer the per-machine config, fall back to the share one.

    The copy installed under share/ is a placeholder (host 192.168.1.50, LiDAR
    .160): loading it gets you a driver that starts, publishes nothing, and
    logs no error -- the classic first-run trap. If the user has run
    set_mid360_ips.py the real file is where that script writes it.
    """
    user = os.path.expanduser('~/.config/handheld_mapping/MID360_config.json')
    return user if os.path.exists(user) else share_path('config', 'MID360_config.json')


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='false when replaying a bag'),
        DeclareLaunchArgument('use_accumulator', default_value='true'),
        DeclareLaunchArgument(
            'min_range', default_value='1.0',
            description='drop points closer than this (m) to the sensor before '
                        'fast_limo, so the handle and the operator do not end up '
                        'in the map; 0 disables'),
        DeclareLaunchArgument(
            'use_pgo', default_value='true',
            description='false = raw accumulation only, no loop closure'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when replaying a bag with --clock'),
        DeclareLaunchArgument('mid360_config', default_value=_default_mid360_config()),
        DeclareLaunchArgument('limo_config',
                              default_value=share_path('config', 'fast_limo_handheld.yaml')),
        DeclareLaunchArgument('accumulator_config',
                              default_value=share_path('config',
                                                       'map_accumulator_limo.yaml')),
        DeclareLaunchArgument(
            'map_cloud_topic', default_value='/fast_limo/final_raw',
            description='/fast_limo/pointcloud needs less CPU but caps the map at 0.1 m'),
        DeclareLaunchArgument('pgo_config',
                              default_value=share_path('config', 'pgo_limo.yaml')),
    ]

    ld = LaunchDescription(args)
    ld.add_action(SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')))

    ld.add_action(GroupAction([
        livox_driver(LaunchConfiguration('mid360_config')),
        livox_adapter(LaunchConfiguration('min_range')),
    ], condition=IfCondition(LaunchConfiguration('use_lidar'))))

    ld.add_action(fast_limo_front_end(LaunchConfiguration('limo_config')))
    # Until fast_limo reports is_calibrated() it publishes neither state nor TF,
    # so this is what the operator is waiting for.
    ld.add_action(LogInfo(msg='[mapping_limo] hold the rig still for ~5 s: '
                              'gravity and IMU biases are measured in that window'))

    # Both /fast_limo/state and every cloud are already in the world frame, so
    # map -> odom is the identity here and exists only to give RViz (fixed
    # frame: map) somewhere to stand. It is conditional, because with the back
    # end running that edge carries the loop-closure correction and only one
    # node may publish it.
    ld.add_action(GroupAction([
        static_tf('map', 'odom'),
    ], condition=UnlessCondition(LaunchConfiguration('use_pgo'))))

    # base_link sits at the LiDAR centre with the LiDAR's axes (see the
    # extrinsics block of fast_limo_handheld.yaml), so livox_frame -- the
    # frame_id the driver stamps on /livox/lidar_limo -- is the same frame.
    # The transform is here so that the raw point cloud is displayable at all;
    # nothing else in this chain publishes it, and without it RViz refuses to
    # draw /livox/lidar_limo (no transform to the fixed frame).
    ld.add_action(static_tf('base_link', 'livox_frame'))

    ld.add_action(GroupAction([
        map_accumulator(LaunchConfiguration('accumulator_config'),
                        LaunchConfiguration('map_cloud_topic')),
    ], condition=IfCondition(LaunchConfiguration('use_accumulator'))))

    ld.add_action(GroupAction([
        pgo_backend(LaunchConfiguration('pgo_config')),
    ], condition=IfCondition(LaunchConfiguration('use_pgo'))))

    ld.add_action(GroupAction([
        rviz(share_path('rviz', 'mapping_limo.rviz')),
    ], condition=IfCondition(LaunchConfiguration('use_rviz'))))
    return ld
