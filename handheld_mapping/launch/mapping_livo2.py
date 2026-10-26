"""The mapping session: sensors -> FAST-LIVO2 -> pose-graph back end -> RViz.

  # first time on a new rig / network
  ros2 run handheld_mapping set_mid360_ips.py --host-ip <PC> --lidar-ip <LiDAR>
  ros2 run handheld_mapping set_camera_controls.sh

  ros2 launch handheld_mapping mapping_livo2.py \\
      mid360_config:=$HOME/.config/handheld_mapping/MID360_config.json

Topic flow (see src/livox_livo2_adapter.cpp):

  /livox/lidar (CustomMsg) --adapter--> /livox/lidar_livo (PointCloud2) --+
  /livox/imu               --adapter--> /livox/imu_sync                 --+--> FAST-LIVO2
  /image_raw  (v4l2_capture, host clock)  --------------------------------+

  FAST-LIVO2 -> /aft_mapped_to_init -> livo2_odom_bridge -> /odometry/lio + TF
             -> /cloud_registered  (camera_init, coloured, dense)
  /cloud_registered + /odometry/lio -> pgo_backend -> /pgo/map (loop-closed),
      /pgo/trajectory, /pgo/loops, map -> odom TF
  (optionally) /cloud_registered -> map_accumulator -> /handheld/map_raw
      (the same map without loop closure: compare the two to tell a bad loop
      from a bad front end)

Save the result before Ctrl-C (this writes the loop-closed map):
  ros2 service call /pgo/save_map std_srvs/srv/Trigger

Notes
  * Hold the rig still for ~2 s after the front end starts: FAST-LIVO2's first
    50 IMU samples define gravity and the gyro bias (imu.imu_int_frame).
  * LIO works without the camera, but the photometric VIO is what makes the map
    dense -- walk so that surfaces are seen from a normal-ish angle, and do not
    expect textureless white walls to help.
  * LIVO2's per-frame chatter goes to /tmp/fastlivo_mapping.log
    (`tail -f` it when something looks wrong).
  * Replaying a bag: use_lidar:=false use_camera:=false use_sim_time:=true.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import SetParameter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import (livox_adapter, livox_driver, livo2_bridge,  # noqa: E402
                     livo2_front_end, map_accumulator, parameter_blackboard,
                     pgo_backend, rviz, share_path, static_tf, v4l2_capture)


def _check_img_en(context, use_camera, img_en):
    """Refuse the silent failure: img_en=1 with no camera publisher.

    LIVO2's sync step waits for the image that belongs to the sweep, so with no
    camera it never emits a pose -- no error, just nothing.
    """
    # OpaqueFunction does not resolve its args for us.
    use_camera = perform_substitutions(context, [use_camera])
    img_en = perform_substitutions(context, [img_en])
    if use_camera.lower() != 'true' and int(img_en) != 0:
        raise RuntimeError(
            'use_camera:=false needs img_en:=0 (LIO only); with img_en:=%s '
            'FAST-LIVO2 waits for images that never arrive' % img_en)
    return []


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('img_en', default_value='1',
                              description='0 = LIO only (no photometric VIO)'),
        DeclareLaunchArgument('use_pgo', default_value='true'),
        DeclareLaunchArgument('use_accumulator', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'min_range', default_value='1.0',
            description='Drop points closer than this to the sensor, both streams. '
                        'Carrying the rig, the 1 m around it is the operator: the '
                        'handle, the stand and an arm, which move with the sensor '
                        'and therefore get baked into the map as a ghost. '
                        '0.0 disables the cut.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when replaying a bag with --clock'),
        DeclareLaunchArgument('mid360_config',
                              default_value=share_path('config', 'MID360_config.json')),
        DeclareLaunchArgument('livo2_config',
                              default_value=share_path('config', 'livo2_mid360.yaml')),
        DeclareLaunchArgument('camera_config',
                              default_value=share_path('config', 'camera_c310.yaml')),
        DeclareLaunchArgument('bridge_config',
                              default_value=share_path('config',
                                                       'livo2_odom_bridge_handheld.yaml')),
        DeclareLaunchArgument('pgo_config',
                              default_value=share_path('config', 'pgo_handheld.yaml')),
        DeclareLaunchArgument('accumulator_config',
                              default_value=share_path('config',
                                                       'map_accumulator_handheld.yaml')),
        DeclareLaunchArgument('pcd_path', default_value='',
                              description='/pgo/save_map target (empty = timestamped default)'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('pixel_format', default_value='yuyv'),
    ]

    ld = LaunchDescription(args)
    ld.add_action(SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')))
    ld.add_action(OpaqueFunction(function=_check_img_en,
                                 args=[LaunchConfiguration('use_camera'),
                                       LaunchConfiguration('img_en')]))

    ld.add_action(GroupAction([
        livox_driver(LaunchConfiguration('mid360_config')),
        livox_adapter(LaunchConfiguration('min_range')),
    ], condition=IfCondition(LaunchConfiguration('use_lidar'))))

    ld.add_action(GroupAction([
        v4l2_capture(LaunchConfiguration('video_device'),
                     LaunchConfiguration('image_width'),
                     LaunchConfiguration('image_height'),
                     LaunchConfiguration('framerate'),
                     LaunchConfiguration('pixel_format')),
    ], condition=IfCondition(LaunchConfiguration('use_camera'))))

    ld.add_action(parameter_blackboard(LaunchConfiguration('camera_config')))
    ld.add_action(livo2_front_end(LaunchConfiguration('livo2_config'),
                                  img_en=LaunchConfiguration('img_en')))

    # base_link IS the IMU on this rig, so /odometry/lio and the TF are the
    # front end's pose unchanged (config/livo2_odom_bridge_handheld.yaml).
    ld.add_action(livo2_bridge(LaunchConfiguration('bridge_config')))

    # camera_init is the world frame both the clouds and LIVO2's own poses are
    # expressed in; odom coincides with it (base_link = IMU at init). This is
    # what lets RViz draw /cloud_registered and /path in the map frame.
    ld.add_action(static_tf('odom', 'camera_init'))

    ld.add_action(GroupAction([
        pgo_backend(LaunchConfiguration('pgo_config'), LaunchConfiguration('pcd_path')),
    ], condition=IfCondition(LaunchConfiguration('use_pgo'))))
    # Without the back end nothing publishes map -> odom, while RViz's fixed
    # frame is map.
    ld.add_action(GroupAction([
        static_tf('map', 'odom'),
    ], condition=UnlessCondition(LaunchConfiguration('use_pgo'))))

    ld.add_action(GroupAction([
        map_accumulator(LaunchConfiguration('accumulator_config')),
    ], condition=IfCondition(LaunchConfiguration('use_accumulator'))))

    ld.add_action(GroupAction([
        rviz(share_path('rviz', 'mapping.rviz')),
    ], condition=IfCondition(LaunchConfiguration('use_rviz'))))
    return ld
