"""LiDAR-camera extrinsics, by looking at the projection.

Starts the sensors plus project_points.py, which draws the latest sweep on the
image using the same convention FAST-LIVO2 uses (p_cam = Rcl * p_lidar + Pcl):

  ros2 launch handheld_mapping calibrate_lidar_camera.py

Then tune the transform live and watch /image_with_points:

  ros2 param set /project_points Rcl "[0,0,1, -1,0,0, 0,-1,0]"   # row-major, 9 values
  ros2 param set /project_points Pcl "[x, y, z]"                  # metres, LiDAR axes

Pcl = where the camera sits in the LiDAR frame: measure it with a ruler
(camera lens centre relative to the Mid-360's centre, in x-forward / y-left /
z-up axes; behind the LiDAR means negative x). Rcl maps LiDAR axes to the
camera's optical axes (x right, y down, z forward); the default here is
"camera looking along LiDAR +x, level".

When the points sit on the surfaces they came from from several viewpoints,
copy the numbers into config/livo2_mid360.yaml (extrin_calib.Rcl / Pcl). This
is a "good enough to fuse" calibration; a checkerboard solve is better if the
map shows double walls.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import PKG, livox_adapter, livox_driver, share_path, usb_cam  # noqa: E402


def generate_launch_description():
    args = [
        DeclareLaunchArgument('mid360_config',
                              default_value=share_path('config', 'MID360_config.json')),
        DeclareLaunchArgument('use_rqt', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('pixel_format', default_value='yuyv'),
        # Same file FAST-LIVO2 reads, so the overlay and the front end cannot
        # disagree about the intrinsics.
        DeclareLaunchArgument('intrinsics_file',
                              default_value=share_path('config', 'camera_c310.yaml')),
    ]

    projector = Node(
        package=PKG,
        executable='project_points.py',
        name='project_points',
        output='screen',
        parameters=[{
            'image_topic': '/image_raw',
            'cloud_topic': '/livox/lidar_livo',
            'camera_info_topic': '/camera_info',
            'output_topic': '/image_with_points',
            'msg_type': 'pc2',
            'intrinsics_file': LaunchConfiguration('intrinsics_file'),
        }],
    )

    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/image_with_points'],
    )

    ld = LaunchDescription(args)
    ld.add_action(livox_driver(LaunchConfiguration('mid360_config')))
    ld.add_action(livox_adapter())
    ld.add_action(usb_cam(LaunchConfiguration('video_device'),
                          LaunchConfiguration('image_width'),
                          LaunchConfiguration('image_height'),
                          LaunchConfiguration('framerate'),
                          LaunchConfiguration('pixel_format')))
    ld.add_action(projector)
    ld.add_action(GroupAction([rqt], condition=IfCondition(LaunchConfiguration('use_rqt'))))
    return ld
