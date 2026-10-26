"""Camera intrinsics with camera_calibration, on the real C310.

  ros2 launch handheld_mapping calibrate_camera.py size:=8x6 square:=0.020

Note the argument syntax: ros2 launch only accepts name:=value. The older
"--size 8x6 --square 0.02" form that used to be written down here is rejected
outright by argparse ("unrecognized arguments").

Wave the checkerboard until X, Y, Size and Skew all turn green, then press
SAVE. The result lands in /tmp/calibrationdata.tar.gz; the file inside that
matters is ost.yaml:

  tar -xzf /tmp/calibrationdata.tar.gz -C /tmp ost.yaml
  ros2 run handheld_mapping camera_calib_to_livo2.py \\
      --calib /tmp/ost.yaml --out src/handheld_mapping/config/camera_c310.yaml

Calibrate at 640x480 (the resolution the mapping launch runs) and make sure the
camera's exposure is what it will be while mapping -- run
  ros2 run handheld_mapping set_camera_controls.sh
first, otherwise auto-exposure smears the corners.

--size counts *interior* corners: the printed sheet in docs/calib_boards_*.pdf
has 9x7 squares, hence size:=8x6. --square is the square size in metres, and it
must match the sheet that was actually printed -- 0.020 for that one. Getting it
wrong scales fx/fy by the same ratio and does not fail loudly: LIVO2 just
projects every point at the wrong size.

--no-service-check is required: usb_cam publishes camera_info but has no
set_camera_info service, and the calibrator otherwise waits for one forever.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import usb_cam  # noqa: E402


def generate_launch_description():
    args = [
        DeclareLaunchArgument('size', default_value='8x6'),
        DeclareLaunchArgument('square', default_value='0.020',
                              description='checkerboard square size, metres'),
        DeclareLaunchArgument('camera_name', default_value='c310'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('pixel_format', default_value='yuyv'),
        DeclareLaunchArgument(
            'disable_fast_check', default_value='true',
            description='pass --disable_calib_cb_fast_check. The node defaults '
                        'to cv2.CALIB_CB_FAST_CHECK, which bails out early on '
                        'frames it thinks have no board, so the corners blink '
                        'in and out of the display. Off = full detection on '
                        'every frame, which is what you want for a one-off '
                        'calibration'),
    ]

    def calibrator(disabled):
        # The flag has no value, so it cannot be turned into a substitution;
        # two nodes with opposite conditions is the plain way to make it
        # optional.
        arguments = [
            '--size', LaunchConfiguration('size'),
            '--square', LaunchConfiguration('square'),
            '--camera_name', LaunchConfiguration('camera_name'),
            '--no-service-check',
        ]
        if disabled:
            arguments.append('--disable_calib_cb_fast_check')
        cond = LaunchConfiguration('disable_fast_check')
        return Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='cameracalibrator',
            output='screen',
            arguments=arguments,
            remappings=[('image', '/image_raw')],
            condition=IfCondition(cond) if disabled else UnlessCondition(cond),
        )

    ld = LaunchDescription(args)
    ld.add_action(usb_cam(LaunchConfiguration('video_device'),
                          LaunchConfiguration('image_width'),
                          LaunchConfiguration('image_height'),
                          LaunchConfiguration('framerate'),
                          LaunchConfiguration('pixel_format')))
    ld.add_action(calibrator(True))
    ld.add_action(calibrator(False))
    return ld
