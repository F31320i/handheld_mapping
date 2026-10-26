"""Node builders shared by the handheld_mapping launch files.

Kept in one place because the same handful of nodes show up in the bring-up,
calibration and mapping entry points, and getting one of them subtly wrong
(parameter file, node name, topic) would only show up much later.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'handheld_mapping'


def share_path(*parts):
    return os.path.join(get_package_share_directory(PKG), *parts)


# --------------------------------------------------------------------- sensors
def livox_driver(config_path):
    """Mid-360 driver.

    xfer_format 1 = CustomMsg. Not because the front end takes it -- it does
    not, see livox_adapter() -- but because CustomMsg carries the per-point
    `offset_time` the LiDAR actually measured. The adapter turns that into the
    absolute-nanosecond `timestamp` field LIVO2's mid360_handler reads; the
    driver's own PointCloud2 output (xfer_format 0) writes seconds since the
    sweep start instead, which LIVO2 would interpret as a point ~1.7e9 s in
    the past. multi_topic 0 = one LiDAR, shared topics /livox/lidar and
    /livox/imu (multi_topic 1 would name them after the device).
    """
    return Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_ros_driver2',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': config_path,
            'enable_lidar_bag': False,
            'enable_imu_bag': False,
        }],
    )


def livox_adapter(min_range=0.0):
    """CustomMsg -> PointCloud2 for LIVO2, plus the host-clock time-base guard.

    min_range (metres) blanks the rig out of BOTH output streams: everything
    closer than that to the sensor is dropped before either front end sees the
    sweep, so the near field cannot reach the LIO, the photometric VIO or the
    map. 0 = off.

    C++, not Python: deserialising 20k CustomMsg points per sweep in Python
    costs ~0.7 of a core, and the measured result was an IMU stream sliding
    from +0.05 s to +4.8 s of backlog in 85 s, after which LIVO2's sync step
    never fired again (no LIO/VIO at all, and later a SIGSEGV). See the header
    of src/livox_livo2_adapter.cpp.

    The topic name is deliberately not /livox/lidar_sync: that one used to carry
    CustomMsg, and a router that has seen both types under one name makes
    `ros2 topic hz` / `ros2 topic echo` refuse the topic outright.
    """
    return Node(
        package=PKG,
        executable='livox_livo2_adapter',
        name='livox_livo2_adapter',
        output='screen',
        parameters=[{
            'lidar_in': '/livox/lidar',
            'lidar_out': '/livox/lidar_livo',
            'imu_in': '/livox/imu',
            'imu_out': '/livox/imu_sync',
            'clock_offset': 0.0,      # 0 = measure it
            'estimate_frames': 20,
            'warmup_frames': 5,
            # ParameterValue: the launch argument arrives as a string and the
            # node declares a double; without this the override is rejected.
            'min_range': ParameterValue(min_range, value_type=float),
        }],
    )


def usb_cam(video_device, width, height, framerate, pixel_format, camera_info_url=''):
    # LaunchConfiguration values are substitutions, so the numeric parameters
    # have to say what they should become; plain strings would reach the node
    # as strings and be rejected.
    overrides = {
        'video_device': video_device,
        'image_width': ParameterValue(width, value_type=int),
        'image_height': ParameterValue(height, value_type=int),
        'framerate': ParameterValue(framerate, value_type=float),
        'pixel_format': pixel_format,
    }
    if camera_info_url:
        overrides['camera_info_url'] = camera_info_url
    return Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[share_path('config', 'usb_cam.yaml'), overrides],
    )


def v4l2_capture(video_device, width, height, framerate, pixel_format):
    """The camera node the mapping runs use (usb_cam stays for calibration).

    usb_cam 0.8.1 delivers frames 0.1-1.0 s late on this machine -- measured,
    fixed per process, and unaffected by QoS or polling rate -- because its
    timer-driven capture loop can never drain a backlog. This one drains the
    driver queue every cycle and publishes immediately (33-48 ms at the V4L2
    layer). Same topic, same encoding, same honest buffer timestamp, so nothing
    downstream changes. It publishes no camera_info, which is why the
    camera_calibration launches still use usb_cam().
    """
    return Node(
        package=PKG,
        executable='v4l2_capture',
        name='v4l2_capture',
        output='screen',
        parameters=[
            share_path('config', 'v4l2_capture.yaml'),
            {
                'video_device': video_device,
                'image_width': ParameterValue(width, value_type=int),
                'image_height': ParameterValue(height, value_type=int),
                'framerate': ParameterValue(framerate, value_type=float),
                'pixel_format': pixel_format,
            },
        ],
    )


# -------------------------------------------------------------------- front end
def parameter_blackboard(camera_params_file):
    """Mandatory for FAST-LIVO2, not cosmetic.

    vikit's camera_loader reads the intrinsics from a node called
    parameter_blackboard through a cross-node parameter client. Without it
    cam_width comes back 0 and fastlivo_mapping dies with SIGFPE.
    """
    return Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='parameter_blackboard',
        output='screen',
        parameters=[camera_params_file],
    )


def livo2_front_end(params_file, img_en=1, log_path='/tmp/fastlivo_mapping.log'):
    """FAST-LIVO2 mapping node.

    img_en 0 switches it to LIO-only (slam_mode_ = ONLY_LIO); the camera nodes
    can then be left out, but parameter_blackboard still has to run. img_en is
    written as "1"/"0" (an int parameter), so it survives being a
    LaunchConfiguration; value_type=int is what does the conversion at launch
    time.

    LIVO2 prints several lines per frame, which drowns out everything else.
    output='log' does NOT hide it: launch_ros still mirrors the process output
    to the terminal. Only a shell-level redirect does, so stdout+stderr go to
    log_path instead (`tail -f` it, or pass log_path='' to see it live). The
    prefix is built here, so log_path must be a plain string, not a
    substitution.
    """
    parameters = [params_file, {'common.img_en': ParameterValue(img_en, value_type=int)}]
    prefix = f'''bash -c 'exec "$@" > {log_path} 2>&1' _''' if log_path else ''
    return Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='laserMapping',
        output='log',
        # Python-side name of the XML attribute "launch-prefix".
        prefix=prefix,
        parameters=parameters,
    )


def fast_limo_front_end(params_file):
    """LiDAR-inertial front end, used when there is no camera (or no calibration).

    Takes /livox/lidar_limo and /livox/imu_sync from the adapter. Not
    /livox/lidar_livo: same sweeps, but fast_limo reads a LIVOX point
    `timestamp` as seconds since the sweep start while LIVO2 needs absolute
    nanoseconds, and handing it the wrong one diverges silently. Publishes
    /fast_limo/state (nav_msgs/Odometry), /fast_limo/pointcloud (0.1 m voxel,
    world frame) and, with debug on, /fast_limo/final_raw (same but full
    density). No remap: this launch keeps fast_limo's own topic names so
    `ros2 topic list` shows exactly what the front end does.
    """
    return Node(
        package='fast_limo',
        executable='fast_limo_multi_exec',
        name='fast_limo',
        output='screen',
        parameters=[params_file],
    )


def livo2_bridge(yaml_path=None):
    """IMU body pose -> base_link pose, camera_init/aft_mapped -> odom/base_link."""
    params = [yaml_path] if yaml_path else [share_path('config', 'livo2_odom_bridge_handheld.yaml')]
    return Node(
        package='vector_localization',
        executable='livo2_odom_bridge',
        name='livo2_odom_bridge',
        output='screen',
        parameters=params,
    )


# ---------------------------------------------------------------------- frames
def static_tf(parent, child, xyz=(0.0, 0.0, 0.0)):
    node_name = f'static_{parent}_to_{child}'.replace('/', '_')
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=node_name,
        arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', parent, '--child-frame-id', child],
    )


# ---------------------------------------------------------------------- back end
def pgo_backend(config_path, pcd_path=''):
    params = [config_path]
    if pcd_path:
        params.append({'pcd_path': pcd_path})
    return Node(
        package='vector_mapping',
        executable='pgo_backend_node',
        name='pgo_backend',
        output='screen',
        parameters=params,
    )


def map_accumulator(config_path, cloud_topic=None):
    parameters = [config_path]
    if cloud_topic is not None:
        parameters.append({'input_cloud_topic': cloud_topic})
    return Node(
        package='vector_mapping',
        executable='map_accumulator_node',
        name='map_accumulator',
        output='log',
        parameters=parameters,
    )


def rviz(config_path):
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', config_path],
    )
