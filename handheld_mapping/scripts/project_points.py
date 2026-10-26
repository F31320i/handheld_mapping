#!/usr/bin/env python3
"""Overlay LiDAR points on the camera image, to check/fine-tune the extrinsics.

This is the visual feedback loop for the LiDAR-camera calibration: it takes the
latest Mid-360 sweep, applies the same convention FAST-LIVO2 uses

    p_cam = Rcl * p_lidar + Pcl

projects with the camera matrix, and publishes the image with coloured points
on it. Points are coloured by depth (near = red, far = blue).

Workflow:
  1. Start the rig (launch calibrate_lidar_camera.py).
  2. rqt_image_view -> /image_with_points.
  3. Point the rig at something with a clear edge (a door frame, a box) and
     tune Rcl/Pcl live:
        ros2 param set /project_points Rcl "[r00, r01, ...]"   (row-major, 9 values)
        ros2 param set /project_points Pcl "[x, y, z]"
     until the points sit on the real surface.
  4. Copy the final Rcl/Pcl into config/livo2_mid360.yaml
     (extrin_calib.Rcl / extrin_calib.Pcl).

A ruler is good enough for Pcl (LiDAR to camera position, in LiDAR axes). For
Rcl, start from: camera optical frame = the camera's own x right / y down /
z forward; the LiDAR is x forward / y left / z up. If the camera looks along
LiDAR +x, both frames are level and the rotation is
  Rcl = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]   (LiDAR x,y,z -> cam x,y,z)
then adjust the angles from the projection. A proper checkerboard solve gives
better numbers; this gets you close enough to see what is wrong.

msg_type: "pc2" for the PointCloud2 our lidar adapter publishes (the default;
that is /livox/lidar_livo) or "custom" for the driver's raw CustomMsg.

Intrinsics come from /camera_info if it carries a real camera matrix, else from
`intrinsics_file` (the same vikit YAML FAST-LIVO2 uses, so after
camera_calib_to_livo2.py this node and the front end cannot disagree), else from
the fx/fy/cx/cy parameters. usb_cam with an empty camera_info_url publishes an
all-zero K, which is why the file/parameter fallbacks exist.
"""
import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:
    CustomMsg = None


def sensor_qos(depth=5):
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class Projector(Node):
    def __init__(self):
        super().__init__('project_points')
        self.image_topic = self.declare_parameter('image_topic', '/image_raw').value
        self.cloud_topic = self.declare_parameter('cloud_topic', '/livox/lidar_livo').value
        self.info_topic = self.declare_parameter('camera_info_topic', '/camera_info').value
        self.out_topic = self.declare_parameter('output_topic', '/image_with_points').value
        self.msg_type = self.declare_parameter('msg_type', 'pc2').value
        self.intrinsics_file = self.declare_parameter('intrinsics_file', '').value
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', -1.0)      # negative = image centre
        self.declare_parameter('cy', -1.0)
        self.declare_parameter('Rcl', [0.0, 0.0, 1.0,
                                       -1.0, 0.0, 0.0,
                                       0.0, -1.0, 0.0])
        self.declare_parameter('Pcl', [0.0, 0.0, 0.0])
        self.declare_parameter('max_range', 25.0)
        self.declare_parameter('min_depth', 0.15)
        self.declare_parameter('point_radius', 2)
        self.declare_parameter('max_points', 8000)

        self.bridge = CvBridge()
        self.image = None
        self.xyz = None
        self.K = None
        self.file_K = self._load_intrinsics_file()
        self._warned_no_K = False
        self.pub = self.create_publisher(Image, self.out_topic, sensor_qos())
        self.create_subscription(Image, self.image_topic, self.on_image, sensor_qos())
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, sensor_qos())
        if self.msg_type == 'custom':
            if CustomMsg is None:
                raise RuntimeError('livox_ros_driver2.msg unavailable; use msg_type:=pc2')
            self.create_subscription(CustomMsg, self.cloud_topic, self.on_custom, sensor_qos())
        else:
            self.create_subscription(PointCloud2, self.cloud_topic, self.on_pc2, sensor_qos())
        self.get_logger().info(
            f'projecting {self.cloud_topic} into {self.image_topic} -> {self.out_topic}; '
            'tune Rcl/Pcl with ros2 param set')

    # ---------------------------------------------------------------- inputs
    def _load_intrinsics_file(self):
        """Read cam_fx/cam_fy/cam_cx/cam_cy from a vikit camera YAML."""
        if not self.intrinsics_file:
            return None
        try:
            with open(self.intrinsics_file, 'r') as fh:
                data = yaml.safe_load(fh)
            params = data.get('**', data).get('ros__parameters', data)
            fx = float(params['cam_fx'])
            fy = float(params['cam_fy'])
            cx = float(params['cam_cx'])
            cy = float(params['cam_cy'])
        except Exception as exc:
            self.get_logger().error(f'cannot read intrinsics from {self.intrinsics_file}: {exc}')
            return None
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().error(
                f'{self.intrinsics_file} still holds placeholder/zero intrinsics')
            return None
        return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    def on_info(self, msg):
        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        # usb_cam with an empty camera_info_url publishes an all-zero matrix;
        # using it would project every point to 0/0 and draw nothing.
        if k[0, 0] > 0.0 and k[1, 1] > 0.0:
            self.K = k

    def _intrinsics(self, w, h):
        if self.K is not None:
            return self.K
        if self.file_K is not None:
            return self.file_K
        fx = float(self.get_parameter('fx').value)
        fy = float(self.get_parameter('fy').value)
        if fx <= 0.0 or fy <= 0.0:
            if not self._warned_no_K:
                self._warned_no_K = True
                self.get_logger().warning(
                    'no usable intrinsics: /camera_info has an all-zero K, '
                    'intrinsics_file is empty and fx/fy are unset. Run the camera '
                    'calibration, or pass intrinsics_file:=<camera_c310.yaml> / '
                    'fx:=.. fy:=.. cx:=.. cy:=..')
            return None
        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)
        return np.array([[fx, 0.0, w * 0.5 if cx < 0.0 else cx],
                         [0.0, fy, h * 0.5 if cy < 0.0 else cy],
                         [0.0, 0.0, 1.0]])

    def _keep(self, xyz):
        stride = max(1, int(np.ceil(len(xyz) / max(1, self.get_parameter('max_points').value))))
        self.xyz = np.ascontiguousarray(xyz[::stride], dtype=np.float32)

    def on_custom(self, msg):
        pts = msg.points
        stride = max(1, int(np.ceil(len(pts) / max(1, self.get_parameter('max_points').value))))
        arr = np.fromiter(
            (c for p in pts[::stride] for c in (p.x, p.y, p.z)),
            dtype=np.float32, count=((len(pts) + stride - 1) // stride) * 3)
        self.xyz = arr.reshape(-1, 3)

    def on_pc2(self, msg):
        # read_points yields one (x, y, z) per point, either as tuples (Humble)
        # or as rows of a structured array; tuple() handles both.
        xyz = np.array([tuple(p) for p in
                        pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)],
                       dtype=np.float32)
        if xyz.size:
            self._keep(xyz.reshape(-1, 3))

    # ---------------------------------------------------------------- output
    def on_image(self, msg):
        if self.xyz is None or len(self.xyz) == 0:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # pragma: no cover - depends on the camera
            self.get_logger().error(f'cv_bridge: {exc}')
            return
        h, w = img.shape[:2]
        K = self._intrinsics(w, h)
        if K is None:
            return

        Rcl = np.array(self.get_parameter('Rcl').value, dtype=np.float64).reshape(3, 3)
        Pcl = np.array(self.get_parameter('Pcl').value, dtype=np.float64).reshape(3)
        max_range = float(self.get_parameter('max_range').value)
        min_depth = float(self.get_parameter('min_depth').value)
        radius = int(self.get_parameter('point_radius').value)

        p_cam = self.xyz.astype(np.float64) @ Rcl.T + Pcl
        depth = p_cam[:, 2]
        keep = (depth > min_depth) & (depth < max_range)
        if not np.any(keep):
            self.pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))
            return
        p_cam = p_cam[keep]
        depth = depth[keep]

        uv = (K @ p_cam.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        uv, depth = uv[inside], depth[inside]

        # near = red, far = blue
        norm = np.clip(depth / max_range, 0.0, 1.0)
        colours = np.stack([255 * (1 - norm), 40 * np.ones_like(norm), 255 * norm], axis=1)
        for (u, v), c in zip(uv, colours):
            cv2.circle(img, (int(u), int(v)), radius,
                       (int(c[0]), int(c[1]), int(c[2])), -1)

        out = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        out.header = msg.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Projector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
