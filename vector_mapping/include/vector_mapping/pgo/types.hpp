// Common types for the pose-graph back end.
//
// The back end is deliberately decoupled from the front end: it only consumes
// (a) the raw Livox scan, (b) the front-end odometry, and it produces its own
// map. Nothing in FAST-LIVO2 is modified.

#ifndef VECTOR_MAPPING_PGO_TYPES_HPP
#define VECTOR_MAPPING_PGO_TYPES_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vector_mapping
{
namespace pgo
{

// Same point layout fast_limo expects: x/y/z/intensity plus a FLOAT64
// `timestamp` relative to the scan's header stamp, in seconds. Produced by
// pcd_timestamp_injector with abs_ns:=false.
//
// `rgba` is what lets the same type carry /cloud_registered, which is
// pcl::PointXYZRGB: averaged per channel by pcl::VoxelGrid (CentroidPoint
// special-cases colour), so a rebuilt map keeps the front end's colours. The
// union members r/g/b/a are what PCL's colour accumulator reads.
struct EIGEN_ALIGN16 PointXYZIT
{
  PCL_ADD_POINT4D;
  float intensity = 0.0f;
  double timestamp = 0.0;
  union
  {
    // Opaque white by default, so a cloud without colour renders as plain
    // grey instead of whatever happened to be on the stack.
    std::uint32_t rgba = 0xFFFFFFFFu;
    struct {std::uint8_t b, g, r, a;};
    float rgb;
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using PointT = PointXYZIT;
using CloudT = pcl::PointCloud<PointT>;

// Rigid transform, translation + unit quaternion.
struct Pose
{
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  Eigen::Matrix4d matrix() const
  {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m.block<3, 3>(0, 0) = q.toRotationMatrix();
    m.block<3, 1>(0, 3) = t;
    return m;
  }

  static Pose fromMatrix(const Eigen::Matrix4d & m)
  {
    Pose p;
    p.t = m.block<3, 1>(0, 3);
    p.q = Eigen::Quaterniond(m.block<3, 3>(0, 0));
    p.q.normalize();
    return p;
  }

  Pose operator*(const Pose & o) const
  {
    Pose r;
    r.q = (q * o.q).normalized();
    r.t = t + q * o.t;
    return r;
  }

  Pose inverse() const
  {
    Pose r;
    r.q = q.conjugate();
    r.t = -(r.q * t);
    return r;
  }

  // Rotation angle in radians, in [0, pi].
  double angle() const { return 2.0 * std::acos(std::min(1.0, std::fabs(q.w()))); }
};

// One node of the pose graph. `cloud` is expressed in this keyframe's own
// body frame, so the map assembly is just `pose * cloud`.
struct Keyframe
{
  int id = -1;
  double stamp = 0.0;
  Pose odom_pose;  // front-end pose at capture (never changes)
  Pose pose;       // current estimate; rewritten by the optimiser
  CloudT::Ptr cloud;
};

}  // namespace pgo
}  // namespace vector_mapping

POINT_CLOUD_REGISTER_POINT_STRUCT(
  vector_mapping::pgo::PointXYZIT,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)
  (double, timestamp, timestamp)
  (std::uint32_t, rgba, rgba))

#endif  // VECTOR_MAPPING_PGO_TYPES_HPP
