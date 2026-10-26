// Motion compensation for the raw Livox scan.
//
// Each point of a sweep is measured at a different instant; without deskewing
// a rotating platform smears every surface. LIVO2 already deskews internally,
// but its published /cloud_registered is a *voxel-filtered* cloud in the world
// frame -- too coarse to keyframe from. So the back end takes the raw scan and
// undoes the motion itself, using the front-end odometry as the motion prior.
//
// Chain for one point with relative timestamp tau:
//   p_odom(tau) = T_odom_base(tau) * T_base_lidar * p_lidar
// and the result is re-expressed in the body frame at the end of the sweep:
//   p_out = T_odom_base(t_end)^-1 * p_odom(tau) = M(tau) * p_lidar
// so a keyframe cloud always lives in that keyframe's own body frame.

#ifndef VECTOR_MAPPING_PGO_DESKEWER_HPP
#define VECTOR_MAPPING_PGO_DESKEWER_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "vector_mapping/pgo/pose_buffer.hpp"
#include "vector_mapping/pgo/types.hpp"

namespace vector_mapping
{
namespace pgo
{

struct DeskewOptions
{
  bool enabled = true;
  Eigen::Matrix4d T_base_lidar = Eigen::Matrix4d::Identity();
};

struct DeskewStats
{
  std::size_t points = 0;
  std::size_t invalid = 0;   // non-finite returns, dropped
  double span = 0.0;         // seconds between first and last point of the sweep
  double max_motion = 0.0;   // largest per-point displacement applied, metres
  double mean_motion = 0.0;  // mean per-point displacement, metres
  bool extrapolated = false; // odometry did not cover the whole sweep
};

class Deskewer
{
public:
  explicit Deskewer(const DeskewOptions & opts) : opts_(opts) {}

  // `scan_stamp` is the cloud header stamp (end of sweep, absolute seconds).
  // `pose_end` is T_odom_base at that instant; the result is in that body frame.
  // Returns nullptr only when `raw` is empty.
  CloudT::Ptr deskew(
    const CloudT & raw, double scan_stamp,
    const std::vector<PoseBuffer::Entry> & poses,
    const Pose & pose_end, DeskewStats * stats) const
  {
    if (raw.empty()) {
      return nullptr;
    }

    auto out = std::make_shared<CloudT>();
    out->reserve(raw.size());
    out->header = raw.header;

    const Eigen::Matrix4d T_end_inv = pose_end.inverse().matrix();
    const Eigen::Matrix4d T_bl = opts_.T_base_lidar;

    double min_tau = 0.0, max_tau = 0.0;
    bool first = true;
    double motion_sum = 0.0, motion_max = 0.0;
    std::size_t invalid = 0;
    bool extrapolated = false;

    // Livox points arrive time-ordered within a sweep, so walk the pose buffer
    // with a monotonic cursor instead of binary-searching per point.
    std::size_t cursor = 0;
    Pose p_tau = pose_end;

    for (const auto & pt : raw.points) {
      // Gazebo's gpu_lidar returns NaN for rays that hit nothing, and the
      // published cloud still claims is_dense -- which makes PCL's VoxelGrid
      // index on NaN. Drop them here, once, for everyone downstream.
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        ++invalid;
        continue;
      }

      const double tau = pt.timestamp;
      if (first) {
        min_tau = max_tau = tau;
        first = false;
      } else {
        min_tau = std::min(min_tau, tau);
        max_tau = std::max(max_tau, tau);
      }

      PointT o = pt;

      if (opts_.enabled) {
        bool extrap = false;
        const double stamp = scan_stamp + tau;
        if (!PoseBuffer::interpolateFrom(poses, stamp, &cursor, &p_tau, &extrap)) {
          p_tau = pose_end;  // no odometry yet: leave the point where it is
          extrap = true;
        }
        extrapolated = extrapolated || extrap;

        const Eigen::Matrix4d M = T_end_inv * p_tau.matrix() * T_bl;
        const Eigen::Vector3d p_lidar(pt.x, pt.y, pt.z);
        const Eigen::Vector3d p = (M * p_lidar.homogeneous()).head<3>();

        // Only the motion correction, not the base<->lidar frame change: the
        // reference is where the point would sit if the body had stood still.
        const Eigen::Vector3d p_ref = (T_bl * p_lidar.homogeneous()).head<3>();
        const double d = (p - p_ref).norm();
        motion_sum += d;
        motion_max = std::max(motion_max, d);

        o.x = static_cast<float>(p.x());
        o.y = static_cast<float>(p.y());
        o.z = static_cast<float>(p.z());
      } else {
        // Still move the cloud from the lidar frame into the body frame, so
        // keyframes are consistent whether or not deskewing is on.
        const Eigen::Vector3d p = (T_bl * Eigen::Vector3d(pt.x, pt.y, pt.z).homogeneous()).head<3>();
        o.x = static_cast<float>(p.x());
        o.y = static_cast<float>(p.y());
        o.z = static_cast<float>(p.z());
      }

      out->points.push_back(o);
    }

    out->is_dense = true;
    if (stats) {
      stats->points = out->size();
      stats->invalid = invalid;
      stats->span = max_tau - min_tau;
      stats->max_motion = motion_max;
      stats->mean_motion = out->empty() ? 0.0 : motion_sum / static_cast<double>(out->size());
      stats->extrapolated = extrapolated;
    }
    return out;
  }

private:
  DeskewOptions opts_;
};

}  // namespace pgo
}  // namespace vector_mapping

#endif  // VECTOR_MAPPING_PGO_DESKEWER_HPP
