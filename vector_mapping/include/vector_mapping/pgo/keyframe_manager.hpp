// Keyframe selection and local-map assembly.
//
// A keyframe is a deskewed, voxel-filtered scan stored in its own body frame
// together with the front-end pose that produced it. The front-end pose is
// kept forever as `odom_pose` so the graph can always rebuild an odometry
// edge; `pose` is the optimised estimate the mapper actually uses.

#ifndef VECTOR_MAPPING_PGO_KEYFRAME_MANAGER_HPP
#define VECTOR_MAPPING_PGO_KEYFRAME_MANAGER_HPP

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "vector_mapping/pgo/types.hpp"

namespace vector_mapping
{
namespace pgo
{

constexpr double kRadToDeg = 180.0 / M_PI;

struct KeyframeOptions
{
  double min_distance = 0.5;    // m between the last keyframe and the current pose
  double min_angle_deg = 10.0;  // deg of accumulated rotation
  double min_time = 0.5;        // s, never sample faster than this
  double max_time = 5.0;        // s, force a keyframe when nearly stationary
  double voxel_leaf = 0.05;     // m, resolution of the stored scan
  int min_points = 200;         // reject near-empty scans
};

// Every consumer of this class runs on the single back-end worker thread, so
// there is no locking here on purpose.
class KeyframeManager
{
public:
  void configure(const KeyframeOptions & opts)
  {
    opts_ = opts;
    last_stamp_ = -1.0;
    has_last_ = false;
  }

  // True when `pose` is far/rotated/late enough since the previous keyframe.
  bool accepts(const Pose & pose, double stamp) const
  {
    if (!has_last_) {
      return true;
    }
    if (stamp - last_stamp_ < opts_.min_time) {
      return false;
    }
    const Pose delta = last_pose_.inverse() * pose;
    if (delta.t.norm() >= opts_.min_distance) {
      return true;
    }
    if (delta.angle() * kRadToDeg >= opts_.min_angle_deg) {
      return true;
    }
    // Long gap: force a keyframe so the time span of the graph stays bounded,
    // but only if the platform actually moved. A parked robot generates an
    // endless stream of identical scans, and duplicate nodes add nothing but
    // map bloat.
    if (stamp - last_stamp_ >= opts_.max_time) {
      return delta.t.norm() > 0.05 || delta.angle() * kRadToDeg > 1.0;
    }
    return false;
  }

  // Downsamples and stores. Returns the new keyframe id, or -1 when rejected.
  int add(const Pose & pose, const Pose & odom_pose, double stamp, const CloudT & cloud)
  {
    auto down = voxelize(cloud, opts_.voxel_leaf);
    if (!down || static_cast<int>(down->size()) < opts_.min_points) {
      return -1;
    }
    Keyframe kf;
    kf.id = static_cast<int>(frames_.size());
    kf.stamp = stamp;
    kf.odom_pose = odom_pose;
    kf.pose = pose;
    kf.cloud = down;
    frames_.push_back(std::move(kf));

    last_pose_ = pose;
    last_stamp_ = stamp;
    has_last_ = true;
    return static_cast<int>(frames_.size()) - 1;
  }

  // The local map around keyframe `i`: neighbours [i-n, i+n] re-expressed in
  // keyframe i's frame and fused at `voxel`. This is the target cloud for a
  // loop-closure registration, and the source of a sliding-window map.
  CloudT::Ptr localMap(int i, int neighbors, double voxel) const
  {
    if (i < 0 || i >= static_cast<int>(frames_.size())) {
      return nullptr;
    }
    const int lo = std::max(0, i - neighbors);
    const int hi = std::min(static_cast<int>(frames_.size()) - 1, i + neighbors);
    const Pose inv_i = frames_[i].pose.inverse();

    auto acc = std::make_shared<CloudT>();
    for (int j = lo; j <= hi; ++j) {
      if (!frames_[j].cloud || frames_[j].cloud->empty()) {
        continue;
      }
      const Eigen::Matrix4f T = (inv_i * frames_[j].pose).matrix().cast<float>();
      auto moved = std::make_shared<CloudT>();
      pcl::transformPointCloud(*frames_[j].cloud, *moved, T);
      *acc += *moved;
    }
    return (voxel > 0.0) ? voxelize(*acc, voxel) : acc;
  }

  // Whole map in the graph frame, fused at `voxel` over keyframes [lo, hi].
  CloudT::Ptr globalMap(double voxel, int lo = 0, int hi = -1) const
  {
    if (frames_.empty()) {
      return std::make_shared<CloudT>();
    }
    if (hi < 0 || hi >= static_cast<int>(frames_.size())) {
      hi = static_cast<int>(frames_.size()) - 1;
    }
    auto acc = std::make_shared<CloudT>();
    for (int j = std::max(0, lo); j <= hi; ++j) {
      if (!frames_[j].cloud || frames_[j].cloud->empty()) {
        continue;
      }
      const Eigen::Matrix4f T = frames_[j].pose.matrix().cast<float>();
      auto moved = std::make_shared<CloudT>();
      pcl::transformPointCloud(*frames_[j].cloud, *moved, T);
      *acc += *moved;
    }
    return (voxel > 0.0) ? voxelize(*acc, voxel) : acc;
  }

  static CloudT::Ptr voxelize(const CloudT & in, double leaf)
  {
    if (in.empty()) {
      return std::make_shared<CloudT>();
    }
    if (leaf <= 0.0) {
      return std::make_shared<CloudT>(in);
    }
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
    vg.setInputCloud(std::make_shared<CloudT>(in));
    auto out = std::make_shared<CloudT>();
    vg.filter(*out);
    return out;
  }

  std::vector<Keyframe> & frames() {return frames_;}
  const std::vector<Keyframe> & frames() const {return frames_;}
  std::size_t size() const {return frames_.size();}
  bool empty() const {return frames_.empty();}

  void clear()
  {
    frames_.clear();
    has_last_ = false;
    last_stamp_ = -1.0;
  }

private:
  KeyframeOptions opts_;
  std::vector<Keyframe> frames_;
  Pose last_pose_;
  double last_stamp_ = -1.0;
  bool has_last_ = false;
};

}  // namespace pgo
}  // namespace vector_mapping

#endif  // VECTOR_MAPPING_PGO_KEYFRAME_MANAGER_HPP
