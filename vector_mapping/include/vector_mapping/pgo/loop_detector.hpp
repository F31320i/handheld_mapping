// Loop-closure candidate search and geometric verification.
//
// Stage 1 is cheap and geometric: a candidate must be temporally distant but
// spatially close according to the current graph estimate. Stage 2 registers
// the two keyframes with GICP and accepts the result only if the aligned cloud
// actually explains the target -- good enough to keep a demo honest without
// pulling in a DBoW vocabulary.

#ifndef VECTOR_MAPPING_PGO_LOOP_DETECTOR_HPP
#define VECTOR_MAPPING_PGO_LOOP_DETECTOR_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

#include "vector_mapping/pgo/keyframe_manager.hpp"
#include "vector_mapping/pgo/types.hpp"

namespace vector_mapping
{
namespace pgo
{

struct LoopOptions
{
  bool enabled = true;
  double search_radius = 3.0;    // m allowed between the two prior poses
  double max_yaw_deg = 45.0;     // prior yaw disagreement we are willing to fix
  double min_time_gap = 20.0;    // s between the two keyframes
  int min_index_gap = 15;        // keyframes; nearer neighbours are already linked
  // Being spatially close is not enough: on a straight run the keyframes are
  // close *because* they are the same corridor, and closing a loop there
  // re-optimises the graph every keyframe on pure GICP noise. A candidate only
  // counts as a revisit if the robot actually travelled a detour to get back:
  // the path along the trajectory must be long in absolute terms and long
  // compared to the straight line between the two poses.
  double min_path_gap = 3.0;     // m travelled between the two keyframes
  double min_path_ratio = 1.5;   // travelled / straight-line distance
  int max_candidates = 3;        // GICP attempts per query keyframe
  int local_map_neighbors = 5;   // keyframes fused into each registration target
  double icp_source_voxel = 0.10;
  double icp_target_voxel = 0.10;
  double icp_max_corr = 1.0;
  int icp_max_iter = 30;
  double inlier_dist = 0.20;     // m, below this a point counts as explained
  double max_fitness = 0.10;     // m, mean residual over the inliers
  double min_inlier_ratio = 0.60;
  double max_icp_shift = 1.5;    // m, refuse a correction larger than the prior
  double max_icp_yaw_deg = 60.0; // deg, same, for rotation
};

// Measured constraint: the older keyframe `from` and the newer one `to`, with
// `from_T_to` the pose of `to` expressed in `from`'s frame.
struct LoopConstraint
{
  int from = -1;
  int to = -1;
  Pose from_T_to;
  double fitness = 0.0;
  double inlier_ratio = 0.0;
};

struct Verification
{
  bool accepted = false;
  Pose from_T_to;
  double fitness = std::numeric_limits<double>::infinity();
  double inlier_ratio = 0.0;
  int inliers = 0;
  // Post-voxel sizes and whether GICP itself reported convergence. Kept so a
  // rejected candidate can be diagnosed from the log: fitness stays infinite
  // both when a cloud was too small to try and when the registration failed
  // outright, and those two want opposite fixes.
  std::size_t source_points = 0;
  std::size_t target_points = 0;
  bool converged = false;
  // Inliers at a deliberately loose 1 m radius. Tells apart "the two clouds
  // do not overlap at all" (loose also 0 -- wrong frames or no shared view)
  // from "they overlap but are shifted" (loose high, tight 0 -- the
  // registration converged to the wrong place, or the prior is a few
  // centimetres off).
  int loose_inliers = 0;

  // 0 is perfect; used to rank competing candidates.
  double score() const {return fitness * (2.0 - inlier_ratio);}
};

class LoopDetector
{
public:
  void configure(const LoopOptions & opts) {opts_ = opts;}
  const LoopOptions & options() const {return opts_;}

  // Keyframe indices worth a GICP attempt for query `i`, best first.
  std::vector<int> candidates(int i, const std::vector<Keyframe> & frames) const
  {
    std::vector<int> out;
    if (!opts_.enabled || i < 0 || i >= static_cast<int>(frames.size())) {
      return out;
    }

    const Keyframe & cur = frames[i];

    // Cumulative travelled distance along the front-end trajectory, so the
    // detour test below is O(1) per candidate instead of re-walking the path.
    // odom_pose, not pose: the optimisation moves `pose` around, while the
    // detour the robot actually drove is what decides whether this is a loop.
    std::vector<double> arc(i + 1, 0.0);
    for (int k = 1; k <= i; ++k) {
      arc[k] = arc[k - 1] + (frames[k].odom_pose.t - frames[k - 1].odom_pose.t).norm();
    }

    std::vector<std::pair<double, int>> scored;

    for (int j = 0; j < i - opts_.min_index_gap; ++j) {
      if (cur.stamp - frames[j].stamp < opts_.min_time_gap) {
        continue;
      }
      const Pose delta = frames[j].pose.inverse() * cur.pose;
      if (delta.t.norm() > opts_.search_radius) {
        continue;
      }
      if (delta.angle() * kRadToDeg > opts_.max_yaw_deg) {
        continue;
      }
      const double detour = arc[i] - arc[j];
      if (detour < opts_.min_path_gap) {
        continue;
      }
      const double straight = (cur.odom_pose.t - frames[j].odom_pose.t).norm();
      if (detour < opts_.min_path_ratio * std::max(straight, 0.5)) {
        continue;
      }
      scored.emplace_back(delta.t.norm(), j);
    }

    std::partial_sort(
      scored.begin(), scored.begin() + std::min<std::size_t>(scored.size(), opts_.max_candidates),
      scored.end());
    const std::size_t keep = std::min<std::size_t>(scored.size(), opts_.max_candidates);
    out.reserve(keep);
    for (std::size_t k = 0; k < keep; ++k) {
      out.push_back(scored[k].second);
    }
    return out;
  }

  // `source` is keyframe `to`'s cloud in its own body frame; `target` is the
  // local map around `from` expressed in `from`'s body frame. `guess` is the
  // prior from_T_to.
  Verification verify(
    const CloudT & source, const CloudT & target, const Pose & guess) const
  {
    Verification v;
    if (source.empty() || target.empty()) {
      return v;
    }

    CloudT::Ptr src = KeyframeManager::voxelize(source, opts_.icp_source_voxel);
    CloudT::Ptr tgt = KeyframeManager::voxelize(target, opts_.icp_target_voxel);
    v.source_points = src->size();
    v.target_points = tgt->size();
    if (src->size() < 50 || tgt->size() < 50) {
      return v;
    }

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(src);
    gicp.setInputTarget(tgt);
    gicp.setMaximumIterations(opts_.icp_max_iter);
    gicp.setMaxCorrespondenceDistance(opts_.icp_max_corr);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setCorrespondenceRandomness(20);

    CloudT aligned;
    gicp.align(aligned, guess.matrix().cast<float>());
    // Deliberately NOT gated on gicp.hasConverged(). PCL's GICP leaves that
    // flag false when its internal Kabsch solve throws on a degenerate
    // correspondence set -- it breaks out of the iteration and returns the
    // last transform, which on the first iteration is the prior. Rejecting on
    // the flag threw away perfectly usable results: on a 180 deg revisit every
    // candidate came back "converged=0", so the back end silently never closed
    // a loop. Everything below re-scores the returned transform against our
    // own inlier distance anyway, which is the check that actually matters.
    v.converged = gicp.hasConverged();

    // Re-score with our own metric rather than trusting GICP's fitness, which
    // is computed on its (possibly generous) correspondence subset.
    const Eigen::Matrix4d T = gicp.getFinalTransformation().cast<double>();
    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(tgt);

    double sum = 0.0;
    int inliers = 0;
    int loose_inliers = 0;
    for (const auto & pt : *src) {
      const Eigen::Vector3d p = (T * Eigen::Vector3d(pt.x, pt.y, pt.z).homogeneous()).head<3>();
      PointT q;
      q.x = static_cast<float>(p.x());
      q.y = static_cast<float>(p.y());
      q.z = static_cast<float>(p.z());
      std::vector<int> idx(1);
      std::vector<float> sq(1);
      if (tree.nearestKSearch(q, 1, idx, sq) < 1) {
        continue;
      }
      const double d = std::sqrt(static_cast<double>(sq[0]));
      if (d < 1.0) {
        ++loose_inliers;
      }
      if (d < opts_.inlier_dist) {
        ++inliers;
        sum += d;
      }
    }

    v.loose_inliers = loose_inliers;
    v.inliers = inliers;
    v.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(v.source_points);
    v.fitness = (inliers > 0) ? sum / static_cast<double>(inliers)
                              : std::numeric_limits<double>::infinity();
    v.from_T_to = Pose::fromMatrix(T);

    // A registration that disagrees with the prior by more than the search
    // radius was told to allow is far more likely to be a bad match than a
    // genuine correction; refuse it before it reaches the graph.
    const Pose correction = guess.inverse() * v.from_T_to;
    const bool plausible =
      correction.t.norm() <= opts_.max_icp_shift &&
      correction.angle() * kRadToDeg <= opts_.max_icp_yaw_deg;

    v.accepted = plausible && inliers > 0 &&
      v.inlier_ratio >= opts_.min_inlier_ratio &&
      v.fitness <= opts_.max_fitness;
    return v;
  }

private:
  LoopOptions opts_;
};

}  // namespace pgo
}  // namespace vector_mapping

#endif  // VECTOR_MAPPING_PGO_LOOP_DETECTOR_HPP
