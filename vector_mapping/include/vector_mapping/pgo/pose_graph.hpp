// GTSAM pose graph over the keyframes.
//
// The graph is small (hundreds of nodes in a demo run) so it is rebuilt and
// batch-optimised on demand instead of using ISAM2: simpler to reason about,
// and the cost of one Levenberg-Marquardt solve is far below the GICP
// verification that triggered it.

#ifndef VECTOR_MAPPING_PGO_POSE_GRAPH_HPP
#define VECTOR_MAPPING_PGO_POSE_GRAPH_HPP

#include <array>
#include <cmath>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>

#include "vector_mapping/pgo/types.hpp"

namespace vector_mapping
{
namespace pgo
{

struct GraphOptions
{
  // Standard deviations, translation first: [x, y, z, roll, pitch, yaw].
  std::array<double, 6> odom_sigmas{{0.02, 0.02, 0.02, 0.01, 0.01, 0.02}};
  std::array<double, 6> loop_sigmas{{0.01, 0.01, 0.01, 0.005, 0.005, 0.01}};
  std::array<double, 6> prior_sigmas{{1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3}};
  double loop_huber = 1.0;   // Cauchy kernel width on loop factors
  int max_iterations = 50;
};

// One absolute position measurement, already expressed in the graph's own
// frame and attributed to a keyframe. Translation only: a GNSS fix says where
// the robot is, not which way it points.
struct Anchor
{
  int id = -1;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  std::array<double, 3> sigmas{{0.02, 0.02, 0.05}};
  // Cauchy width in sigmas; 0 leaves the factor unrobustified. See the
  // comment on graph_gnss_robust in pgo_backend.yaml: this one has to be wide,
  // because unlike a loop it is supposed to *cause* a large residual.
  double robust = 0.0;
};

class PoseGraph
{
public:
  void configure(const GraphOptions & opts)
  {
    opts_ = opts;
    reset();
  }

  void reset()
  {
    nodes_.clear();
    odom_edges_.clear();
    loop_edges_.clear();
    anchors_.clear();
  }

  // Registers the keyframe as a graph node. Must be called in increasing id
  // order; the initial value is the current estimate.
  void addNode(int id, const Pose & pose)
  {
    nodes_.push_back(Node{id, pose});
  }

  // `sigma_scale` widens this edge's noise relative to odom_sigmas. It exists
  // for the one edge per survey that spans a front-end re-acquisition: the
  // relative pose across a pose jump is the size of the jump, not a distance
  // walked, so at the normal 8 mm it would be the stiffest constraint in the
  // graph. It must still be *an* edge -- dropping it disconnects the graph and
  // the next optimisation fails outright on a singular system.
  void addOdomEdge(int from, int to, const Pose & from_T_to, double sigma_scale = 1.0)
  {
    odom_edges_.push_back(Edge{from, to, from_T_to, sigma_scale});
  }

  void addLoopEdge(int from, int to, const Pose & from_T_to)
  {
    loop_edges_.push_back(Edge{from, to, from_T_to, 1.0});
  }

  void addAnchor(const Anchor & anchor)
  {
    anchors_.push_back(anchor);
  }

  // Used to undo an anchor whose optimisation failed.
  void removeLastAnchor()
  {
    if (!anchors_.empty()) {
      anchors_.pop_back();
    }
  }

  // Replace every anchor at once. The back end re-expresses all of them in the
  // graph frame whenever its estimate of the frame offset is refined, which is
  // a single call rather than a remove/add per anchor.
  void setAnchors(std::vector<Anchor> anchors)
  {
    anchors_ = std::move(anchors);
  }

  std::size_t anchorCount() const {return anchors_.size();}

  // Used to undo an edge whose optimisation failed.
  void removeLastLoopEdge()
  {
    if (!loop_edges_.empty()) {
      loop_edges_.pop_back();
    }
  }

  bool hasLoops() const {return !loop_edges_.empty();}
  std::size_t nodeCount() const {return nodes_.size();}

  // Batch solve. On success `poses` is filled with the optimised estimate for
  // every node, in node order. Returns the RMS displacement applied.
  bool optimise(std::vector<Pose> * poses, double * rms_shift)
  {
    if (poses) {
      poses->clear();
    }
    if (nodes_.empty()) {
      return false;
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // Gauge: pin the first node to its front-end estimate so that the graph
    // frame stays aligned with the odometry frame (and therefore with the
    // front end's map -> odom -> base_link chain).
    const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(toSigmas(opts_.prior_sigmas));
    graph.addPrior(nodes_.front().id, toGtsam(nodes_.front().pose), prior_noise);

    for (const auto & n : nodes_) {
      initial.insert(n.id, toGtsam(n.pose));
    }

    const auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(toSigmas(opts_.odom_sigmas));
    for (const auto & e : odom_edges_) {
      gtsam::SharedNoiseModel noise = odom_noise;
      if (e.sigma_scale != 1.0) {
        std::array<double, 6> s = opts_.odom_sigmas;
        for (auto & v : s) {
          v *= e.sigma_scale;
        }
        noise = gtsam::noiseModel::Diagonal::Sigmas(toSigmas(s));
      }
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        e.from, e.to, toGtsam(e.relative), noise);
    }

    const auto loop_prior = gtsam::noiseModel::Diagonal::Sigmas(toSigmas(opts_.loop_sigmas));
    const auto loop_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(opts_.loop_huber), loop_prior);
    for (const auto & e : loop_edges_) {
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        e.from, e.to, toGtsam(e.relative), loop_noise);
    }

    // Absolute position measurements (GNSS-like). PoseTranslationPrior
    // measures the translation component of the pose against a fixed point,
    // which is exactly a "the robot was here in the map frame" factor: the
    // orientation of the keyframe is left completely free, as it should be,
    // because a position fix says nothing about heading.
    for (const auto & a : anchors_) {
      gtsam::SharedNoiseModel anchor_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(a.sigmas[0], a.sigmas[1], a.sigmas[2]));
      if (a.robust > 0.0) {
        anchor_noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Cauchy::Create(a.robust), anchor_noise);
      }
      graph.emplace_shared<gtsam::PoseTranslationPrior<gtsam::Pose3>>(
        a.id, gtsam::Point3(a.p), anchor_noise);
    }

    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(opts_.max_iterations);
    params.setRelativeErrorTol(1e-5);
    params.setAbsoluteErrorTol(1e-5);
    params.setErrorTol(1e-5);

    gtsam::Values result;
    try {
      result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
    } catch (const std::exception &) {
      return false;
    }
    if (result.size() != initial.size()) {
      return false;
    }

    double sq = 0.0;
    int n = 0;
    std::vector<Pose> out;
    out.reserve(nodes_.size());
    for (const auto & node : nodes_) {
      if (!result.exists(node.id)) {
        out.push_back(node.pose);
        continue;
      }
      const Pose p = fromGtsam(result.at<gtsam::Pose3>(node.id));
      sq += (p.t - node.pose.t).squaredNorm();
      ++n;
      out.push_back(p);
    }
    if (n == 0) {
      return false;
    }

    if (rms_shift) {
      *rms_shift = std::sqrt(sq / static_cast<double>(n));
    }
    if (poses) {
      *poses = std::move(out);
    }
    return true;
  }

private:
  struct Node
  {
    int id;
    Pose pose;
  };
  struct Edge
  {
    int from;
    int to;
    Pose relative;
    double sigma_scale = 1.0;
  };

  static gtsam::Pose3 toGtsam(const Pose & p)
  {
    return gtsam::Pose3(gtsam::Rot3(p.q), gtsam::Point3(p.t));
  }

  static Pose fromGtsam(const gtsam::Pose3 & p)
  {
    Pose out;
    out.t = p.translation();
    out.q = p.rotation().toQuaternion();
    out.q.normalize();
    return out;
  }

  // GTSAM orders a Pose3 tangent as [rotation, translation]; the config files
  // are written translation-first because that is how people read them.
  static gtsam::Vector6 toSigmas(const std::array<double, 6> & s)
  {
    gtsam::Vector6 v;
    v << s[3], s[4], s[5], s[0], s[1], s[2];
    return v;
  }

  GraphOptions opts_;
  std::vector<Node> nodes_;
  std::vector<Edge> odom_edges_;
  std::vector<Edge> loop_edges_;
  std::vector<Anchor> anchors_;
};

}  // namespace pgo
}  // namespace vector_mapping

#endif  // VECTOR_MAPPING_PGO_POSE_GRAPH_HPP
