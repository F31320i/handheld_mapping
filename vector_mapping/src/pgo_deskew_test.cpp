// Synthetic check for the back-end deskewer.
//
// Gazebo renders every ray of a sweep in one physics tick, so in simulation the
// motion compensation is a no-op and cannot be validated by driving around.
// This test builds a known trajectory and a known static wall, produces the
// exact measurements a moving LiDAR would report (each point expressed in the
// LiDAR frame at *its own* instant), and checks that the deskewed cloud lands
// on the wall when placed at the end-of-sweep pose.
//
//   deskewed error  -> should collapse to float precision
//   naive error     -> should stay large (that is the smearing we remove)

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "vector_mapping/pgo/deskewer.hpp"
#include "vector_mapping/pgo/pose_buffer.hpp"
#include "vector_mapping/pgo/types.hpp"

namespace pgo = vector_mapping::pgo;

namespace
{

// Body pose at time t: straight-line translation plus a constant yaw rate.
pgo::Pose trajectory(double t)
{
  pgo::Pose p;
  p.t = Eigen::Vector3d(0.5 * t, 0.2 * t, 0.05 * t);
  p.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.8 * t, Eigen::Vector3d::UnitZ()));
  return p;
}

// A static vertical wall at x = 5, sampled on a grid.
std::vector<Eigen::Vector3d> wall()
{
  std::vector<Eigen::Vector3d> pts;
  for (int iy = -6; iy <= 6; ++iy) {
    for (int iz = -4; iz <= 4; ++iz) {
      pts.emplace_back(5.0, 0.25 * iy, 0.25 * iz);
    }
  }
  return pts;
}

Eigen::Matrix4d extrinsic()
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R =
    (Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitY())).toRotationMatrix();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = Eigen::Vector3d(0.033234, 0.0, 0.539234);
  return T;
}

struct Result
{
  double mean = 0.0;
  double max = 0.0;
};

Result compare(const pgo::CloudT & cloud, const std::vector<Eigen::Vector3d> & expected)
{
  Result r;
  if (cloud.size() != expected.size()) {
    r.mean = r.max = 1e9;
    return r;
  }
  double sum = 0.0;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const Eigen::Vector3d p(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    const double e = (p - expected[i]).norm();
    sum += e;
    if (e > r.max) {
      r.max = e;
    }
  }
  r.mean = sum / static_cast<double>(cloud.size());
  return r;
}

}  // namespace

int main()
{
  constexpr double kSweep = 0.1;  // 100 ms sweep, like a real Mid-360

  // Odometry, 100 Hz, exactly the trajectory above.
  pgo::PoseBuffer buffer(5000);
  for (int k = -100; k <= 200; ++k) {
    const double t = 0.01 * static_cast<double>(k);
    buffer.add(t, trajectory(t));
  }
  const auto snapshot = buffer.snapshot();

  const Eigen::Matrix4d T_bl = extrinsic();
  const std::vector<Eigen::Vector3d> world = wall();

  // Measurements: wall point seen at instant tau, in the LiDAR frame.
  const std::size_t n = world.size();
  pgo::CloudT raw;
  raw.resize(n);
  double max_tau = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double tau = kSweep * static_cast<double>(i) / static_cast<double>(n - 1);
    max_tau = tau;
    const Eigen::Matrix4d T_world_base = trajectory(tau).matrix();
    const Eigen::Vector4d p_lidar =
      (T_bl.inverse() * T_world_base.inverse() * world[i].homogeneous());
    raw.points[i].x = static_cast<float>(p_lidar.x());
    raw.points[i].y = static_cast<float>(p_lidar.y());
    raw.points[i].z = static_cast<float>(p_lidar.z());
    raw.points[i].intensity = 1.0f;
    raw.points[i].timestamp = tau;
  }

  const double t_end = max_tau;
  pgo::Pose pose_end;
  bool extrapolated = false;
  pgo::PoseBuffer::interpolate(snapshot, t_end, &pose_end, &extrapolated);

  // Expected: the wall expressed in the body frame at the end of the sweep.
  std::vector<Eigen::Vector3d> expected(world.size());
  const Eigen::Matrix4d T_end_inv = pose_end.inverse().matrix();
  for (std::size_t i = 0; i < world.size(); ++i) {
    expected[i] = (T_end_inv * world[i].homogeneous()).head<3>();
  }

  pgo::DeskewOptions opts;
  opts.enabled = true;
  opts.T_base_lidar = T_bl;
  pgo::Deskewer deskewer(opts);

  pgo::DeskewStats stats;
  auto corrected = deskewer.deskew(raw, 0.0, snapshot, pose_end, &stats);
  const Result deskewed = compare(*corrected, expected);

  // Naive: take the cloud as measured and place it rigidly at the end pose.
  pgo::CloudT naive;
  naive.resize(world.size());
  for (std::size_t i = 0; i < world.size(); ++i) {
    const Eigen::Vector4d p =
      pose_end.matrix() * (T_bl * Eigen::Vector3d(
        raw.points[i].x, raw.points[i].y, raw.points[i].z).homogeneous());
    naive.points[i].x = static_cast<float>(p.x());
    naive.points[i].y = static_cast<float>(p.y());
    naive.points[i].z = static_cast<float>(p.z());
  }
  const Result naive_err = compare(naive, world);

  std::printf("sweep %.3f s, %zu points, extrapolated=%d\n",
    stats.span, stats.points, static_cast<int>(extrapolated || stats.extrapolated));
  std::printf("deskew displacement: mean %.4f m, max %.4f m\n",
    stats.mean_motion, stats.max_motion);
  std::printf("error vs wall (body frame at sweep end):\n");
  std::printf("  deskewed : mean %.6f m, max %.6f m\n", deskewed.mean, deskewed.max);
  std::printf("  naive    : mean %.6f m, max %.6f m\n", naive_err.mean, naive_err.max);

  const bool ok = deskewed.max < 1e-3 && naive_err.mean > 0.05;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
