// Map assembly: turns optimised keyframes into a publishable cloud / PCD.
//
// Incremental on purpose. A handheld survey runs for tens of minutes and the
// map is republished every couple of seconds; rebuilding it from every
// keyframe on each publish makes the work grow with the square of the session
// length -- 10 minutes at 0.2 m keyframes is ~3000 keyframes, i.e. tens of
// millions of points to re-transform and re-voxelise every two seconds, and
// the back end eventually spends all its time rebuilding a map nobody looks
// at. Instead each keyframe is fused exactly once, when it is accepted, into a
// voxel hash that keeps the running position / intensity / colour sums and a
// count per cell; publishing only materialises those cells (mean position,
// mean colour).
//
// A pose-graph optimisation moves most keyframes at once and a fused cell
// cannot be un-fused, so the caller has to call rebuild() afterwards. That is
// the only O(all points) operation left, and it happens once per accepted
// loop rather than once per publish.
//
// The result is the same kind of map the previous rebuild-every-time
// implementation produced (one centroid per leaf-sized cell, colour averaged
// the way pcl::VoxelGrid does it). It is equivalent, not bit-identical: PCL
// aligns its grid to the cloud's own minimum while this hash aligns it to the
// origin, so cells sitting on a leaf boundary can be grouped differently by up
// to a fraction of a leaf. Nothing downstream depends on the exact grouping.

#ifndef VECTOR_MAPPING_PGO_MAP_ASSEMBLER_HPP
#define VECTOR_MAPPING_PGO_MAP_ASSEMBLER_HPP

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/io/pcd_io.h>

#include "vector_mapping/pgo/keyframe_manager.hpp"
#include "vector_mapping/pgo/types.hpp"

namespace vector_mapping
{
namespace pgo
{

class MapAssembler
{
public:
  explicit MapAssembler(double voxel_leaf = 0.05) {setVoxel(voxel_leaf);}

  // Changing the resolution invalidates everything fused so far.
  void setVoxel(double v)
  {
    v = (v > 0.0) ? v : 0.05;
    if (v == voxel_ && !cells_.empty()) {
      return;
    }
    voxel_ = v;
    clear();
  }

  double voxel() const {return voxel_;}

  void clear()
  {
    cells_.clear();
    points_ = 0;
    materialised_.reset();
  }

  bool empty() const {return cells_.empty();}
  std::size_t pointCount() const {return points_;}
  std::size_t cellCount() const {return cells_.size();}

  // Fuse one keyframe, transformed by its *current* pose. `cloud` is the
  // keyframe's own body-frame scan, so callers pass frames[id].cloud and
  // frames[id].pose and nothing has to be copied.
  void add(const Pose & pose, const CloudT & cloud)
  {
    if (cloud.empty()) {
      return;
    }
    const Eigen::Matrix4f T = pose.matrix().cast<float>();
    const float inv_leaf = static_cast<float>(1.0 / voxel_);
    for (const auto & p : cloud.points) {
      const float x = T(0, 0) * p.x + T(0, 1) * p.y + T(0, 2) * p.z + T(0, 3);
      const float y = T(1, 0) * p.x + T(1, 1) * p.y + T(1, 2) * p.z + T(1, 3);
      const float z = T(2, 0) * p.x + T(2, 1) * p.y + T(2, 2) * p.z + T(2, 3);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      Cell & c = cells_[keyFor(x, y, z, inv_leaf)];
      c.sx += x;
      c.sy += y;
      c.sz += z;
      c.si += p.intensity;
      c.sr += p.r;
      c.sg += p.g;
      c.sb += p.b;
      ++c.n;
    }
    points_ += cloud.size();
    materialised_.reset();
  }

  // Re-fuse every keyframe. Required after a pose-graph optimisation, which
  // changes the poses of keyframes that have already been fused.
  void rebuild(const std::vector<Keyframe> & frames)
  {
    clear();
    for (const auto & kf : frames) {
      if (kf.cloud) {
        add(kf.pose, *kf.cloud);
      }
    }
  }

  // The fused map, materialised once and cached until the next add/rebuild.
  CloudT::Ptr build()
  {
    if (!materialised_) {
      materialise();
    }
    return materialised_;
  }

  static bool savePcd(const CloudT & cloud, const std::string & path)
  {
    if (cloud.empty()) {
      return false;
    }
    return pcl::io::savePCDFileBinary(path, cloud) == 0;
  }

private:
  struct Cell
  {
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    float si = 0.0f;
    float sr = 0.0f, sg = 0.0f, sb = 0.0f;
    std::uint32_t n = 0;
  };

  struct CellKey
  {
    std::int32_t x = 0, y = 0, z = 0;
    bool operator==(const CellKey & o) const {return x == o.x && y == o.y && z == o.z;}
  };

  struct CellKeyHash
  {
    std::size_t operator()(const CellKey & k) const
    {
      // The usual prime-multiplier mix; the keys are small integers, so this
      // spreads them adequately without a real hash function.
      const std::uint64_t h =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.x)) * 73856093ull) ^
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.y)) * 19349663ull) ^
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.z)) * 83492791ull);
      return static_cast<std::size_t>(h);
    }
  };

  CellKey keyFor(float x, float y, float z, float inv_leaf) const
  {
    CellKey k;
    k.x = static_cast<std::int32_t>(std::floor(x * inv_leaf));
    k.y = static_cast<std::int32_t>(std::floor(y * inv_leaf));
    k.z = static_cast<std::int32_t>(std::floor(z * inv_leaf));
    return k;
  }

  void materialise()
  {
    auto out = std::make_shared<CloudT>();
    out->points.reserve(cells_.size());
    for (const auto & kv : cells_) {
      const Cell & c = kv.second;
      if (c.n == 0) {
        continue;
      }
      const float inv_n = 1.0f / static_cast<float>(c.n);
      PointT p{};
      p.x = c.sx * inv_n;
      p.y = c.sy * inv_n;
      p.z = c.sz * inv_n;
      p.data[3] = 1.0f;
      p.intensity = c.si * inv_n;
      const auto r = static_cast<std::uint8_t>(std::lround(c.sr * inv_n));
      const auto g = static_cast<std::uint8_t>(std::lround(c.sg * inv_n));
      const auto b = static_cast<std::uint8_t>(std::lround(c.sb * inv_n));
      p.rgba = (static_cast<std::uint32_t>(0xFFu) << 24) |
        (static_cast<std::uint32_t>(r) << 16) |
        (static_cast<std::uint32_t>(g) << 8) |
        static_cast<std::uint32_t>(b);
      out->points.push_back(p);
    }
    out->width = static_cast<std::uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = true;
    materialised_ = out;
  }

  double voxel_ = 0.05;
  std::size_t points_ = 0;
  std::unordered_map<CellKey, Cell, CellKeyHash> cells_;
  CloudT::Ptr materialised_;
};

}  // namespace pgo
}  // namespace vector_mapping

#endif  // VECTOR_MAPPING_PGO_MAP_ASSEMBLER_HPP
