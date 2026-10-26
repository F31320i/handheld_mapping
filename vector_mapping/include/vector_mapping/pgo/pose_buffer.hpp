// Ring buffer of front-end poses, with interpolation.
//
// The back end deskews with the *fused* odometry rather than by integrating
// raw IMU: it is already corrected by the LiDAR update, and it keeps the back
// end free of any IMU bias handling or extrinsic guessing.

#ifndef VECTOR_MAPPING_PGO_POSE_BUFFER_HPP
#define VECTOR_MAPPING_PGO_POSE_BUFFER_HPP

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

#include "vector_mapping/pgo/types.hpp"

namespace vector_mapping
{
namespace pgo
{

class PoseBuffer
{
public:
  struct Entry
  {
    double stamp;
    Pose pose;
  };

  explicit PoseBuffer(std::size_t capacity = 5000) : capacity_(capacity) {}

  // Out-of-order samples (older than the newest one) are dropped: the front end
  // can publish a stale pose right after a scan update.
  void add(double stamp, const Pose & pose)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!buf_.empty() && stamp <= buf_.back().stamp) {
      return;
    }
    buf_.push_back(Entry{stamp, pose});
    while (buf_.size() > capacity_) {
      buf_.pop_front();
    }
    cv_.notify_all();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    buf_.clear();
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return buf_.empty();
  }

  bool latest(std::size_t * count, double * stamp) const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (buf_.empty()) {
      return false;
    }
    *count = buf_.size();
    *stamp = buf_.back().stamp;
    return true;
  }

  // The newest sample, without interpolating. The back end needs this when its
  // input is a cloud the front end already baked into the world frame: the
  // pose to undo is the last one the front end reported, not the one covering
  // the cloud's (later) publishing stamp.
  bool newest(Entry * out) const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (buf_.empty()) {
      return false;
    }
    *out = buf_.back();
    return true;
  }

  // Block until the newest sample is at or past `stamp`.
  bool waitFor(double stamp, double timeout_s)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    return cv_.wait_for(lock, std::chrono::duration<double>(timeout_s), [&] {
      return !buf_.empty() && buf_.back().stamp >= stamp;
    });
  }

  std::vector<Entry> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return {buf_.begin(), buf_.end()};
  }

  // Linear interpolation on translation, slerp on rotation.
  // Outside the buffer the nearest endpoint is returned and *extrapolated is
  // set, so the caller can count how often it happens.
  static bool interpolate(
    const std::vector<Entry> & snap, double stamp, Pose * out, bool * extrapolated)
  {
    if (snap.empty()) {
      return false;
    }
    if (atEndpoints(snap, stamp, out, extrapolated)) {
      return true;
    }
    const std::size_t hi = locate(snap, stamp, nullptr);
    blend(snap[hi - 1], snap[hi], stamp, out);
    return true;
  }

  // Same, but reuses `cursor` across calls. Only valid when successive queries
  // are non-decreasing in time (true for the points of one Livox sweep); an
  // out-of-order query transparently falls back to a binary search.
  static bool interpolateFrom(
    const std::vector<Entry> & snap, double stamp, std::size_t * cursor,
    Pose * out, bool * extrapolated)
  {
    if (snap.empty()) {
      return false;
    }
    if (atEndpoints(snap, stamp, out, extrapolated)) {
      return true;
    }
    const std::size_t hi = locate(snap, stamp, cursor);
    blend(snap[hi - 1], snap[hi], stamp, out);
    return true;
  }

private:
  // Handles the clamping cases shared by both interpolators. Returns true when
  // the query was answered without needing a bracketing pair.
  static bool atEndpoints(
    const std::vector<Entry> & snap, double stamp, Pose * out, bool * extrapolated)
  {
    *extrapolated = false;
    if (stamp <= snap.front().stamp) {
      *out = snap.front().pose;
      *extrapolated = stamp < snap.front().stamp;
      return true;
    }
    if (stamp >= snap.back().stamp) {
      *out = snap.back().pose;
      *extrapolated = stamp > snap.back().stamp;
      return true;
    }
    return false;
  }

  // Index of the first entry at or after `stamp`, so that snap[hi-1].stamp <
  // stamp <= snap[hi]. Guaranteed to be in [1, size-1] because the endpoints
  // were handled already.
  static std::size_t locate(
    const std::vector<Entry> & snap, double stamp, std::size_t * cursor)
  {
    const std::size_t n = snap.size();
    if (cursor && *cursor > 0 && *cursor < n &&
      snap[*cursor].stamp >= stamp && snap[*cursor - 1].stamp <= stamp)
    {
      return *cursor;  // still brackets the query
    }

    std::size_t hi;
    if (cursor && *cursor > 0 && *cursor < n && snap[*cursor].stamp < stamp) {
      hi = *cursor;  // query moved forward: walk instead of searching
      while (hi + 1 < n && snap[hi].stamp < stamp) {
        ++hi;
      }
    } else {
      auto it = std::lower_bound(
        snap.begin() + 1, snap.end(), stamp,
        [](const Entry & e, double s) { return e.stamp < s; });
      hi = static_cast<std::size_t>(it - snap.begin());
    }
    if (cursor) {
      *cursor = hi;
    }
    return hi;
  }

  static void blend(const Entry & lo, const Entry & hi, double stamp, Pose * out)
  {
    const double span = hi.stamp - lo.stamp;
    const double alpha = (span > 1e-9) ? (stamp - lo.stamp) / span : 0.0;

    Eigen::Quaterniond q_lo = lo.pose.q;
    Eigen::Quaterniond q_hi = hi.pose.q;
    if (q_lo.dot(q_hi) < 0.0) {
      q_hi.coeffs() *= -1.0;  // shortest path
    }
    out->q = q_lo.slerp(alpha, q_hi).normalized();
    out->t = lo.pose.t + alpha * (hi.pose.t - lo.pose.t);
  }

  std::size_t capacity_;
  std::deque<Entry> buf_;
  mutable std::mutex mtx_;
  std::condition_variable cv_;
};

}  // namespace pgo
}  // namespace vector_mapping

#endif  // VECTOR_MAPPING_PGO_POSE_BUFFER_HPP
