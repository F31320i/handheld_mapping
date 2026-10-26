#ifndef VECTOR_MAPPING__MAP_ACCUMULATOR_HPP_
#define VECTOR_MAPPING__MAP_ACCUMULATOR_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

namespace vector_mapping
{

/// Integer coordinates of one voxel of the accumulated map.
struct VoxelKey
{
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept
  {
    // Standard spatial hash: three large primes, one per axis.
    const auto hx = static_cast<std::uint32_t>(key.x) * 73856093u;
    const auto hy = static_cast<std::uint32_t>(key.y) * 19349663u;
    const auto hz = static_cast<std::uint32_t>(key.z) * 83492791u;
    return static_cast<std::size_t>(hx ^ hy ^ hz);
  }
};

struct VoxelKeyEqual
{
  bool operator()(const VoxelKey & a, const VoxelKey & b) const noexcept
  {
    return a.x == b.x && a.y == b.y && a.z == b.z;
  }
};

/**
 * @brief Rebuilds fast_limo's map from the outside, asynchronously.
 *
 * fast_limo publishes, on /fast_limo/pointcloud, the scan it has just registered
 * into its own world frame. That message is byte for byte the cloud it hands to
 * its internal octree (Localizer.cpp: `final_scan = mapped_scan`, then
 * `map.add(mapped_scan, ...)`), so concatenating those scans reproduces what
 * fast_limo itself has accumulated. fast_limo never publishes the octree and
 * exposes no accessor for it.
 *
 * The scans are accumulated in a voxel-keyed hash grid, which keeps at most one
 * point per voxel and therefore keeps the map bounded and evenly sampled. No
 * optimisation of any kind happens here: this is the skeleton that loop closure
 * and pose-graph optimisation will later plug into.
 *
 * The subscription callback only moves the incoming message into a bounded
 * queue; a worker thread does the PCL conversion and the grid insertion, and
 * publishes the accumulated map periodically.
 */
class MapAccumulator : public rclcpp::Node
{
public:
  explicit MapAccumulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~MapAccumulator() override;

  MapAccumulator(const MapAccumulator &) = delete;
  MapAccumulator & operator=(const MapAccumulator &) = delete;

private:
  // RGB, not XYZI: FAST-LIVO2's /cloud_registered carries the camera colour of
  // every point, and keeping it makes the accumulated map a coloured map. A
  // plain XYZI cloud (e.g. fast_limo's) still converts, just with rgb = 0.
  using PointT = pcl::PointXYZRGB;
  using PointCloudT = pcl::PointCloud<PointT>;
  using VoxelGrid = std::unordered_map<VoxelKey, PointT, VoxelKeyHash, VoxelKeyEqual>;

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void worker_loop();
  std::size_t insert_cloud(const PointCloudT & cloud);
  void publish_map();

  // Parameters.
  double voxel_leaf_;
  double publish_rate_;
  std::size_t max_points_;
  std::size_t queue_depth_;
  // Seconds of scans to throw away at the start of a run, timed from the first
  // scan received. 0 disables the gate.
  double warmup_seconds_;

  // Accumulated map. Only the worker thread writes it; the mutex keeps a future
  // service or parameter hook, and the destructor, on the safe side.
  VoxelGrid voxels_;
  std::mutex map_mutex_;

  // World frame of the accumulated map. fast_limo's own world frame, adopted
  // from the first scan or pose that carries one. Written before the worker
  // starts (constructor) or by the worker itself, so no lock is needed.
  std::string map_frame_;

  // Incoming scan queue, filled by the executor thread, drained by the worker.
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_queue_;
  std::size_t dropped_clouds_ = 0;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool stopping_ = false;
  std::thread worker_;

  // fast_limo's trajectory, kept for the back-end that will arrive later. Only
  // the odometry callback touches it, and callback groups are mutually
  // exclusive, so it needs no lock of its own.
  nav_msgs::msg::Path trajectory_;

  bool map_dirty_ = false;
  // Stamp of the newest scan folded into the map, used as the published cloud's
  // stamp. Only the worker thread touches these.
  builtin_interfaces::msg::Time last_scan_stamp_;
  bool has_scan_stamp_ = false;
  std::size_t publish_count_ = 0;

  // Warm-up window state, worker thread only. Anchored on the first scan's own
  // stamp rather than on a clock read, so it means "15 s of scans" whether the
  // stamps come from the host or from a replayed bag.
  rclcpp::Time warmup_start_;
  bool warmup_started_ = false;
  bool warmup_done_ = false;

  // Subscriptions and publishers. rclcpp only holds these weakly, so they have
  // to be kept alive here.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace vector_mapping

#endif  // VECTOR_MAPPING__MAP_ACCUMULATOR_HPP_
