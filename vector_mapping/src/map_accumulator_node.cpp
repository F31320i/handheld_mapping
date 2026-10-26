#include "vector_mapping/map_accumulator.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>

namespace vector_mapping
{

MapAccumulator::MapAccumulator(const rclcpp::NodeOptions & options)
: Node("map_accumulator", options)
{
  const auto input_cloud_topic = this->declare_parameter<std::string>(
    "input_cloud_topic", "/fast_limo/pointcloud");
  // fast_limo's /fast_limo/state is remapped to /odometry/lio by
  // localization.launch.xml; /odometry/lio carries the raw LIO odometry.
  const auto input_odom_topic = this->declare_parameter<std::string>(
    "input_odom_topic", "/odometry/lio");
  const auto output_map_topic = this->declare_parameter<std::string>(
    "output_map_topic", "/vector_mapping/map_accumulated");
  const auto output_path_topic = this->declare_parameter<std::string>(
    "output_path_topic", "/vector_mapping/trajectory");

  // Empty means "whatever frame the scans arrive in", which is fast_limo's own
  // world frame (the `world` parameter of fast_limo.yaml, i.e. odom).
  map_frame_ = this->declare_parameter<std::string>("map_frame", "");

  // Voxel edge length of the accumulated map, in metres. fast_limo's octree
  // already runs with min_extent 0.1 and downsampling enabled, so the same
  // order of magnitude gives a comparable sampling density.
  voxel_leaf_ = this->declare_parameter<double>("voxel_leaf", 0.1);
  publish_rate_ = this->declare_parameter<double>("publish_rate", 1.0);
  // 0 disables the cap. 20 M voxels is roughly 0.6 GB of hash-grid overhead.
  max_points_ = static_cast<std::size_t>(
    this->declare_parameter<int>("max_points", 20000000));
  queue_depth_ = static_cast<std::size_t>(
    this->declare_parameter<int>("queue_depth", 20));
  // 0 keeps the old behaviour: accumulate everything from the first scan.
  warmup_seconds_ = this->declare_parameter<double>("warmup_seconds", 0.0);

  if (voxel_leaf_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "voxel_leaf must be positive, using 0.1 m");
    voxel_leaf_ = 0.1;
  }
  if (publish_rate_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate must be positive, using 1.0 Hz");
    publish_rate_ = 1.0;
  }
  if (queue_depth_ == 0) {
    queue_depth_ = 1;
  }
  if (warmup_seconds_ < 0.0) {
    RCLCPP_WARN(get_logger(), "warmup_seconds must not be negative, using 0");
    warmup_seconds_ = 0.0;
  }

  // Transient local: a late-joining RViz still receives the latest map even
  // when the robot has stopped and nothing new is being accumulated.
  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_map_topic, rclcpp::QoS(1).transient_local());
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    output_path_topic, rclcpp::QoS(1).transient_local());

  // fast_limo publishes with rclcpp::QoS(1), i.e. reliable + keep_last(1).
  // Same reliability here, with a deeper queue: this callback is cheap, it
  // only enqueues, so it will not actually back up.
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic, rclcpp::QoS(10).reliable(),
    std::bind(&MapAccumulator::pointcloud_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic, rclcpp::QoS(10).reliable(),
    std::bind(&MapAccumulator::odometry_callback, this, std::placeholders::_1));

  worker_ = std::thread(&MapAccumulator::worker_loop, this);

  RCLCPP_INFO(
    get_logger(),
    "Accumulating %s into %s (voxel %.3f m, publishing at %.1f Hz)",
    input_cloud_topic.c_str(), output_map_topic.c_str(), voxel_leaf_, publish_rate_);
  if (warmup_seconds_ > 0.0) {
    RCLCPP_INFO(
      get_logger(), "Warm-up gate: the first %.1f s of scans will be discarded",
      warmup_seconds_);
  }
}

MapAccumulator::~MapAccumulator()
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stopping_ = true;
  }
  queue_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void MapAccumulator::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (cloud_queue_.size() >= queue_depth_) {
      cloud_queue_.pop_front();
      ++dropped_clouds_;
    }
    cloud_queue_.push_back(std::move(msg));
  }
  queue_cv_.notify_one();
}

void MapAccumulator::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;

  if (trajectory_.poses.empty()) {
    // The first pose is the closest thing to fast_limo's world origin: the node
    // starts with an identity state and the robot is at rest when the
    // simulation comes up, so this should read as roughly zero.
    RCLCPP_INFO(
      get_logger(),
      "First fast_limo pose (its world origin baseline): x=%.3f y=%.3f z=%.3f, frame '%s'",
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
      pose.header.frame_id.c_str());
  }

  trajectory_.header = pose.header;
  trajectory_.poses.push_back(pose);
  path_pub_->publish(trajectory_);
}

void MapAccumulator::worker_loop()
{
  using Clock = std::chrono::steady_clock;
  // Wall clock on purpose: the publish cadence should not stall when the
  // simulation clock is paused, and it does not depend on use_sim_time.
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / publish_rate_));
  auto last_publish = Clock::now();

  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> batch;
  std::size_t dropped = 0;

  while (true) {
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait_for(
        lock, std::chrono::milliseconds(100),
        [this] { return stopping_ || !cloud_queue_.empty(); });
      if (stopping_ && cloud_queue_.empty()) {
        break;
      }
      batch.swap(cloud_queue_);
      dropped += dropped_clouds_;
      dropped_clouds_ = 0;
    }

    for (const auto & msg : batch) {
      if (map_frame_.empty() && !msg->header.frame_id.empty()) {
        map_frame_ = msg->header.frame_id;
        RCLCPP_INFO(get_logger(), "Accumulating in frame '%s'", map_frame_.c_str());
      }

      // An empty message carries no fields at all, and pcl::fromROSMsg would
      // warn "Failed to find match for field 'x'" for each one. FAST-LIVO2
      // publishes such a placeholder on every frame it handles in LIO-only
      // mode (publish_frame_world only fills the cloud on VIO frames), so
      // skip them before the conversion.
      if (msg->width == 0 || msg->height == 0 || msg->fields.empty()) {
        continue;
      }

      // Warm-up gate. On the real rig the first scans arrive while fast_limo is
      // still converging (and the operator is still settling), so they would be
      // folded into the map at poses that are not yet trustworthy -- and a voxel
      // grid never takes them back out. The window starts at the first scan, not
      // at node start, and it is measured on the scans' own stamps, so it means
      // "N seconds of scans" regardless of how long the driver took to come up.
      // A zero stamp (publisher that does not stamp) disables the gate instead of
      // freezing the map forever.
      if (warmup_seconds_ > 0.0 && (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)) {
        const rclcpp::Time stamp(msg->header.stamp);
        if (!warmup_started_) {
          warmup_start_ = stamp;
          warmup_started_ = true;
        }
        const double elapsed = (stamp - warmup_start_).seconds();
        if (elapsed < warmup_seconds_) {
          continue;
        }
        if (!warmup_done_) {
          warmup_done_ = true;
          RCLCPP_INFO(
            get_logger(), "Warm-up finished after %.1f s of scans, accumulating from now on",
            elapsed);
        }
      }

      PointCloudT cloud;
      try {
        pcl::fromROSMsg(*msg, cloud);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Could not convert incoming scan: %s", e.what());
        continue;
      }

      if (insert_cloud(cloud) > 0) {
        map_dirty_ = true;
        // The map is as current as the newest scan folded into it.
        last_scan_stamp_ = msg->header.stamp;
        has_scan_stamp_ = true;
      }
    }
    batch.clear();

    if (dropped > 0) {
      RCLCPP_WARN(
        get_logger(), "Accumulation is falling behind, dropped %zu scan(s)", dropped);
      dropped = 0;
    }

    if (map_dirty_ && Clock::now() - last_publish >= period) {
      publish_map();
      last_publish = Clock::now();
      map_dirty_ = false;
    }
  }
}

std::size_t MapAccumulator::insert_cloud(const PointCloudT & cloud)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  std::size_t inserted = 0;

  for (const auto & point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    if (max_points_ > 0 && voxels_.size() >= max_points_) {
      RCLCPP_WARN_ONCE(
        get_logger(), "Map reached max_points (%zu), dropping further points", max_points_);
      break;
    }

    const VoxelKey key{
      static_cast<std::int32_t>(std::floor(point.x / voxel_leaf_)),
      static_cast<std::int32_t>(std::floor(point.y / voxel_leaf_)),
      static_cast<std::int32_t>(std::floor(point.z / voxel_leaf_))};

    // try_emplace keeps the first point seen in each voxel, the same way the
    // octree keeps its existing leaf contents and drops saturated updates.
    if (voxels_.try_emplace(key, point).second) {
      ++inserted;
    }
  }

  return inserted;
}

void MapAccumulator::publish_map()
{
  if (!rclcpp::ok()) {
    return;
  }

  PointCloudT cloud;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (voxels_.empty()) {
      return;
    }
    cloud.reserve(voxels_.size());
    for (const auto & entry : voxels_) {
      cloud.push_back(entry.second);
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = map_frame_;
  // Stamp with the newest scan folded in rather than the wall/sim clock: the
  // map is a function of the scans, not of when it happened to be published,
  // and this avoids touching the clock from the worker thread at all.
  msg.header.stamp = has_scan_stamp_ ? last_scan_stamp_ : builtin_interfaces::msg::Time();

  map_pub_->publish(msg);

  // Roughly every five publishes, without a throttled logger: those go through
  // the node clock, which is not safe to read before /clock has arrived.
  if (++publish_count_ % 5 == 1) {
    RCLCPP_INFO(
      get_logger(), "Accumulated map: %zu points in frame '%s'",
      cloud.points.size(), map_frame_.c_str());
  }
}

}  // namespace vector_mapping


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vector_mapping::MapAccumulator>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
