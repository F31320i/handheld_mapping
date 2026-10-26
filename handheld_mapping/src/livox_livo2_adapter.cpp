// The one message conversion FAST-LIVO2 forces on us, done in C++.
//
// Why this exists
// ---------------
// config/livo2_mid360.yaml selects the MID360 preprocessor (lidar_type: 8),
// which subscribes sensor_msgs/PointCloud2 and reads each point's `timestamp`
// field as an *absolute* nanosecond epoch (preprocess.cpp:
// `curvature = pt.timestamp * 1e-9 - header_time`). livox_ros_driver2 publishes
// livox_ros_driver2/CustomMsg instead, with `offset_time` relative to
// `timebase`. Two endpoints with different types do not error out, they simply
// never bind: LIVO2 then receives no LiDAR at all, logs nothing, publishes no
// pose, and still consumes images. This node rebuilds the sweep in the shape
// the MID360 handler reads.
//
// The driver's own PointCloud2 (xfer_format: 0) is not a substitute: it stores
// `timestamp` as seconds since the sweep start (`(offset_time - base_time)/1e9`
// in lddc.cpp), which the handler would read as a point ~1.7e9 s in the past.
//
// Why C++ and not the Python node this replaces
// ---------------------------------------------
// A Python adapter deserialises 20k CustomMsg point objects per sweep, which
// costs ~0.7 of a core at 10 Hz -- just enough to sit *below* the 200 Hz IMU
// rate. With a RELIABLE, depth-200 subscription that deficit never recovers:
// measured, the IMU arriving at LIVO2 slid from +0.05 s to +4.8 s within 85 s,
// at which point `sync_packages` sees an image newer than the newest sweep,
// returns false forever, and the front end stops computing entirely (no LIO/VIO
// at all) -- and eventually died with SIGSEGV when the backlog was replayed.
// The conversion itself is a 32-byte memcpy per point; there is no reason for
// it to be the bottleneck.
//
// Time base
// ---------
// The driver stamps with the host clock unless the LiDAR has PTP/GPS
// (comm/pub_handler.cpp::GetEthPacketTimestamp). So normally the offset is zero
// and stamps pass through untouched. The failure in the other direction is
// silent -- a device clock would put every sweep ~1.7e9 s away from the camera
// and LIVO2 would wait forever -- so the offset is still measured: for the
// first `warmup_frames` sweeps nothing is published (the driver replays a
// backlog right after connecting), then over `estimate_frames` sweeps
// `median(now - sweep_end)` is taken; below `host_time_threshold` it means
// "already host time" and the stamps are left alone, above it the offset is
// added to the sweep header, to every point and to the IMU.
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

using livox_ros_driver2::msg::CustomMsg;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

// The layout pcl::fromROSMsg reads for MID360: it matches fields by name and
// uses the offsets we advertise, so they only have to be self-consistent.
// Sized and aligned like PCL's mid360_ros::Point (the double timestamp needs
// an 8-byte boundary), i.e. 32 bytes with 6 bytes of padding after `line`.
struct Mid360Point
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  std::uint8_t pad[6];
  double timestamp;
};
static_assert(sizeof(Mid360Point) == 32, "mid360 point must stay 32 bytes");
static_assert(offsetof(Mid360Point, timestamp) == 24, "timestamp offset");

std::int64_t to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

builtin_interfaces::msg::Time to_stamp(std::int64_t ns)
{
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<std::int32_t>(ns / 1000000000LL);
  out.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
  return out;
}

}  // namespace

class LivoxLivo2Adapter : public rclcpp::Node
{
public:
  LivoxLivo2Adapter()
  : rclcpp::Node("livox_livo2_adapter")
  {
    lidar_in_ = declare_parameter<std::string>("lidar_in", "/livox/lidar");
    lidar_out_ = declare_parameter<std::string>("lidar_out", "/livox/lidar_livo");
    lidar_out_limo_ = declare_parameter<std::string>("lidar_out_limo", "/livox/lidar_limo");
    imu_in_ = declare_parameter<std::string>("imu_in", "/livox/imu");
    imu_out_ = declare_parameter<std::string>("imu_out", "/livox/imu_sync");
    clock_offset_ = declare_parameter<double>("clock_offset", 0.0);
    estimate_frames_ = std::max(1, static_cast<int>(declare_parameter<int>("estimate_frames", 20)));
    warmup_frames_ = std::max(0, static_cast<int>(declare_parameter<int>("warmup_frames", 5)));
    host_time_threshold_ = declare_parameter<double>("host_time_threshold", 10.0);
    stats_period_ = declare_parameter<double>("stats_period", 10.0);
    // Blind zone around the sensor, metres. 0 = off. Applied to both output
    // streams, before either front end (or the map built from them) sees the
    // sweep -- see on_lidar().
    min_range_ = declare_parameter<double>("min_range", 0.0);
    if (min_range_ < 0.0) {
      RCLCPP_WARN(get_logger(), "min_range must not be negative, using 0");
      min_range_ = 0.0;
    }

    // 0 = measure it, anything else = use as-is.
    if (clock_offset_ != 0.0) {
      offset_ns_ = static_cast<std::int64_t>(clock_offset_ * 1e9);
      have_offset_ = true;
    } else {
      warmup_left_ = warmup_frames_;
    }
    samples_.reserve(static_cast<std::size_t>(estimate_frames_));
    fields_ = build_fields();

    // Reliable to match FAST-LIVO2's default subscription; the depths are the
    // backstop -- if this node ever fell behind, it must replay old sweeps, not
    // grow without bound the way the Python node did.
    pub_lidar_ = create_publisher<PointCloud2>(lidar_out_, rclcpp::QoS(20));
    // Same sweep, different time convention; see on_lidar(). Only fast_limo
    // reads this one, and it is cheap next to being wrong.
    pub_lidar_limo_ = create_publisher<PointCloud2>(lidar_out_limo_, rclcpp::QoS(20));
    if (min_range_ > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "min_range %.2f m: points closer than that are dropped from both %s and %s "
        "(the rig and the operator's arm)",
        min_range_, lidar_out_.c_str(), lidar_out_limo_.c_str());
    }
    pub_imu_ = create_publisher<Imu>(imu_out_, rclcpp::QoS(1000));
    sub_lidar_ = create_subscription<CustomMsg>(
      lidar_in_, rclcpp::QoS(20),
      [this](CustomMsg::ConstSharedPtr msg) {on_lidar(msg);});
    sub_imu_ = create_subscription<Imu>(
      imu_in_, rclcpp::QoS(1000),
      [this](Imu::ConstSharedPtr msg) {on_imu(msg);});

    RCLCPP_INFO(
      get_logger(), "%s (CustomMsg) -> %s + %s (PointCloud2), %s -> %s; %s",
      lidar_in_.c_str(), lidar_out_.c_str(), lidar_out_limo_.c_str(),
      imu_in_.c_str(), imu_out_.c_str(),
      have_offset_
      ? ("offset fixed at " + std::to_string(clock_offset_) + " s").c_str()
      : ("estimating the offset over " + std::to_string(estimate_frames_) +
      " sweeps after a " + std::to_string(warmup_frames_) + "-sweep warm-up").c_str());

    if (stats_period_ > 0.0) {
      stats_timer_ = create_wall_timer(
        std::chrono::duration<double>(stats_period_),
        [this]() {report_stats();});
    }
  }

private:
  static std::vector<PointField> build_fields()
  {
    std::vector<PointField> fields(7);
    auto set = [&fields](std::size_t i, const char * name, std::uint32_t offset,
        std::uint8_t type) {
        fields[i].name = name;
        fields[i].offset = offset;
        fields[i].datatype = type;
        fields[i].count = 1;
      };
    set(0, "x", offsetof(Mid360Point, x), PointField::FLOAT32);
    set(1, "y", offsetof(Mid360Point, y), PointField::FLOAT32);
    set(2, "z", offsetof(Mid360Point, z), PointField::FLOAT32);
    set(3, "intensity", offsetof(Mid360Point, intensity), PointField::FLOAT32);
    set(4, "tag", offsetof(Mid360Point, tag), PointField::UINT8);
    set(5, "line", offsetof(Mid360Point, line), PointField::UINT8);
    set(6, "timestamp", offsetof(Mid360Point, timestamp), PointField::FLOAT64);
    return fields;
  }

  void on_lidar(const CustomMsg::ConstSharedPtr & msg)
  {
    if (!have_offset_) {
      if (warmup_left_ > 0) {
        if (--warmup_left_ == 0) {
          RCLCPP_INFO(
            get_logger(), "warm-up done, measuring over the next %d sweeps", estimate_frames_);
        }
        return;
      }
      const std::int64_t sweep_end_ns =
        static_cast<std::int64_t>(msg->timebase) + last_offset_time(msg);
      add_sample(now().nanoseconds() - sweep_end_ns);
      if (!have_offset_) {
        return;
      }
    }

    const std::int64_t base_ns = static_cast<std::int64_t>(msg->timebase) + offset_ns_;
    const std::size_t n = msg->points.size();

    auto cloud = std::make_unique<PointCloud2>();
    cloud->header.stamp = to_stamp(base_ns);
    cloud->header.frame_id = msg->header.frame_id;
    cloud->height = 1;
    cloud->width = static_cast<std::uint32_t>(n);
    cloud->fields = fields_;
    cloud->is_bigendian = false;
    cloud->point_step = sizeof(Mid360Point);
    cloud->row_step = cloud->point_step * cloud->width;
    cloud->is_dense = true;
    cloud->data.resize(n * sizeof(Mid360Point));

    // Absolute nanoseconds at offset 24, which is what mid360_handler subtracts
    // the header from to get the within-sweep offset.
    //
    // min_range: drop the rig itself. Carrying the handle means the stand and
    // the operator's arm sit a few tens of centimetres from the sensor, and they
    // are not noise a front end can average away -- they move exactly with the
    // sensor, so to a scan matcher they look like static structure, and the map
    // ends up with a ghost of the rig everywhere it has been. A sphere around
    // the sensor is the cheapest thing that removes them without removing the
    // room.
    //
    // Filtered HERE, once, before either stream exists: that is what keeps the
    // near field out of everything downstream -- LIO, the photometric VIO, the
    // per-frame cloud and the pose-graph map alike. (The alternative, LIVO2's
    // own preprocess.blind, would work for LIVO2 but not for fast_limo, and
    // would still pay for deserialising the near points.)
    //
    // The boundary is inclusive: a point at exactly min_range is kept.
    const double min_r2 = min_range_ * min_range_;
    auto * out = reinterpret_cast<Mid360Point *>(cloud->data.data());
    std::size_t kept = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const auto & p = msg->points[i];
      if (min_r2 > 0.0 && (p.x * p.x + p.y * p.y + p.z * p.z) < min_r2) {
        continue;
      }
      out[kept].x = p.x;
      out[kept].y = p.y;
      out[kept].z = p.z;
      out[kept].intensity = p.reflectivity;
      out[kept].tag = p.tag;
      out[kept].line = p.line;
      std::memset(out[kept].pad, 0, sizeof(out[kept].pad));
      out[kept].timestamp =
        static_cast<double>(base_ns + static_cast<std::int64_t>(p.offset_time));
      ++kept;
    }
    cloud->data.resize(kept * sizeof(Mid360Point));
    cloud->width = static_cast<std::uint32_t>(kept);
    cloud->row_step = cloud->point_step * cloud->width;

    last_lag_ms_ = (now().nanoseconds() - (base_ns + last_offset_time(msg))) / 1e6;
    sweeps_++;
    last_points_ = n;
    last_kept_points_ = kept;

    // fast_limo's LIVOX branch reads the same `timestamp` field as SECONDS
    // RELATIVE TO THE SWEEP START and adds it to the cloud stamp
    // (Localizer.cpp deskewPointCloud: `sweep_ref_time + pt.timestamp`), then
    // uses the result to look up propagated IMU states. Feeding it the absolute
    // nanoseconds above gives per-point times ~1.79e18 s away, every point gets
    // deskewed with a nonsense pose and the filter diverges within a second --
    // no error, just a map that flies off. Hence a second topic with the
    // unit/epoch the other consumer actually wants.
    //
    // Published unconditionally rather than behind a subscription count: Zenoh
    // reporting zero here while fast_limo listens would be a silent failure,
    // and one extra memcpy per sweep at 10 Hz is cheaper than that.
    {
      // Same sweep, same min_range cut (already applied above), only the
      // per-point timestamp convention differs.
      auto limo = std::make_unique<PointCloud2>(*cloud);
      auto * limo_pts = reinterpret_cast<Mid360Point *>(limo->data.data());
      for (std::size_t i = 0; i < kept; ++i) {
        limo_pts[i].timestamp =
          (limo_pts[i].timestamp - static_cast<double>(base_ns)) * 1e-9;
      }
      limo->data.resize(kept * sizeof(Mid360Point));
      limo->width = static_cast<std::uint32_t>(kept);
      limo->row_step = limo->point_step * limo->width;
      limo->is_dense = true;
      pub_lidar_limo_->publish(std::move(limo));
    }

    pub_lidar_->publish(std::move(cloud));
  }

  void on_imu(const Imu::ConstSharedPtr & msg)
  {
    if (!have_offset_) {
      return;
    }
    if (offset_ns_ == 0) {
      pub_imu_->publish(*msg);
      return;
    }
    auto out = std::make_unique<Imu>(*msg);
    out->header.stamp = to_stamp(to_ns(msg->header.stamp) + offset_ns_);
    pub_imu_->publish(std::move(out));
  }

  static std::int64_t last_offset_time(const CustomMsg::ConstSharedPtr & msg)
  {
    return msg->points.empty() ? 0 : static_cast<std::int64_t>(msg->points.back().offset_time);
  }

  void add_sample(std::int64_t offset_ns)
  {
    samples_.push_back(offset_ns);
    if (static_cast<int>(samples_.size()) < estimate_frames_) {
      return;
    }
    std::vector<std::int64_t> sorted = samples_;
    std::sort(sorted.begin(), sorted.end());
    const std::int64_t median_ns = sorted[sorted.size() / 2];
    const double spread_ms = (sorted.back() - sorted.front()) / 1e6;

    if (std::llabs(median_ns) < static_cast<std::int64_t>(host_time_threshold_ * 1e9)) {
      // The driver already stamped in host time; adding anything would push the
      // sweeps into the future and break LIVO2's association.
      offset_ns_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "input is already in host time (median offset %+.6f s, spread %.1f ms over %zu "
        "sweeps): passing the stamps through unchanged",
        median_ns / 1e9, spread_ms, samples_.size());
    } else {
      offset_ns_ = median_ns;
      RCLCPP_INFO(
        get_logger(), "clock offset locked: %+.6f s (spread %.1f ms over %zu sweeps)",
        offset_ns_ / 1e9, spread_ms, samples_.size());
    }
    have_offset_ = true;
    samples_.clear();
  }

  void report_stats()
  {
    if (sweeps_ == 0) {
      return;
    }
    if (min_range_ > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "%.1f sweeps/s, %zu points, %zu kept after min_range %.2f m (%zu dropped), "
        "%.0f ms behind the host clock",
        sweeps_ / stats_period_, last_points_, last_kept_points_, min_range_,
        last_points_ - last_kept_points_, last_lag_ms_);
    } else {
      RCLCPP_INFO(
        get_logger(), "%.1f sweeps/s, %zu points, %.0f ms behind the host clock",
        sweeps_ / stats_period_, last_points_, last_lag_ms_);
    }
    sweeps_ = 0;
  }

  std::string lidar_in_, lidar_out_, lidar_out_limo_, imu_in_, imu_out_;
  double clock_offset_ = 0.0;
  int estimate_frames_ = 20;
  int warmup_frames_ = 5;
  double host_time_threshold_ = 10.0;
  double stats_period_ = 10.0;
  double min_range_ = 0.0;

  bool have_offset_ = false;
  std::int64_t offset_ns_ = 0;
  int warmup_left_ = 0;
  std::vector<std::int64_t> samples_;
  std::vector<PointField> fields_;

  std::size_t sweeps_ = 0;
  std::size_t last_points_ = 0;
  std::size_t last_kept_points_ = 0;
  double last_lag_ms_ = 0.0;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_lidar_limo_;
  rclcpp::Publisher<Imu>::SharedPtr pub_imu_;
  rclcpp::Subscription<CustomMsg>::SharedPtr sub_lidar_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxLivo2Adapter>());
  rclcpp::shutdown();
  return 0;
}
