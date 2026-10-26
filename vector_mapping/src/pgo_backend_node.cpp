// Pose-graph back end for the LIVO2 front end.
//
// Subscribes to
//   * a raw Livox scan with per-point timestamps (pcd_timestamp_injector,
//     abs_ns:=false -> seconds since the start of the sweep), and
//   * the front-end odometry (/odometry/lio, base_link in "odom"),
//
// deskews the scan with the odometry, picks keyframes, verifies loop closures
// with GICP, optimises a GTSAM pose graph and publishes the resulting map /
// trajectory / correction transform.
//
// Nothing in FAST-LIVO2 is touched: the front end keeps publishing exactly what
// it published before. The back end only *observes* it.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "vector_mapping/pgo/deskewer.hpp"
#include "vector_mapping/pgo/keyframe_manager.hpp"
#include "vector_mapping/pgo/loop_detector.hpp"
#include "vector_mapping/pgo/map_assembler.hpp"
#include "vector_mapping/pgo/pose_buffer.hpp"
#include "vector_mapping/pgo/pose_graph.hpp"
#include "vector_mapping/pgo/types.hpp"

namespace vector_mapping
{
namespace pgo
{

namespace
{

double toSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

geometry_msgs::msg::Pose toRosPose(const Pose & p)
{
  geometry_msgs::msg::Pose out;
  out.position.x = p.t.x();
  out.position.y = p.t.y();
  out.position.z = p.t.z();
  out.orientation.x = p.q.x();
  out.orientation.y = p.q.y();
  out.orientation.z = p.q.z();
  out.orientation.w = p.q.w();
  return out;
}

Pose fromRosPose(const geometry_msgs::msg::Pose & p)
{
  Pose out;
  out.t = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  out.q = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  out.q.normalize();
  return out;
}

bool hasField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & f : msg.fields) {
    if (f.name == name) {
      return true;
    }
  }
  return false;
}

// Converts a PointCloud2 by field name, filling only what the message actually
// carries.
//
// pcl::fromROSMsg is not used here because it warns once per message for every
// field of the target struct the cloud lacks, and both inputs lack something:
// the Livox sweep has no colour, the front-end cloud has no per-point
// timestamp. Missing fields keep their defaults (zero intensity, zero
// timestamp, opaque white).
CloudT::Ptr cloudFromRos(const sensor_msgs::msg::PointCloud2 & msg, std::size_t * invalid)
{
  if (invalid) {
    *invalid = 0;
  }
  if (msg.width == 0 || msg.height == 0 || msg.fields.empty() || msg.data.empty()) {
    return nullptr;
  }
  if (!hasField(msg, "x") || !hasField(msg, "y") || !hasField(msg, "z")) {
    return nullptr;
  }

  using F32 = sensor_msgs::PointCloud2ConstIterator<float>;
  using F64 = sensor_msgs::PointCloud2ConstIterator<double>;
  using U32 = sensor_msgs::PointCloud2ConstIterator<std::uint32_t>;

  const std::size_t count = static_cast<std::size_t>(msg.width) * msg.height;
  if (msg.data.size() < count * msg.point_step) {
    return nullptr;
  }

  F32 ix(msg, "x");
  F32 iy(msg, "y");
  F32 iz(msg, "z");
  std::unique_ptr<F32> ii;
  std::unique_ptr<F64> it;
  std::unique_ptr<F32> irgb;
  std::unique_ptr<U32> irgba;
  if (hasField(msg, "intensity")) {
    ii = std::make_unique<F32>(msg, "intensity");
  }
  if (hasField(msg, "timestamp")) {
    it = std::make_unique<F64>(msg, "timestamp");
  }
  if (hasField(msg, "rgb")) {
    irgb = std::make_unique<F32>(msg, "rgb");
  } else if (hasField(msg, "rgba")) {
    irgba = std::make_unique<U32>(msg, "rgba");
  }

  auto out = std::make_shared<CloudT>();
  out->reserve(count);
  std::size_t bad = 0;
  for (std::size_t i = 0; i < count; ++i, ++ix, ++iy, ++iz) {
    PointT p;
    p.x = *ix;
    p.y = *iy;
    p.z = *iz;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      ++bad;
    } else {
      if (ii) {
        p.intensity = **ii;
      }
      if (it) {
        p.timestamp = **it;
      }
      if (irgb) {
        // "rgb" is a FLOAT32 whose bits are the packed 0xAARRGGBB value.
        const float packed = **irgb;
        std::memcpy(&p.rgba, &packed, sizeof(p.rgba));
      } else if (irgba) {
        p.rgba = **irgba;
      }
      p.a = 255;
      out->push_back(p);
    }
    if (ii) {
      ++(*ii);
    }
    if (it) {
      ++(*it);
    }
    if (irgb) {
      ++(*irgb);
    }
    if (irgba) {
      ++(*irgba);
    }
  }
  out->is_dense = true;
  if (invalid) {
    *invalid = bad;
  }
  return out;
}

}  // namespace

class PgoBackendNode : public rclcpp::Node
{
public:
  PgoBackendNode() : Node("pgo_backend")
  {
    declareParameters();
    readParameters();
    setupPublishers();
    setupSubscriptions();
    setupServices();

    // verbose is the one parameter worth changing mid-survey: silent for the
    // walk, detail when something looks wrong. Every other parameter is read
    // once at startup and reported "successful" here so that setting it is not
    // an error -- it simply does nothing until the next launch.
    verbose_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "verbose") {
            verbose_ = p.as_bool();
          }
        }
        return result;
      });

    running_ = true;
    worker_ = std::thread(&PgoBackendNode::workerLoop, this);

    if (verbose_) {
      RCLCPP_INFO(
        get_logger(),
        "PGO back end: cloud='%s' (%s, deskew=%s) odom='%s' map_frame='%s' loops=%s",
        (frontend_source_ ? frontend_cloud_topic_ : scan_topic_).c_str(),
        frontend_source_ ? "front end, already undistorted" : "raw lidar",
        (!frontend_source_ && deskew_enabled_) ? "on" : "off",
        odom_topic_.c_str(), map_frame_.c_str(), loop_enabled_ ? "on" : "off");
    }
    if (gnss_enabled_ && verbose_) {
      RCLCPP_INFO(
        get_logger(), "Absolute position factor: '%s', sigmas [%.3f %.3f %.3f] m",
        gnss_topic_.c_str(), gnss_sigmas_[0], gnss_sigmas_[1], gnss_sigmas_[2]);
    }
  }

  ~PgoBackendNode() override
  {
    stop();
    // A survey that ends with Ctrl-C instead of the service call must not
    // silently lose the map. Only when asked, because the simulation exits
    // this way too and there the map is throwaway.
    if (save_on_shutdown_) {
      saveMap(pcd_path_.empty() ? defaultPcdPath() : pcd_path_);
    }
  }

  void stop()
  {
    if (!running_.exchange(false)) {
      return;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  // ------------------------------------------------------- absolute position
  // Declared up here so that member function *signatures* can name them; the
  // class body is otherwise one long run of inline definitions.
  struct GnssFix
  {
    double stamp = 0.0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
  };

  // One accepted fix, stored in the frames it arrived in. The graph-frame
  // position is recomputed from these every time the frame alignment is
  // refined, which is why the raw values are kept instead of the factor.
  //   lever = the keyframe's odom position minus the odom position at the
  //   fix's own timestamp, so the factor lands on the keyframe and not on
  //   where the fix happened to be taken.
  struct GnssAnchorRecord
  {
    int kf = -1;
    Eigen::Vector3d fix = Eigen::Vector3d::Zero();
    Eigen::Vector3d lever = Eigen::Vector3d::Zero();
  };

  static constexpr std::size_t kMaxGnssSamples = 2000;  // 33 min at 1 Hz
  static constexpr std::size_t kMaxGnssAnchors = 300;

  // ---------------------------------------------------------------- config
  void declareParameters()
  {
    declare_parameter<std::string>("cloud_source", "frontend");
    declare_parameter<std::string>("scan_topic", "/livox/lidar_be");
    declare_parameter<std::string>("frontend_cloud_topic", "/cloud_registered");
    declare_parameter<std::string>("odom_topic", "/odometry/lio");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    // Added to every published cloud / pose. See readParameters().
    declare_parameter<std::vector<double>>(
      "output_translation", {0.0098146, 0.02329, 0.5002583});
    // base_link -> IMU, applied to the trajectory and the loop markers only,
    // so they are drawn where /path and /ground_truth/path are. See
    // displayPosition().
    declare_parameter<std::vector<double>>(
      "trajectory_imu_offset", {0.0098146, 0.02329, 0.5002583});

    declare_parameter<std::vector<double>>(
      "lidar_translation", {0.033234, 0.0, 0.539234});
    declare_parameter<std::vector<double>>("lidar_rotation_rpy", {0.0, M_PI / 4.0, 0.0});
    declare_parameter<bool>("deskew", true);
    declare_parameter<double>("odom_wait_timeout", 0.5);

    declare_parameter<double>("keyframe_distance", 0.5);
    declare_parameter<double>("keyframe_angle_deg", 10.0);
    declare_parameter<double>("keyframe_time", 0.5);
    declare_parameter<double>("keyframe_max_time", 5.0);
    declare_parameter<double>("keyframe_voxel", 0.05);
    declare_parameter<int>("keyframe_min_points", 200);

    declare_parameter<bool>("enable_loop_closure", true);
    declare_parameter<double>("loop_search_radius", 3.0);
    declare_parameter<double>("loop_max_yaw_deg", 45.0);
    declare_parameter<double>("loop_min_time_gap", 20.0);
    declare_parameter<int>("loop_min_index_gap", 15);
    declare_parameter<double>("loop_min_path_gap", 3.0);
    declare_parameter<double>("loop_min_path_ratio", 1.5);
    declare_parameter<int>("loop_max_candidates", 3);
    declare_parameter<int>("loop_local_map_neighbors", 5);
    declare_parameter<double>("loop_icp_source_voxel", 0.10);
    declare_parameter<double>("loop_icp_target_voxel", 0.10);
    declare_parameter<double>("loop_icp_max_corr", 1.0);
    declare_parameter<int>("loop_icp_max_iter", 30);
    declare_parameter<double>("loop_inlier_dist", 0.20);
    declare_parameter<double>("loop_max_fitness", 0.10);
    declare_parameter<double>("loop_min_inlier_ratio", 0.60);

    declare_parameter<std::vector<double>>(
      "graph_odom_sigmas", {0.02, 0.02, 0.02, 0.01, 0.01, 0.02});
    declare_parameter<std::vector<double>>(
      "graph_loop_sigmas", {0.01, 0.01, 0.01, 0.005, 0.005, 0.01});
    declare_parameter<std::vector<double>>(
      "graph_prior_sigmas", {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3});
    declare_parameter<double>("graph_loop_huber", 1.0);
    declare_parameter<int>("graph_max_iterations", 50);

    // Absolute position factor (GNSS-like). See the block comment in
    // pgo_backend.yaml for what these mean and why they are set this way.
    declare_parameter<bool>("enable_gnss_factor", false);
    declare_parameter<std::string>("gnss_topic", "/odometry/gps");
    declare_parameter<std::vector<double>>("graph_gnss_sigmas", {0.02, 0.02, 0.05});
    declare_parameter<double>("graph_gnss_robust", 0.0);
    declare_parameter<double>("gnss_match_window", 0.5);
    declare_parameter<double>("gnss_align_min_distance", 1.0);

    declare_parameter<double>("map_voxel", 0.05);
    declare_parameter<double>("map_publish_period", 2.0);
    declare_parameter<bool>("publish_correction_tf", true);
    declare_parameter<bool>("publish_current_scan", false);

    declare_parameter<std::string>("pcd_path", "");
    declare_parameter<bool>("save_on_shutdown", false);
    declare_parameter<bool>("save_extra_outputs", false);
    declare_parameter<int>("scan_queue_size", 2);

    // Console volume. False (the default) keeps the terminal silent during a
    // survey: no keyframe lines, no map/loop/GNSS chatter, no startup banner.
    // What still prints is what needs a human: warnings, errors, the pose-jump
    // report below, and the save confirmations.
    declare_parameter<bool>("verbose", false);

    // Pose-jump watch. By default it only *reports*: a pose that jumps more
    // than sanity_max_jump (or implies more than sanity_max_speed) since the
    // last accepted pose prints a WARN, and the scan is still built into the
    // map. sanity_refuse_scans=true brings back the old behaviour of dropping
    // such scans instead.
    declare_parameter<bool>("sanity_enabled", true);
    declare_parameter<bool>("sanity_refuse_scans", false);
    declare_parameter<double>("sanity_max_speed", 3.0);
    declare_parameter<double>("sanity_max_jump", 0.5);
    declare_parameter<int>("sanity_reacquire_scans", 20);
    declare_parameter<double>("sanity_reacquire_jump", 2.0);
  }

  void readParameters()
  {
    cloud_source_ = get_parameter("cloud_source").as_string();
    if (cloud_source_ != "frontend" && cloud_source_ != "lidar") {
      throw std::runtime_error("cloud_source must be 'frontend' or 'lidar'");
    }
    frontend_source_ = cloud_source_ == "frontend";
    scan_topic_ = get_parameter("scan_topic").as_string();
    frontend_cloud_topic_ = get_parameter("frontend_cloud_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    deskew_enabled_ = get_parameter("deskew").as_bool();
    odom_wait_timeout_ = get_parameter("odom_wait_timeout").as_double();
    loop_enabled_ = get_parameter("enable_loop_closure").as_bool();
    publish_correction_tf_ = get_parameter("publish_correction_tf").as_bool();
    publish_current_scan_ = get_parameter("publish_current_scan").as_bool();
    map_publish_period_ = get_parameter("map_publish_period").as_double();
    pcd_path_ = get_parameter("pcd_path").as_string();
    save_on_shutdown_ = get_parameter("save_on_shutdown").as_bool();
    save_extra_outputs_ = get_parameter("save_extra_outputs").as_bool();
    scan_queue_size_ = std::max(1, static_cast<int>(get_parameter("scan_queue_size").as_int()));

    verbose_ = get_parameter("verbose").as_bool();

    sanity_enabled_ = get_parameter("sanity_enabled").as_bool();
    sanity_refuse_scans_ = get_parameter("sanity_refuse_scans").as_bool();
    sanity_max_speed_ = std::max(0.1, get_parameter("sanity_max_speed").as_double());
    sanity_max_jump_ = std::max(0.01, get_parameter("sanity_max_jump").as_double());
    sanity_reacquire_scans_ =
      std::max(1, static_cast<int>(get_parameter("sanity_reacquire_scans").as_int()));
    sanity_reacquire_jump_ =
      std::max(sanity_max_jump_, get_parameter("sanity_reacquire_jump").as_double());

    // Everything this node publishes has to be lifted from LIVO2's coordinate
    // convention into the map frame, by exactly the static odom -> camera_init
    // translation that sim_min_livo2.launch.xml publishes.
    //
    // Why there is a mismatch at all: /odometry/lio carries base_link poses
    // whose *numbers* are in camera_init (the bridge relabels the frame, it
    // does not subtract the IMU mount offset), and a front-end cloud converted
    // to the body frame and back reproduces camera_init numbers exactly. So the
    // map this node builds is in camera_init coordinates, but publishing it as
    // "map" skips the +0.5003 m that TF adds for everything LIVO2 stamps
    // camera_init -- the map and the path then sit 0.5 m below the front end's
    // own cloud and trajectory, while agreeing in x/y.
    const auto out_t = get_parameter("output_translation").as_double_array();
    if (out_t.size() != 3) {
      throw std::runtime_error("output_translation needs 3 elements");
    }
    output_translation_ = Eigen::Vector3f(
      static_cast<float>(out_t[0]), static_cast<float>(out_t[1]), static_cast<float>(out_t[2]));

    // base_link -> IMU, so the trajectory can be drawn at the same point on the
    // robot as /path and /ground_truth/path.
    const auto imu_t = get_parameter("trajectory_imu_offset").as_double_array();
    if (imu_t.size() != 3) {
      throw std::runtime_error("trajectory_imu_offset needs 3 elements");
    }
    trajectory_imu_offset_ = Eigen::Vector3f(
      static_cast<float>(imu_t[0]), static_cast<float>(imu_t[1]), static_cast<float>(imu_t[2]));

    // Extrinsics: base_link -> livox_lidar.
    const auto t = get_parameter("lidar_translation").as_double_array();
    const auto rpy = get_parameter("lidar_rotation_rpy").as_double_array();
    if (t.size() != 3 || rpy.size() != 3) {
      throw std::runtime_error("lidar_translation / lidar_rotation_rpy need 3 elements");
    }
    const Eigen::Matrix3d R =
      (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

    DeskewOptions dopt;
    dopt.enabled = deskew_enabled_;
    dopt.T_base_lidar = Eigen::Matrix4d::Identity();
    dopt.T_base_lidar.block<3, 3>(0, 0) = R;
    dopt.T_base_lidar.block<3, 1>(0, 3) = Eigen::Vector3d(t[0], t[1], t[2]);
    deskewer_ = std::make_unique<Deskewer>(dopt);

    KeyframeOptions kopt;
    kopt.min_distance = get_parameter("keyframe_distance").as_double();
    kopt.min_angle_deg = get_parameter("keyframe_angle_deg").as_double();
    kopt.min_time = get_parameter("keyframe_time").as_double();
    kopt.max_time = get_parameter("keyframe_max_time").as_double();
    kopt.voxel_leaf = get_parameter("keyframe_voxel").as_double();
    kopt.min_points = static_cast<int>(get_parameter("keyframe_min_points").as_int());
    keyframes_.configure(kopt);

    LoopOptions lopt;
    lopt.enabled = loop_enabled_;
    lopt.search_radius = get_parameter("loop_search_radius").as_double();
    lopt.max_yaw_deg = get_parameter("loop_max_yaw_deg").as_double();
    lopt.min_time_gap = get_parameter("loop_min_time_gap").as_double();
    lopt.min_index_gap = static_cast<int>(get_parameter("loop_min_index_gap").as_int());
    lopt.min_path_gap = get_parameter("loop_min_path_gap").as_double();
    lopt.min_path_ratio = get_parameter("loop_min_path_ratio").as_double();
    lopt.max_candidates = static_cast<int>(get_parameter("loop_max_candidates").as_int());
    lopt.local_map_neighbors = static_cast<int>(
      get_parameter("loop_local_map_neighbors").as_int());
    lopt.icp_source_voxel = get_parameter("loop_icp_source_voxel").as_double();
    lopt.icp_target_voxel = get_parameter("loop_icp_target_voxel").as_double();
    lopt.icp_max_corr = get_parameter("loop_icp_max_corr").as_double();
    lopt.icp_max_iter = static_cast<int>(get_parameter("loop_icp_max_iter").as_int());
    lopt.inlier_dist = get_parameter("loop_inlier_dist").as_double();
    lopt.max_fitness = get_parameter("loop_max_fitness").as_double();
    lopt.min_inlier_ratio = get_parameter("loop_min_inlier_ratio").as_double();
    loops_.configure(lopt);

    GraphOptions gopt;
    gopt.odom_sigmas = toArray(get_parameter("graph_odom_sigmas").as_double_array());
    gopt.loop_sigmas = toArray(get_parameter("graph_loop_sigmas").as_double_array());
    gopt.prior_sigmas = toArray(get_parameter("graph_prior_sigmas").as_double_array());
    gopt.loop_huber = get_parameter("graph_loop_huber").as_double();
    gopt.max_iterations = static_cast<int>(get_parameter("graph_max_iterations").as_int());
    graph_.configure(gopt);
    // Reference for widening an odom edge across a front-end re-acquisition.
    graph_odom_sigma_t_ = gopt.odom_sigmas[0];

    gnss_enabled_ = get_parameter("enable_gnss_factor").as_bool();
    gnss_topic_ = get_parameter("gnss_topic").as_string();
    const auto gs = get_parameter("graph_gnss_sigmas").as_double_array();
    if (gs.size() != 3) {
      throw std::runtime_error("graph_gnss_sigmas needs 3 elements [x y z]");
    }
    gnss_sigmas_ = {{gs[0], gs[1], gs[2]}};
    gnss_robust_ = get_parameter("graph_gnss_robust").as_double();
    gnss_match_window_ = get_parameter("gnss_match_window").as_double();
    gnss_align_min_distance_ = get_parameter("gnss_align_min_distance").as_double();

    assembler_.setVoxel(get_parameter("map_voxel").as_double());
  }

  static std::array<double, 6> toArray(const std::vector<double> & v)
  {
    if (v.size() != 6) {
      throw std::runtime_error("sigma arrays need 6 elements [x y z roll pitch yaw]");
    }
    return {{v[0], v[1], v[2], v[3], v[4], v[5]}};
  }

  // ------------------------------------------------------------ ROS plumbing
  void setupPublishers()
  {
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pgo/map", rclcpp::QoS(1).transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/pgo/trajectory", 1);
    loop_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/pgo/loops", 1);
    if (publish_current_scan_) {
      scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/pgo/current_scan", 1);
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    if (publish_correction_tf_) {
      correction_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {broadcastCorrection();});
    }
  }

  void setupSubscriptions()
  {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        const Pose p = fromRosPose(msg->pose.pose);
        poses_.add(toSec(msg->header.stamp), p);
        // Publish an identity map -> odom until the first keyframe exists, so
        // consumers of the frame tree are not left with a dangling chain.
        if (publish_correction_tf_) {
          std::lock_guard<std::mutex> lock(correction_mtx_);
          if (!have_correction_) {
            correction_ = Pose{};
            have_correction_ = true;
          }
        }
      });

    if (gnss_enabled_) {
      // The callback only parks the fix. Every mutation of keyframes_ and
      // graph_ happens on the worker thread, so doing the graph work here
      // would race with it.
      gnss_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        gnss_topic_, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          GnssFix fix;
          fix.stamp = toSec(msg->header.stamp);
          fix.p = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
          if (!fix.p.allFinite()) {
            return;
          }
          std::lock_guard<std::mutex> lock(gnss_mtx_);
          pending_gnss_ = fix;
        });
    }

    const std::string cloud_topic = frontend_source_ ? frontend_cloud_topic_ : scan_topic_;
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mtx_);
        if (static_cast<int>(scan_queue_.size()) >= scan_queue_size_) {
          scan_queue_.pop_front();
          ++dropped_scans_;
        }
        scan_queue_.push_back(msg);
        queue_cv_.notify_one();
      });
  }

  void setupServices()
  {
    save_srv_ = create_service<std_srvs::srv::Trigger>(
      "/pgo/save_map",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lock(queue_mtx_);
        save_requested_ = true;
        res->success = true;
        res->message = "save scheduled";
      });
  }

  // ------------------------------------------------------------- worker loop
  void workerLoop()
  {
    while (running_) {
      sensor_msgs::msg::PointCloud2::SharedPtr msg;
      bool do_save = false;
      {
        std::unique_lock<std::mutex> lock(queue_mtx_);
        queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
          return !running_ || !scan_queue_.empty() || save_requested_;
        });
        if (!running_) {
          break;
        }
        if (!scan_queue_.empty()) {
          msg = scan_queue_.front();
          scan_queue_.pop_front();
        }
        if (save_requested_) {
          save_requested_ = false;
          do_save = true;
        }
      }

      if (do_save) {
        const std::string path = pcd_path_.empty() ? defaultPcdPath() : pcd_path_;
        saveMap(path);
      }
      if (msg) {
        try {
          processScan(*msg);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "scan processing failed: %s", e.what());
        }
      }
      try {
        processPendingGnss();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "gnss anchor failed: %s", e.what());
      }
      maybePublishMap();
    }
  }

  // A scan that has been turned into a body-frame cloud and paired with the
  // body pose it was captured at; this is everything the keyframe tail needs.
  struct PreparedScan
  {
    CloudT::Ptr cloud;
    Pose pose_end;
    double stamp = 0.0;
    std::size_t raw_points = 0;
    std::size_t invalid = 0;
    std::string detail;  // appended to the keyframe log line
    bool odom_extrapolated = false;
  };

  void processScan(const sensor_msgs::msg::PointCloud2 & msg)
  {
    PreparedScan scan;
    const bool ok = frontend_source_ ? prepareFrontendScan(msg, &scan) : prepareLidarScan(msg, &scan);
    if (!ok) {
      return;
    }

    // Read the pose-jump watch before anything is derived from this pose. In
    // the default report-only mode it warns and returns true; see
    // poseIsSane().
    if (sanity_enabled_ && !poseIsSane(scan.pose_end, scan.stamp)) {
      return;
    }

    // Where this keyframe goes in the map frame. The front end gives the pose
    // in odom; the current correction carries whatever the pose graph has
    // already decided. Inserting at the raw odom pose instead would make every
    // keyframe that closes no loop look like "no correction", and since
    // map -> odom is derived from the newest keyframe, the TF would snap back
    // to identity on it and jump back to the last loop's shift on the next one
    // that does close a loop -- a few centimetres, back and forth, at keyframe
    // rate.
    const Pose map_pose = currentCorrection() * scan.pose_end;

    if (publish_current_scan_ && scan_pub_) {
      publishCloud(*scan.cloud, map_pose);
    }

    if (!keyframes_.accepts(scan.pose_end, scan.stamp)) {
      return;
    }

    const int id = keyframes_.add(map_pose, scan.pose_end, scan.stamp, *scan.cloud);
    if (id < 0) {
      return;
    }

    graph_.addNode(id, map_pose);
    if (id > 0) {
      const Pose rel = keyframes_.frames()[id - 1].odom_pose.inverse() * scan.pose_end;
      // First keyframe after a re-acquisition: the front end teleported, so
      // `rel` is the size of the jump, not a distance walked. As a normal odom
      // edge (8 mm) it would be the stiffest constraint in the graph and would
      // pull every later loop closure towards a lie. It cannot simply be
      // skipped either -- an unconnected node makes the next optimisation fail
      // on a singular system. So it is kept and widened to "as uncertain as the
      // jump is large", which lets a later loop closure decide what to do.
      const double scale = pending_reacquire_ ?
        std::max(1.0, rel.t.norm() / std::max(graph_odom_sigma_t_, 1e-6)) : 1.0;
      graph_.addOdomEdge(id - 1, id, rel, scale);
      if (pending_reacquire_) {
        RCLCPP_WARN(
          get_logger(), "kf %d: odom edge to kf %d widened x%.0f (re-acquired after a pose jump)",
          id, id - 1, scale);
      }
    }
    pending_reacquire_ = false;

    // Fuse the keyframe the back end stored (not the finer front-end scan) so
    // an incremental add and a later full rebuild see exactly the same points.
    assembler_.add(map_pose, *keyframes_.frames()[id].cloud);

    if (verbose_) {
      RCLCPP_INFO(
        get_logger(),
        "kf %d: t=%.2f pts=%zu (raw %zu, %zu invalid)%s%s",
        id, scan.stamp, scan.cloud->size(), scan.raw_points, scan.invalid, scan.detail.c_str(),
        scan.odom_extrapolated ? " [odom extrapolated]" : "");
    }

    if (loop_enabled_) {
      tryLoopClosure(id);
    }

    publishPath();
    updateCorrection();
    map_dirty_ = true;
  }

  // Raw Livox sweep: deskew it with the front-end odometry, which is also the
  // pose the keyframe is stored at.
  bool prepareLidarScan(const sensor_msgs::msg::PointCloud2 & msg, PreparedScan * out)
  {
    std::size_t invalid = 0;
    CloudT::Ptr raw = cloudFromRos(msg, &invalid);
    if (!raw || raw->size() < 100) {
      return false;
    }

    const double scan_stamp = toSec(msg.header.stamp);

    // Sweep end: the body pose a keyframe is stored in. The injector writes
    // per-point offsets in [0, scan_period] relative to the header stamp.
    double max_tau = 0.0;
    for (const auto & pt : raw->points) {
      if (pt.timestamp > max_tau) {
        max_tau = pt.timestamp;
      }
    }
    const double t_end = scan_stamp + max_tau;

    if (!poses_.waitFor(t_end, odom_wait_timeout_)) {
      noteMissingOdom();
      return false;
    }

    const auto snapshot = poses_.snapshot();
    Pose pose_end;
    bool extrapolated = false;
    if (!PoseBuffer::interpolate(snapshot, t_end, &pose_end, &extrapolated)) {
      return false;
    }

    DeskewStats stats;
    CloudT::Ptr scan = deskewer_->deskew(*raw, scan_stamp, snapshot, pose_end, &stats);
    if (!scan || scan->empty()) {
      return false;
    }

    char detail[160];
    std::snprintf(
      detail, sizeof(detail), " deskew span=%.3fs max=%.3fm mean=%.3fm",
      stats.span, stats.max_motion, stats.mean_motion);

    out->cloud = std::move(scan);
    out->pose_end = pose_end;
    out->stamp = t_end;
    out->raw_points = raw->size() + invalid;
    out->invalid = invalid + stats.invalid;
    out->detail = detail;
    out->odom_extrapolated = extrapolated || stats.extrapolated;
    return true;
  }

  // /cloud_registered: every VIO frame FAST-LIVO2 emits, already fused with the
  // camera, coloured and motion-compensated, expressed in its own world frame
  // (camera_init, numerically the same frame /odometry/lio reports in).
  //
  // The points carry the pose the front end had when it baked them, so the
  // body-frame conversion has to undo exactly that pose. FAST-LIVO2 reports
  // odometry *before* publishing the cloud in the same iteration and does not
  // report any during a VIO iteration, so the newest sample is that pose.
  // Interpolating at the cloud's publish stamp instead would pair the cloud
  // with a pose it was not built with, and the error would show up as a
  // per-keyframe shift every time the graph moves a node.
  bool prepareFrontendScan(const sensor_msgs::msg::PointCloud2 & msg, PreparedScan * out)
  {
    std::size_t invalid = 0;
    CloudT::Ptr world = cloudFromRos(msg, &invalid);
    // FAST-LIVO2 publishes an empty placeholder on LIO frames; skip those.
    if (!world || world->size() < 50) {
      return false;
    }

    PoseBuffer::Entry newest{};
    if (!poses_.newest(&newest)) {
      noteMissingOdom();
      return false;
    }

    const Eigen::Matrix4f T_body_world = newest.pose.inverse().matrix().cast<float>();
    auto body = std::make_shared<CloudT>();
    body->reserve(world->size());
    for (const auto & p : world->points) {
      const Eigen::Vector4f v = T_body_world * Eigen::Vector4f(p.x, p.y, p.z, 1.0f);
      PointT q = p;
      q.x = v.x();
      q.y = v.y();
      q.z = v.z();
      body->push_back(q);
    }
    body->is_dense = true;

    out->cloud = std::move(body);
    out->pose_end = newest.pose;
    out->stamp = newest.stamp;
    out->raw_points = world->size() + invalid;
    out->invalid = invalid;
    out->detail = " front-end cloud (undistorted by the front end)";
    out->odom_extrapolated = false;
    return true;
  }

  void noteMissingOdom()
  {
    ++dropped_no_odom_;
    if (dropped_no_odom_ % 20 == 1) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No odometry yet (%zu scans dropped); is the front end up?", dropped_no_odom_);
    }
  }

  // Pose-jump watch.
  //
  // Why it exists: a front end that loses its LIO/VIO lock does not stop
  // publishing and does not say so -- FAST-LIVO2 falls back to integrating the
  // IMU alone and the pose runs away. This is the only place in the back end
  // that can see that happen, so it is the only place that says so.
  //
  // By default it REPORTS AND ACCEPTS: a jump prints a WARN and the scan goes
  // into the map like any other. That is deliberate -- on real handheld data
  // the thresholds false-alarm (a fast turn, a dropped frame), and a survey
  // that silently stops growing is worse than one that says "pose jumped" and
  // carries on. sanity_refuse_scans=true restores the stricter mode described
  // at the bottom of this comment.
  //
  // Refuse mode (sanity_refuse_scans: true): a scan whose pose implies more than
  // sanity_max_speed or more than sanity_max_jump of displacement from the last
  // accepted pose is dropped -- no keyframe, no fusion, no loop search -- with a
  // WARN saying how far and how fast the front end moved. Re-acquisition: a
  // *single* bad jump followed by normal motion (a step in the odometry, a
  // dropped frame, a yank on the rig) must not lock the gate out for the rest of
  // the survey, so after sanity_reacquire_scans sane scan-to-scan steps within
  // sanity_reacquire_jump of the last accepted pose the gate adopts the new
  // position.
  //
  // Either way it cannot fix the front end, tell the operator where to walk back
  // to, or restart it.
  bool poseIsSane(const Pose & pose, double stamp)
  {
    const Eigen::Vector3d p = pose.matrix().block<3, 1>(0, 3).cast<double>();

    if (!have_gate_) {
      gate_pose_ = p;
      gate_stamp_ = stamp;
      have_gate_ = true;
      prev_pose_ = p;
      prev_stamp_ = stamp;
      have_prev_ = true;
      return true;
    }

    const double dt = std::max(stamp - gate_stamp_, 1e-3);
    const double jump = (p - gate_pose_).norm();
    const double speed = jump / dt;

    // Step since the previous scan, accepted or not: is the front end still
    // moving at a believable rate right now?
    const double step_dt = std::max(stamp - prev_stamp_, 1e-3);
    const double step_speed = (p - prev_pose_).norm() / step_dt;
    const bool step_ok = step_speed <= sanity_max_speed_;

    bool accept = jump <= sanity_max_jump_ && speed <= sanity_max_speed_;

    if (!sanity_refuse_scans_) {
      // Report-only: the verdict is printed, never acted on.
      if (accept) {
        jump_streak_ = 0;
      } else {
        ++jump_streak_;
        ++dropped_jump_;
        const int period = jump_streak_ == 1 ? 500 : 2000;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), period,
          "Pose jumped %.2f m in %.3f s (= %.1f m/s, limits %.2f m / %.1f m/s); "
          "%zu in a row, %zu total. The scan is being mapped anyway -- if this "
          "is a real divergence, stop and re-run the survey.",
          jump, dt, speed, sanity_max_jump_, sanity_max_speed_,
          static_cast<std::size_t>(jump_streak_), dropped_jump_.load());
      }
      gate_pose_ = p;
      gate_stamp_ = stamp;
      prev_pose_ = p;
      prev_stamp_ = stamp;
      have_prev_ = true;
      return true;
    }

    if (accept) {
      reacquire_streak_ = 0;
    } else if (step_ok && jump <= sanity_reacquire_jump_) {
      ++reacquire_streak_;
      if (reacquire_streak_ >= sanity_reacquire_scans_) {
        RCLCPP_WARN(
          get_logger(),
          "Front end pose came back: re-acquiring at %.2f m from the last good "
          "pose after %d sane scans (total %zu scans refused)",
          jump, reacquire_streak_, dropped_diverged_.load());
        accept = true;
        reacquire_streak_ = 0;
        // The keyframe this scan becomes (or the next one) is separated from
        // its predecessor by the jump, so the relative pose between them is not
        // a motion measurement -- see the odom edge in processScan().
        pending_reacquire_ = true;
      }
    } else {
      reacquire_streak_ = 0;
    }

    if (!accept) {
      ++dropped_diverged_;
      // First refusal is loud and immediate; the repeats are throttled but
      // carry the running count, so a dead front end is obvious at a glance.
      ++refused_streak_;
      const int period = refused_streak_ == 1 ? 500 : 2000;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), period,
        "Front end appears to have diverged: pose jumped %.2f m in %.3f s (= %.1f m/s, "
        "limits %.2f m / %.1f m/s); scan refused. %zu refused in a row, %zu total. "
        "The map is not being polluted, but nothing new is being added either.",
        jump, dt, speed, sanity_max_jump_, sanity_max_speed_,
        static_cast<std::size_t>(refused_streak_), dropped_diverged_.load());
    } else {
      refused_streak_ = 0;
      gate_pose_ = p;
      gate_stamp_ = stamp;
    }

    prev_pose_ = p;
    prev_stamp_ = stamp;
    have_prev_ = true;
    return accept;
  }

  void tryLoopClosure(int id)
  {
    auto & frames = keyframes_.frames();
    const auto candidates = loops_.candidates(id, frames);
    if (candidates.empty()) {
      return;
    }

    for (int j : candidates) {
      CloudT::Ptr target = keyframes_.localMap(
        j, loops_.options().local_map_neighbors, loops_.options().icp_target_voxel);
      if (!target || target->empty()) {
        continue;
      }
      const Pose guess = frames[j].pose.inverse() * frames[id].pose;
      const Verification v = loops_.verify(*frames[id].cloud, *target, guess);

      // fitness stays infinite when GICP bails out -- either it did not
      // converge, or one of the clouds was too small to try. Printing the
      // sizes separates the two, which is the difference between tuning the
      // registration and fixing the input.
      if (verbose_) {
        RCLCPP_INFO(
          get_logger(),
          "loop candidate kf %d -> %d: fitness=%.3f inliers=%.2f shift=%.2fm "
          "voxels=%zu/%zu loose=%d converged=%d %s",
          id, j, v.fitness, v.inlier_ratio,
          (v.from_T_to.t - guess.t).norm(), v.source_points, v.target_points,
          v.loose_inliers, v.converged ? 1 : 0, v.accepted ? "ACCEPTED" : "rejected");
      }

      if (!v.accepted) {
        continue;
      }

      LoopConstraint lc;
      lc.from = j;
      lc.to = id;
      lc.from_T_to = v.from_T_to;
      lc.fitness = v.fitness;
      lc.inlier_ratio = v.inlier_ratio;
      loop_constraints_.push_back(lc);
      graph_.addLoopEdge(j, id, v.from_T_to);

      if (verbose_) {
        RCLCPP_INFO(get_logger(), "Optimising pose graph (%zu loops)", loop_constraints_.size());
      }

      if (!applyOptimisation("loop")) {
        loop_constraints_.pop_back();
        graph_.removeLastLoopEdge();
        RCLCPP_WARN(get_logger(), "Pose graph optimisation failed; loop discarded");
      }
      return;  // one accepted loop per keyframe is enough
    }
  }

  // Batch solve and push the result into the keyframe poses, the published
  // map / path and the map -> odom correction. Every caller adds its edge or
  // anchor first and undoes it when this returns false.
  bool applyOptimisation(const char * what)
  {
    std::vector<Pose> optimised;
    double rms_shift = 0.0;
    if (!graph_.optimise(&optimised, &rms_shift) ||
      optimised.size() != keyframes_.frames().size())
    {
      return false;
    }
    auto & frames = keyframes_.frames();
    for (std::size_t k = 0; k < frames.size(); ++k) {
      frames[k].pose = optimised[k];
    }
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "Pose graph optimised (%s), RMS shift %.3f m", what, rms_shift);
    }
    // The map is fused incrementally, so the keyframes that just moved have to
    // be fused again. Deferred to the next publish/save: an optimisation can
    // arrive every second (GNSS anchors), and re-fusing on each one would undo
    // exactly what the incremental assembly is for.
    assembler_needs_rebuild_ = true;
    map_dirty_ = true;
    force_map_publish_ = true;
    publishLoops();
    publishPath();
    updateCorrection();
    return true;
  }

  // ------------------------------------------------- absolute position factor
  //
  // A GNSS-style fix is a position in *some* global frame, while the graph
  // lives in camera_init: the front end's own world, whose origin and heading
  // are whatever the robot happened to have when LIVO2 started. The two are
  // related by a constant rigid transform. It is estimated here from the data
  // rather than configured: the front end's raw odometry (odom_pose, which no
  // optimisation ever touches) and the fixes arriving at the same moments
  // describe the same motion in two frames, so a 2D rigid fit gives the
  // rotation and the offset.
  //
  // A least-squares rigid fit is used rather than two points: two 2 cm fixes
  // 1 m apart already disagree about the heading by a degree, and a heading
  // error is a lever arm that grows with distance from the reference point.
  // The fit needs at least three fixes spread over gnss_align_min_distance
  // (gnss_align_min_distance/2 in RMS distance from their centroid). Until
  // then, fixes feed the fit and are not used as factors. After that the fit
  // keeps being redone from every sample, and all committed anchors are
  // re-expressed, so the estimate keeps tightening as the robot drives.
  //
  // Everything runs on the worker thread. The ROS callback only parks the
  // newest fix.
  void processPendingGnss()
  {
    if (!gnss_enabled_) {
      return;
    }
    std::optional<GnssFix> fix;
    {
      std::lock_guard<std::mutex> lock(gnss_mtx_);
      fix = pending_gnss_;
      pending_gnss_.reset();
    }
    if (!fix || keyframes_.empty()) {
      return;
    }

    auto & frames = keyframes_.frames();

    // Closest keyframe in time. Keyframes are appended in stamp order, so walk
    // back until we are beyond the window.
    int k = -1;
    double best = gnss_match_window_;
    for (int j = static_cast<int>(frames.size()) - 1; j >= 0; --j) {
      const double dt = std::abs(frames[j].stamp - fix->stamp);
      if (dt <= best) {
        best = dt;
        k = j;
      }
      if (frames[j].stamp < fix->stamp - gnss_match_window_) {
        break;
      }
    }
    if (k < 0 || k == last_gnss_kf_) {
      return;
    }

    // The pose the front end reports *at the fix's own timestamp*, not at the
    // keyframe's. Keyframes are 0.15 m apart, which at 0.3 m/s is half a
    // second, so the nearest keyframe can be ~7 cm away from where the fix was
    // measured. Using the keyframe pose made that lag look like a 6 cm fit
    // residual and a matching bias in every innovation (measured on the depot
    // run before this was fixed). `lever` below carries the difference, so the
    // anchor is still placed on the keyframe.
    Eigen::Vector3d odom_at_fix = frames[k].odom_pose.t;
    {
      const auto snap = poses_.snapshot();
      Pose interp;
      bool extrapolated = false;
      if (!snap.empty() &&
        PoseBuffer::interpolate(snap, fix->stamp, &interp, &extrapolated) && !extrapolated)
      {
        odom_at_fix = interp.t;
      }
    }
    const Eigen::Vector3d lever = frames[k].odom_pose.t - odom_at_fix;

    // Every fix is both a sample for the frame alignment and, once there are
    // enough, an anchor. The fit is redone from every sample collected so far
    // on each pass and all anchors are re-expressed with it, rather than being
    // frozen after the first successful fit.
    //
    // Frozen-after-1-m was measured on the depot run: the fit residual was
    // 0.036-0.060 m and the anchors then sat at a 4-6 cm xy innovation floor,
    // because a fit over a 1 m drive has a ~1.5 deg heading error and a
    // heading error is a lever arm. Refitting as the robot drives turns that
    // floor into a decaying quantity: heading error falls as 1/spread, so a
    // 5 m drive is already three times better than the 1 m one, and every
    // anchor picked up along the way is corrected with it.
    gnss_samples_.emplace_back(odom_at_fix, fix->p);
    if (gnss_samples_.size() > kMaxGnssSamples) {
      gnss_samples_.erase(gnss_samples_.begin());
    }
    last_gnss_kf_ = k;

    const std::size_t n = gnss_samples_.size();
    Eigen::Vector2d c_odom = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_fix = Eigen::Vector2d::Zero();
    double c_odom_z = 0.0;
    double c_fix_z = 0.0;
    for (const auto & s : gnss_samples_) {
      c_odom += s.first.head<2>();
      c_fix += s.second.head<2>();
      c_odom_z += s.first.z();
      c_fix_z += s.second.z();
    }
    const double inv_n = 1.0 / static_cast<double>(n);
    c_odom *= inv_n;
    c_fix *= inv_n;
    c_odom_z *= inv_n;
    c_fix_z *= inv_n;

    // Spread of the fixes about their centroid. A straight 2 m drive with
    // three samples has ~0.8 m, so gnss_align_min_distance/2 in this measure
    // means "drove about min_distance". It is a spread, not a baseline, so
    // driving out and back does not make a degenerate fit look well
    // conditioned.
    double spread2 = 0.0;
    double s_dot = 0.0;   // sum (d_fix . d_odom), the Procrustes cosine term
    double s_cross = 0.0; // sum (d_fix x d_odom), the sine term
    for (const auto & s : gnss_samples_) {
      const Eigen::Vector2d d_fix = s.second.head<2>() - c_fix;
      const Eigen::Vector2d d_odom = s.first.head<2>() - c_odom;
      spread2 += d_fix.squaredNorm();
      s_dot += d_fix.dot(d_odom);
      s_cross += d_fix.x() * d_odom.y() - d_fix.y() * d_odom.x();
    }
    spread2 *= inv_n;
    const double spread = std::sqrt(spread2);
    if (n < 3 || spread < 0.5 * gnss_align_min_distance_) {
      return;
    }

    const double yaw = std::atan2(s_cross, s_dot);
    const Eigen::Matrix3d R = Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector2d t_xy =
      c_odom - (R * Eigen::Vector3d(c_fix.x(), c_fix.y(), 0.0)).head<2>();
    const Eigen::Vector3d t(t_xy.x(), t_xy.y(), c_odom_z - c_fix_z);

    double fit2 = 0.0;
    for (const auto & s : gnss_samples_) {
      const Eigen::Vector3d p = R * s.second + t;
      fit2 += (p.head<2>() - s.first.head<2>()).squaredNorm();
    }
    const double fit_rms = std::sqrt(fit2 * inv_n);

    // A different heading is the one change worth a line of its own: it is the
    // quantity the fit is worst at and the one that biases every anchor.
    const bool notify = !have_gnss_align_ || std::abs(std::remainder(yaw - gnss_yaw_, 2.0 * M_PI)) > 0.01;
    gnss_R_ = R;
    gnss_t_ = t;
    gnss_yaw_ = yaw;
    have_gnss_align_ = true;
    if (notify && verbose_) {
      RCLCPP_INFO(
        get_logger(),
        "gnss: global <- odom alignment from %zu fixes, %.2f m spread: yaw %.4f rad, "
        "t (%.3f, %.3f, %.3f), fit residual %.3f m",
        n, spread, yaw, t.x(), t.y(), t.z(), fit_rms);
    }

    // Re-express every anchor with the refined transform. The samples are raw
    // (front-end odometry, which no optimisation touches, and raw fixes), so
    // refitting cannot feed back on itself.
    std::vector<GnssAnchorRecord> records = gnss_records_;
    records.push_back(GnssAnchorRecord{k, fix->p, lever});
    std::vector<Anchor> anchors;
    anchors.reserve(records.size());
    for (const auto & r : records) {
      Anchor a;
      a.id = r.kf;
      a.p = R * r.fix + t + r.lever;
      a.sigmas = gnss_sigmas_;
      a.robust = gnss_robust_;
      anchors.push_back(a);
    }
    graph_.setAnchors(anchors);

    const Eigen::Vector3d p_map = R * fix->p + t + lever;
    const Eigen::Vector3d residual = p_map - frames[k].pose.t;
    const double innovation = residual.norm();

    if (verbose_) {
      RCLCPP_INFO(
        get_logger(),
        "gnss anchor %zu: kf %d (%.2f s), fix %.2f s, %.2f m away, fixed at "
        "(%.3f, %.3f, %.3f), innovation %.3f m (xy %.3f, z %.3f)",
        gnss_anchors_ + 1, k, frames[k].stamp, fix->stamp, lever.norm(),
        p_map.x(), p_map.y(), p_map.z(), innovation, residual.head<2>().norm(),
        std::abs(residual.z()));
    }

    if (applyOptimisation("gnss")) {
      ++gnss_anchors_;
      gnss_records_ = std::move(records);
      // Keep the factor count bounded on a long run without giving up the
      // global span: dropping every second anchor halves the density and keeps
      // the oldest and newest ones.
      if (gnss_records_.size() > kMaxGnssAnchors) {
        std::vector<GnssAnchorRecord> thinned;
        thinned.reserve(gnss_records_.size() / 2 + 1);
        for (std::size_t i = 0; i < gnss_records_.size(); i += 2) {
          thinned.push_back(gnss_records_[i]);
        }
        gnss_records_ = std::move(thinned);
        if (verbose_) {
          RCLCPP_INFO(
            get_logger(), "gnss: anchor set thinned to %zu factors", gnss_records_.size());
        }
      }
    } else {
      // graph_ still holds `anchors`, which includes the new one; put back the
      // committed set.
      graph_.setAnchors(anchorsFromRecords(gnss_records_));
      RCLCPP_WARN(get_logger(), "Pose graph optimisation failed; gnss anchor discarded");
    }
  }

  std::vector<Anchor> anchorsFromRecords(const std::vector<GnssAnchorRecord> & records) const
  {
    std::vector<Anchor> anchors;
    anchors.reserve(records.size());
    for (const auto & r : records) {
      Anchor a;
      a.id = r.kf;
      a.p = gnss_R_ * r.fix + gnss_t_ + r.lever;
      a.sigmas = gnss_sigmas_;
      a.robust = gnss_robust_;
      anchors.push_back(a);
    }
    return anchors;
  }

  // --------------------------------------------------------------- publishing
  // Lifts a cloud built in camera_init numbers into the map frame. Applied at
  // the output boundary only: the graph, the loop registration and the stored
  // keyframes all stay in the front end's convention, so nothing inside the
  // back end has to know about the offset.
  void shiftIntoMapFrame(CloudT & cloud) const
  {
    for (auto & p : cloud.points) {
      p.x += output_translation_.x();
      p.y += output_translation_.y();
      p.z += output_translation_.z();
    }
  }

  // Where a keyframe pose should be drawn. A keyframe pose is base_link, but
  // /path (FAST-LIVO2) and /ground_truth/path are both drawn at the IMU, which
  // sits trajectory_imu_offset_ above base_link, so drawing ours at base_link
  // puts it half a metre under the other two and they cannot be compared by
  // eye. Rotating the offset by the pose keeps it rigidly attached when the
  // robot pitches. Set trajectory_imu_offset to [0, 0, 0] to draw the true
  // base_link trajectory again.
  Eigen::Vector3f displayPosition(const Pose & pose) const
  {
    Eigen::Vector3f p = pose.t.cast<float>();
    if (trajectory_imu_offset_.squaredNorm() > 0.0f) {
      p += pose.q.toRotationMatrix().cast<float>() * trajectory_imu_offset_;
    }
    return p + output_translation_;
  }

  // The fused map in the graph frame. Keyframes are fused as they arrive, so
  // this is cheap unless the pose graph moved them, in which case everything
  // is fused again once and the result is cached. Both the publisher and the
  // save path go through here.
  CloudT::Ptr currentMap()
  {
    if (assembler_needs_rebuild_) {
      assembler_.rebuild(keyframes_.frames());
      assembler_needs_rebuild_ = false;
      if (verbose_) {
        RCLCPP_INFO(
          get_logger(), "Map rebuilt after pose-graph update: %zu cells from %zu keyframes",
          assembler_.cellCount(), keyframes_.size());
      }
    }
    return assembler_.build();
  }

  void maybePublishMap()
  {
    if (!map_dirty_) {
      return;
    }
    const auto tick = std::chrono::steady_clock::now();
    if (!force_map_publish_ && tick - last_map_publish_ <
      std::chrono::duration<double>(map_publish_period_))
    {
      return;
    }
    force_map_publish_ = false;
    last_map_publish_ = tick;
    map_dirty_ = false;

    CloudT::Ptr cloud = currentMap();
    if (!cloud || cloud->empty()) {
      return;
    }
    // currentMap() hands back the assembler's own cloud; the shift below moves
    // points, so copy it rather than corrupting the cache. The rigs without a
    // base_link lift (the handheld ones) have a zero offset and skip the copy.
    if (output_translation_.squaredNorm() > 0.0f) {
      cloud = std::make_shared<CloudT>(*cloud);
      shiftIntoMapFrame(*cloud);
    }
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*cloud, out);
    out.header.frame_id = map_frame_;
    out.header.stamp = now();
    map_pub_->publish(out);
    if (verbose_) {
      RCLCPP_INFO(
        get_logger(), "Published map: %zu points from %zu keyframes (%zu scans dropped, %zu refused)",
        cloud->size(), keyframes_.size(), dropped_scans_.load(), dropped_diverged_.load());
    }
  }

  void publishCloud(const CloudT & cloud_in_own_frame, const Pose & pose)
  {
    const Eigen::Matrix4f T = pose.matrix().cast<float>();
    auto moved = std::make_shared<CloudT>();
    pcl::transformPointCloud(cloud_in_own_frame, *moved, T);
    shiftIntoMapFrame(*moved);
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*moved, out);
    out.header.frame_id = map_frame_;
    out.header.stamp = now();
    scan_pub_->publish(out);
  }

  void publishPath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    const auto & frames = keyframes_.frames();
    path.poses.reserve(frames.size());
    for (const auto & kf : frames) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.header.stamp = rclcpp::Time(
        static_cast<int64_t>(kf.stamp * 1e9), RCL_ROS_TIME);
      ps.pose = toRosPose(kf.pose);
      const Eigen::Vector3f shown = displayPosition(kf.pose);
      ps.pose.position.x = shown.x();
      ps.pose.position.y = shown.y();
      ps.pose.position.z = shown.z();
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
  }

  void publishLoops()
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = now();
    m.ns = "pgo_loops";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.05;
    m.color.r = 0.1f;
    m.color.g = 1.0f;
    m.color.b = 0.1f;
    m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;

    const auto & frames = keyframes_.frames();
    for (const auto & lc : loop_constraints_) {
      if (lc.from < 0 || lc.from >= static_cast<int>(frames.size()) ||
        lc.to < 0 || lc.to >= static_cast<int>(frames.size()))
      {
        continue;
      }
      geometry_msgs::msg::Point a, b;
      const Eigen::Vector3f pa = displayPosition(frames[lc.from].pose);
      const Eigen::Vector3f pb = displayPosition(frames[lc.to].pose);
      a.x = pa.x();
      a.y = pa.y();
      a.z = pa.z();
      b.x = pb.x();
      b.y = pb.y();
      b.z = pb.z();
      m.points.push_back(a);
      m.points.push_back(b);
    }
    arr.markers.push_back(m);
    loop_pub_->publish(arr);
  }

  // map -> odom, so that the frame chain reports the *optimised* base pose
  // while odom -> base_link keeps coming from the front end untouched.
  //
  // The transform only changes when a keyframe is added or a loop closes, but
  // it is broadcast on a timer: a consumer asking for "now" needs data at
  // "now", and tf2 will not extrapolate past the last sample.
  // The correction currently in force, as a value. Identity until the first
  // keyframe exists, which is exactly what the frame tree assumes (odom and
  // map coincide before the back end has anything to say).
  Pose currentCorrection()
  {
    std::lock_guard<std::mutex> lock(correction_mtx_);
    return have_correction_ ? correction_ : Pose{};
  }

  void updateCorrection()
  {
    if (!publish_correction_tf_ || keyframes_.empty()) {
      return;
    }
    const Keyframe & kf = keyframes_.frames().back();
    // T_map_odom = T_map_base(optimised) * T_odom_base(front end)^-1
    const Pose T_map_odom = kf.pose * kf.odom_pose.inverse();
    std::lock_guard<std::mutex> lock(correction_mtx_);
    correction_ = T_map_odom;
    have_correction_ = true;
  }

  void broadcastCorrection()
  {
    if (!publish_correction_tf_) {
      return;
    }
    Pose T_map_odom;
    {
      std::lock_guard<std::mutex> lock(correction_mtx_);
      if (!have_correction_) {
        return;
      }
      T_map_odom = correction_;
    }

    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = now();
    ts.header.frame_id = map_frame_;
    ts.child_frame_id = odom_frame_;
    ts.transform.translation.x = T_map_odom.t.x();
    ts.transform.translation.y = T_map_odom.t.y();
    ts.transform.translation.z = T_map_odom.t.z();
    ts.transform.rotation.x = T_map_odom.q.x();
    ts.transform.rotation.y = T_map_odom.q.y();
    ts.transform.rotation.z = T_map_odom.q.z();
    ts.transform.rotation.w = T_map_odom.q.w();
    tf_broadcaster_->sendTransform(ts);
  }

  // -------------------------------------------------------------------- misc
  std::string defaultPcdPath() const
  {
    const auto now_ns = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    // $HOME rather than /tmp: this path is also the shutdown fallback, and a
    // survey is exactly the kind of thing that must not be parked in a
    // directory that a reboot clears.
    const char * home = std::getenv("HOME");
    const std::string dir = (home && home[0]) ? std::string(home) : std::string("/tmp");
    return dir + "/pgo_map_" + std::to_string(now_ns) + ".pcd";
  }

  // ---------------------------------------------------- save: map + deliverables
  //
  // The map alone is not a usable result for a survey: the optimised
  // trajectory is what gets compared against a reference or fed to
  // CloudCompare/evo, and the accepted loops are the only record of *why* the
  // map looks the way it does. Both are written next to the PCD, so one call
  // produces the whole deliverable.
  std::string stripPcdSuffix(const std::string & path) const
  {
    const std::string suf = ".pcd";
    if (path.size() > suf.size() &&
      path.compare(path.size() - suf.size(), suf.size(), suf) == 0)
    {
      return path.substr(0, path.size() - suf.size());
    }
    return path;
  }

  // TUM: "stamp tx ty tz qx qy qz qw", one optimised keyframe pose per line,
  // in the map frame (same translation convention as the PCD). Nanoseconds of
  // the keyframe stamp are kept as a decimal so the rows stay aligned with
  // other TUM files down to the LiDAR's own precision.
  void saveTrajectory(const std::string & path) const
  {
    std::ofstream f(path);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to write %s", path.c_str());
      return;
    }
    f << std::fixed << std::setprecision(9);
    const Eigen::Vector3f off = output_translation_;
    for (const auto & kf : keyframes_.frames()) {
      const Eigen::Vector3f p = kf.pose.t.cast<float>() + off;
      f << kf.stamp << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' '
        << kf.pose.q.x() << ' ' << kf.pose.q.y() << ' ' << kf.pose.q.z() << ' '
        << kf.pose.q.w() << '\n';
    }
    f.close();
    RCLCPP_INFO(
      get_logger(), "Saved %zu optimised poses to %s",
      keyframes_.size(), path.c_str());
  }

  void saveLoops(const std::string & path) const
  {
    std::ofstream f(path);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to write %s", path.c_str());
      return;
    }
    f << "from,to,fitness,inlier_ratio\n";
    f << std::fixed << std::setprecision(4);
    for (const auto & lc : loop_constraints_) {
      f << lc.from << ',' << lc.to << ',' << lc.fitness << ',' << lc.inlier_ratio << '\n';
    }
    f.close();
    RCLCPP_INFO(
      get_logger(), "Saved %zu loop closures to %s",
      loop_constraints_.size(), path.c_str());
  }

  void saveMap(const std::string & path)
  {
    CloudT::Ptr cloud = currentMap();
    if (!cloud || cloud->empty()) {
      RCLCPP_WARN(get_logger(), "Nothing to save: map is empty");
      return;
    }
    // Same convention as /pgo/map, so a saved PCD lines up with the front
    // end's own clouds.
    if (output_translation_.squaredNorm() > 0.0f) {
      cloud = std::make_shared<CloudT>(*cloud);
      shiftIntoMapFrame(*cloud);
    }
    if (MapAssembler::savePcd(*cloud, path)) {
      RCLCPP_INFO(get_logger(), "Saved %zu points to %s", cloud->size(), path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to write %s", path.c_str());
    }

    if (save_extra_outputs_) {
      const std::string base = stripPcdSuffix(path);
      saveTrajectory(base + "_traj.tum");
      saveLoops(base + "_loops.csv");
    }
  }

  // -------------------------------------------------------------- parameters
  std::string cloud_source_, scan_topic_, frontend_cloud_topic_;
  std::string odom_topic_, map_frame_, odom_frame_;
  Eigen::Vector3f output_translation_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f trajectory_imu_offset_ = Eigen::Vector3f::Zero();
  bool frontend_source_ = true;
  bool deskew_enabled_ = true;
  double odom_wait_timeout_ = 0.5;
  bool loop_enabled_ = true;

  // Absolute position factor. All of this is touched on the worker thread
  // only; pending_gnss_ is the single hand-off from the subscription.
  // GnssFix / GnssAnchorRecord are declared at the top of the class.
  bool gnss_enabled_ = false;
  std::string gnss_topic_;
  std::array<double, 3> gnss_sigmas_{{0.02, 0.02, 0.05}};
  double gnss_robust_ = 0.0;
  double gnss_match_window_ = 0.5;
  double gnss_align_min_distance_ = 1.0;
  std::mutex gnss_mtx_;
  std::optional<GnssFix> pending_gnss_;
  // (front-end odom position, fix position) pairs, kept for the whole run so
  // the rigid fit keeps improving as the robot covers ground.
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> gnss_samples_;
  // Anchors already committed, in fix-frame coordinates (see GnssAnchorRecord).
  std::vector<GnssAnchorRecord> gnss_records_;
  bool have_gnss_align_ = false;
  double gnss_yaw_ = 0.0;
  Eigen::Matrix3d gnss_R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d gnss_t_ = Eigen::Vector3d::Zero();
  int last_gnss_kf_ = -1;
  std::size_t gnss_anchors_ = 0;

  bool publish_correction_tf_ = true;
  bool publish_current_scan_ = false;
  double map_publish_period_ = 2.0;
  std::string pcd_path_;
  bool save_on_shutdown_ = false;
  bool save_extra_outputs_ = false;
  int scan_queue_size_ = 2;

  // ----------------------------------------------------------------- modules
  PoseBuffer poses_{5000};
  std::unique_ptr<Deskewer> deskewer_;
  KeyframeManager keyframes_;
  LoopDetector loops_;
  PoseGraph graph_;
  MapAssembler assembler_;
  std::vector<LoopConstraint> loop_constraints_;

  // ------------------------------------------------------------- data / sync
  std::mutex queue_mtx_;
  std::condition_variable queue_cv_;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> scan_queue_;
  bool save_requested_ = false;
  std::atomic<bool> running_{false};
  std::thread worker_;

  std::atomic<std::size_t> dropped_scans_{0};
  std::size_t dropped_no_odom_ = 0;

  // --------------------------------------------------- console volume
  bool verbose_ = false;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr verbose_cb_;

  // ------------------------------------------------- pose-jump watch
  bool sanity_enabled_ = true;
  bool sanity_refuse_scans_ = false;  // false: report the jump, map it anyway
  double sanity_max_speed_ = 3.0;   // m/s
  double sanity_max_jump_ = 0.5;    // m between consecutive processed scans
  int sanity_reacquire_scans_ = 20;
  double sanity_reacquire_jump_ = 2.0;  // m, how far re-acquisition may reach
  Eigen::Vector3d gate_pose_ = Eigen::Vector3d::Zero();  // last accepted
  double gate_stamp_ = 0.0;
  bool have_gate_ = false;
  Eigen::Vector3d prev_pose_ = Eigen::Vector3d::Zero();  // last received
  double prev_stamp_ = 0.0;
  bool have_prev_ = false;
  int reacquire_streak_ = 0;
  int refused_streak_ = 0;
  int jump_streak_ = 0;  // report-only mode: consecutive jumps
  bool pending_reacquire_ = false;  // next keyframe's odom edge gets widened
  double graph_odom_sigma_t_ = 0.02;  // metres, from graph_odom_sigmas[0]
  std::atomic<std::size_t> dropped_diverged_{0};
  std::atomic<std::size_t> dropped_jump_{0};  // jumps reported, scans still mapped

  std::mutex correction_mtx_;
  Pose correction_;
  bool have_correction_ = false;

  bool map_dirty_ = false;
  bool assembler_needs_rebuild_ = false;
  bool force_map_publish_ = false;
  std::chrono::steady_clock::time_point last_map_publish_{};

  // ---------------------------------------------------------------- ROS bits
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr correction_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace pgo
}  // namespace vector_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vector_mapping::pgo::PgoBackendNode>();
  rclcpp::spin(node);
  node->stop();
  rclcpp::shutdown();
  return 0;
}
