// Bridges FAST-LIVO2's odometry into the frame conventions the rest of
// vector_robot expects.
//
// FAST-LIVO2 estimates the pose of its IMU body frame: /aft_mapped_to_init is
// the camera_init -> aft_mapped transform, where aft_mapped IS the IMU frame
// (FAST-LIVO2's state is the IMU state; the LiDAR points are moved into that
// frame with extrinsic_T/extrinsic_R before the update). Because the simulated
// IMU is mounted on the 45 deg nose-down LiDAR body, the world frame
// (camera_init) is gravity aligned but the reported body orientation carries
// that 45 deg pitch at standstill.
//
// Everything downstream (ekf_odom, nav2, RViz) wants the pose of base_link in
// an "odom" frame whose z is up. This node therefore publishes
//
//     T_odom_base = T_camera_init_imu * T_imu_base
//
// where T_imu_base is a constant read from the URDF. The 45 deg pitch is
// cancelled by construction, so the published pose is flat at standstill.
//
// There is a second, independent question: where is the ORIGIN of "odom". The
// answer this stack uses is "at base_link at init", i.e. the same convention as
// sim_min_livo2.launch.xml's static odom -> camera_init transform, which puts
// camera_init at +t_base_imu from odom. The pose above does not satisfy that on
// its own: at init T_camera_init_imu is the identity and the result is
// T_imu_base, whose origin is MINUS t_base_imu, so base_link comes out half a
// metre below the origin.
//
// That is why /odometry/lio's numbers are camera_init coordinates, and every
// consumer of it compensates on its own (odometry_path_node's offset for
// /path_base, output_translation in the back end for /pgo/map). The TF cannot
// compensate: it is consumed as-is by RViz's robot model, so with the numbers
// untouched the robot is drawn 0.5 m under the floor that the clouds, the
// map, /path_base and /ground_truth/path all agree on. publish_tf therefore
// adds the same constant lift, which makes odom -> base_link consistent with
// odom -> camera_init and with everything drawn through it. The message is
// left alone so the existing compensations stay valid; flip lift_tf_to_odom to
// false to get the raw camera_init numbers in the TF again.
//
// The input clock is the simulation clock (use_sim_time), so the stamp is
// passed through unchanged.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class Livo2OdomBridge : public rclcpp::Node
{
public:
  Livo2OdomBridge() : Node("livo2_odom_bridge")
  {
    // Defaults are derived from vector_description/urdf:
    //   base_link -> livox_sensor  xyz (0, 0, 0.506), rpy (0, +pi/4, 0)
    //   livox_sensor -> livox_frame xyz (0.011, 0.02329, 0.00288), rpy 0
    // so the IMU origin in base_link is
    //   (0, 0, 0.506) + Ry(pi/4) * (0.011, 0.02329, 0.00288)
    //   = (0.0098146, 0.02329, 0.500258) with the base_link -> IMU rotation
    //   Ry(pi/4).
    t_base_imu_ = declare_parameter<std::vector<double>>(
      "base_to_imu_translation", {0.0098146, 0.02329, 0.5002583});
    if (t_base_imu_.size() != 3) {
      throw std::runtime_error("base_to_imu_translation must have 3 elements");
    }
    const double rpy_roll = declare_parameter<double>("base_to_imu_roll", 0.0);
    const double rpy_pitch = declare_parameter<double>("base_to_imu_pitch", M_PI / 4.0);
    const double rpy_yaw = declare_parameter<double>("base_to_imu_yaw", 0.0);

    output_frame_ = declare_parameter<std::string>("output_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    input_topic_ = declare_parameter<std::string>("input_topic", "/aft_mapped_to_init");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odometry/lio");
    // Publish odom -> base_link here instead of going through ekf_odom. With
    // LIVO2 as the only source the EKF has nothing to fuse: its sole input
    // carries no velocity, so it differentiates the 10 Hz position and the
    // velocity estimate wanders at rest, which shows up as a few millimetres
    // of shake in the robot's TF. Off by default in the launch file's
    // configuration only because the caller decides which node owns the edge.
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    // See the header comment: puts the TF's odom origin at base_link at init.
    lift_tf_to_odom_ = declare_parameter<bool>("lift_tf_to_odom", true);

    tf2::Transform T_base_imu;
    tf2::Quaternion q_base_imu;
    q_base_imu.setRPY(rpy_roll, rpy_pitch, rpy_yaw);
    T_base_imu.setRotation(q_base_imu.normalized());
    T_base_imu.setOrigin(
      tf2::Vector3(t_base_imu_[0], t_base_imu_[1], t_base_imu_[2]));

    // T_base_imu maps a point in the IMU frame into base_link; we need the
    // other direction, because the left multiplication above is
    // T_odom_base = T_odom_imu * T_imu_base.
    T_imu_base_ = T_base_imu.inverse();
    R_base_imu_.setRotation(T_base_imu.getRotation());
    // base_link -> IMU origin, expressed in the initial base_link frame, which
    // is also the vector from odom's origin to camera_init's origin.
    t_lift_ = tf2::Vector3(t_base_imu_[0], t_base_imu_[1], t_base_imu_[2]);

    pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, 10, std::bind(&Livo2OdomBridge::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Bridging %s (IMU body) -> %s (%s) with T_imu_base: t=(%.4f, %.4f, %.4f)",
      input_topic_.c_str(), output_topic_.c_str(), base_frame_.c_str(),
      T_imu_base_.getOrigin().x(), T_imu_base_.getOrigin().y(), T_imu_base_.getOrigin().z());
  }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Transform T_world_imu;
    tf2::fromMsg(msg->pose.pose, T_world_imu);

    const tf2::Transform T_world_base = T_world_imu * T_imu_base_;

    nav_msgs::msg::Odometry out;
    out.header = msg->header;
    out.header.frame_id = output_frame_;
    out.child_frame_id = base_frame_;
    tf2::toMsg(T_world_base, out.pose.pose);
    out.pose.covariance = msg->pose.covariance;

    // Twist is expressed in the child frame (REP 105), so it has to be
    // rotated out of the IMU frame. FAST-LIVO2 leaves it at zero, but
    // rotating it anyway keeps this correct if that ever changes.
    const tf2::Vector3 v_in(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    const tf2::Vector3 w_in(
      msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    const tf2::Vector3 v_out = R_base_imu_ * v_in;
    const tf2::Vector3 w_out = R_base_imu_ * w_in;
    out.twist.twist.linear.x = v_out.x();
    out.twist.twist.linear.y = v_out.y();
    out.twist.twist.linear.z = v_out.z();
    out.twist.twist.angular.x = w_out.x();
    out.twist.twist.angular.y = w_out.y();
    out.twist.twist.angular.z = w_out.z();
    out.twist.covariance = msg->twist.covariance;

    pub_->publish(out);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped ts;
      // The input stamp, not now(): the pose describes where the robot was
      // when LIVO2 solved that frame, and stamping it with the arrival time
      // would fabricate motion whenever the front end runs behind the clock.
      ts.header = out.header;
      ts.child_frame_id = base_frame_;
      const tf2::Vector3 tf_t =
        T_world_base.getOrigin() + (lift_tf_to_odom_ ? t_lift_ : tf2::Vector3(0, 0, 0));
      ts.transform.translation.x = tf_t.x();
      ts.transform.translation.y = tf_t.y();
      ts.transform.translation.z = tf_t.z();
      const tf2::Quaternion q = T_world_base.getRotation();
      ts.transform.rotation.x = q.x();
      ts.transform.rotation.y = q.y();
      ts.transform.rotation.z = q.z();
      ts.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(ts);
    }
  }

  bool publish_tf_ = true;
  bool lift_tf_to_odom_ = true;
  tf2::Vector3 t_lift_ = tf2::Vector3(0, 0, 0);
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<double> t_base_imu_;
  std::string output_frame_;
  std::string base_frame_;
  std::string input_topic_;
  std::string output_topic_;

  tf2::Transform T_imu_base_;
  tf2::Matrix3x3 R_base_imu_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Livo2OdomBridge>());
  rclcpp::shutdown();
  return 0;
}
