// The camera node this rig uses instead of usb_cam.
//
// Why not usb_cam 0.8.1 (measured on this machine, C310 at 640x480 YUYV):
// it delivers every frame 0.1-1.0 s after that frame's capture timestamp --
// eleven independent starts gave 98, 229, 303, 406, 422, 456, 636, 757, 896,
// 1008 and 1011 ms. The number is fixed for the life of the process (sigma 2-6
// ms, flat over 42 s) and does not respond to anything above it: same for
// RELIABLE and BEST_EFFORT at depth 1 and 10, same for two subscribers that
// attached at different times, unchanged when its timer was retimed from 33 ms
// to 16 ms at runtime. A minimal V4L2 reader on the same device with the same
// four mmap buffers sees 33-48 ms, and v4l2-ctl agrees. So the delay is inside
// usb_cam: its capture loop takes exactly one DQBUF per wall-timer tick, which
// leaves it no way to drain a backlog once one exists, and on EAGAIN it
// republishes the previous frame under the previous stamp.
//
// This node has no timer. It polls the device, then dequeues until EAGAIN,
// keeps only the newest frame, requeues the rest and publishes immediately.
// Whatever the driver has queued is therefore discarded every cycle instead of
// being played back later.
//
// The stamp is unchanged in meaning: the V4L2 buffer timestamp (CLOCK_MONOTONIC,
// start of exposure) shifted to wall clock, exactly what usb_cam computed. It is
// still honest about the frame it is attached to, which is why FAST-LIVO2's
// association is unaffected by switching capture nodes.
//
// Deliberately not implemented:
//   * V4L2 controls -- scripts/set_camera_controls.sh owns those. usb_cam tried
//     to write them (brightness 50 and auto-exposure on by default) and this
//     camera rejects most of it; here nothing fights the script.
//   * camera_info -- only the camera_calibration launches need it, and those
//     keep using usb_cam.
//   * pixel formats other than YUYV. MJPEG costs a decode and measured worse.
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/videodev2.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hm
{

namespace
{

std::int64_t wall_us()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::int64_t mono_us()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
}

void check(int ret, const char * what)
{
  if (ret < 0) {
    throw std::runtime_error(std::string(what) + ": " + std::strerror(errno));
  }
}

}  // namespace

class V4l2Capture : public rclcpp::Node
{
public:
  V4l2Capture()
  : rclcpp::Node("v4l2_capture")
  {
    device_ = declare_parameter<std::string>("video_device", "/dev/video0");
    width_ = declare_parameter<int>("image_width", 640);
    height_ = declare_parameter<int>("image_height", 480);
    framerate_ = declare_parameter<double>("framerate", 30.0);
    pixel_format_ = declare_parameter<std::string>("pixel_format", "yuyv");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
    buffer_count_ = declare_parameter<int>("buffer_count", 4);
    stats_period_ = declare_parameter<double>("stats_period", 10.0);

    if (pixel_format_ != "yuyv") {
      throw std::runtime_error(
              "pixel_format '" + pixel_format_ + "' is not implemented; this node only "
              "speaks YUYV (usb_cam handles mjpeg if you really need it)");
    }

    // RELIABLE to match what usb_cam published and what FAST-LIVO2 asks for
    // (create_subscription<Image>(topic, 200000) -- default reliability). A
    // BEST_EFFORT publisher here would be silently incompatible and the front
    // end would simply never see an image. The depth is deliberately small:
    // usb_cam used 100, which lets a slow consumer turn into a 3 s backlog
    // instead of a dropped frame.
    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    msg_ = std::make_unique<sensor_msgs::msg::Image>();

    open_device();
    init_mmap();
    start_streaming();

    RCLCPP_INFO(
      get_logger(), "capturing %dx%d %s @ %.1f fps from %s, %lu buffers",
      width_, height_, encoding_.c_str(), framerate_, device_.c_str(),
      static_cast<unsigned long>(buffers_.size()));

    running_ = true;
    thread_ = std::thread(&V4l2Capture::capture_loop, this);
  }

  ~V4l2Capture() override
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    stop_streaming();
    for (auto & b : buffers_) {
      if (b.start != nullptr) {
        munmap(b.start, b.length);
      }
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  struct Buffer
  {
    void * start{nullptr};
    std::size_t length{0};
  };

  // ------------------------------------------------------------------ setup
  void open_device()
  {
    // Blocking mode: poll() decides when to look, DQBUF is then drained until
    // EAGAIN with MSG_DONTWAIT semantics of a non-blocking descriptor.
    fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
      throw std::runtime_error("open " + device_ + ": " + std::strerror(errno));
    }

    v4l2_capability cap{};
    check(ioctl(fd_, VIDIOC_QUERYCAP, &cap), "VIDIOC_QUERYCAP");
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
      throw std::runtime_error(device_ + " is not a video capture device");
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<std::uint32_t>(width_);
    fmt.fmt.pix.height = static_cast<std::uint32_t>(height_);
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    check(ioctl(fd_, VIDIOC_S_FMT, &fmt), "VIDIOC_S_FMT");

    // The driver is allowed to give back something else; refuse to pretend it
    // worked, because a different geometry silently breaks the intrinsics.
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
      throw std::runtime_error("driver refused YUYV");
    }
    if (static_cast<int>(fmt.fmt.pix.width) != width_ ||
      static_cast<int>(fmt.fmt.pix.height) != height_)
    {
      throw std::runtime_error(
              "driver chose " + std::to_string(fmt.fmt.pix.width) + "x" +
              std::to_string(fmt.fmt.pix.height) + " instead of the requested size");
    }
    step_ = fmt.fmt.pix.bytesperline > 0 ? fmt.fmt.pix.bytesperline : width_ * 2;
    encoding_ = "yuv422_yuy2";

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = static_cast<std::uint32_t>(framerate_);
    if (ioctl(fd_, VIDIOC_S_PARM, &parm) == 0) {
      const auto & tpf = parm.parm.capture.timeperframe;
      RCLCPP_INFO(
        get_logger(), "driver reports %u/%u s per frame", tpf.numerator, tpf.denominator);
    } else {
      RCLCPP_WARN(get_logger(), "VIDIOC_S_PARM failed, using the driver's default frame rate");
    }
  }

  void init_mmap()
  {
    v4l2_requestbuffers req{};
    req.count = static_cast<std::uint32_t>(std::max(2, buffer_count_));
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    check(ioctl(fd_, VIDIOC_REQBUFS, &req), "VIDIOC_REQBUFS");
    if (req.count < 2) {
      throw std::runtime_error("driver granted fewer than 2 buffers");
    }

    buffers_.resize(req.count);
    for (std::uint32_t i = 0; i < req.count; ++i) {
      v4l2_buffer buf{};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      check(ioctl(fd_, VIDIOC_QUERYBUF, &buf), "VIDIOC_QUERYBUF");

      buffers_[i].length = buf.length;
      buffers_[i].start = mmap(
        nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
      if (buffers_[i].start == MAP_FAILED) {
        buffers_[i].start = nullptr;
        throw std::runtime_error("mmap failed");
      }
      check(ioctl(fd_, VIDIOC_QBUF, &buf), "VIDIOC_QBUF");
    }
  }

  void start_streaming()
  {
    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    check(ioctl(fd_, VIDIOC_STREAMON, &type), "VIDIOC_STREAMON");
    streaming_ = true;
  }

  void stop_streaming()
  {
    if (!streaming_) {
      return;
    }
    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;
  }

  // ---------------------------------------------------------------- capture
  void capture_loop()
  {
    try {
      while (running_ && rclcpp::ok()) {
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        const int ready = poll(&pfd, 1, 200);
        if (ready < 0) {
          if (errno == EINTR) {
            continue;
          }
          check(-1, "poll");
        }
        if (ready == 0) {
          continue;  // nothing yet; also the exit path within 200 ms
        }

        // Drain: keep the newest frame, hand the older ones straight back. This
        // is what keeps a stalled second from turning into a permanent offset.
        v4l2_buffer keep{};
        bool have = false;
        int discarded = 0;
        while (true) {
          v4l2_buffer buf{};
          buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          buf.memory = V4L2_MEMORY_MMAP;
          if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) {
              break;
            }
            check(-1, "VIDIOC_DQBUF");
          }
          if (have) {
            check(ioctl(fd_, VIDIOC_QBUF, &keep), "VIDIOC_QBUF (discard)");
            ++discarded;
          }
          keep = buf;
          have = true;
        }
        if (!have) {
          continue;
        }

        publish_frame(keep);
        check(ioctl(fd_, VIDIOC_QBUF, &keep), "VIDIOC_QBUF");

        ++frames_;
        discarded_ += discarded;
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "capture failed: %s", e.what());
      rclcpp::shutdown();
    }
  }

  void publish_frame(const v4l2_buffer & buf)
  {
    const std::size_t available = buffers_[buf.index].length;
    const std::size_t bytes = std::min<std::size_t>(buf.bytesused, available);
    if (msg_->data.size() != bytes) {
      msg_->data.resize(bytes);
    }
    std::memcpy(msg_->data.data(), buffers_[buf.index].start, bytes);

    // Timestamp: the driver's own capture time, converted to wall clock.
    // Computing the shift every frame (two syscalls) is cheaper than worrying
    // about the MONOTONIC/REALTIME difference drifting over a long session.
    const auto stamp_type = buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;
    const std::int64_t raw_us = static_cast<std::int64_t>(buf.timestamp.tv_sec) * 1000000 +
      buf.timestamp.tv_usec;
    std::int64_t st_us = raw_us;
    const std::int64_t now_mono_us = mono_us();
    if (stamp_type == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
      st_us += wall_us() - now_mono_us;
    }
    msg_->header.stamp.sec = static_cast<std::int32_t>(st_us / 1000000);
    msg_->header.stamp.nanosec = static_cast<std::uint32_t>((st_us % 1000000) * 1000);
    msg_->header.frame_id = frame_id_;
    msg_->height = static_cast<std::uint32_t>(height_);
    msg_->width = static_cast<std::uint32_t>(width_);
    msg_->encoding = encoding_;
    msg_->is_bigendian = 0;
    msg_->step = static_cast<std::uint32_t>(step_);

    publisher_->publish(*msg_);

    // Health line: how old the frame is at the moment it leaves the node. If
    // this creeps up, the publish path is not keeping up with the camera.
    if (stamp_type == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
      report((mono_us() - raw_us) / 1000.0);
    }
  }

  void report(double age_ms)
  {
    // The first frames after STREAMON are genuinely stale: the device has been
    // filling its own pipeline since the format was set, so frame #1 can carry
    // a timestamp two seconds old. That is a startup artifact, not the steady
    // state the health line is supposed to describe.
    if (frames_ < 5) {
      return;
    }
    if (age_min_ < 0 || age_ms < age_min_) {
      age_min_ = age_ms;
    }
    age_max_ = std::max(age_max_, age_ms);

    const auto now = std::chrono::steady_clock::now();
    if (last_report_.time_since_epoch().count() == 0) {
      last_report_ = now;
      frames_at_report_ = frames_;
      discarded_at_report_ = discarded_;
      return;
    }
    const double dt = std::chrono::duration<double>(now - last_report_).count();
    if (dt < stats_period_) {
      return;
    }
    const auto fps = static_cast<double>(frames_ - frames_at_report_) / dt;
    RCLCPP_INFO(
      get_logger(), "%.1f fps, frame age at publish %.0f-%.0f ms, %lu stale frame(s) discarded",
      fps, age_min_, age_max_,
      static_cast<unsigned long>(discarded_ - discarded_at_report_));
    last_report_ = now;
    frames_at_report_ = frames_;
    discarded_at_report_ = discarded_;
    age_min_ = -1;
    age_max_ = 0;
  }

  // ---------------------------------------------------------------- members
  std::string device_;
  std::string pixel_format_;
  std::string frame_id_;
  std::string encoding_;
  int width_{0};
  int height_{0};
  int step_{0};
  int buffer_count_{4};
  double framerate_{30.0};
  double stats_period_{10.0};

  int fd_{-1};
  bool streaming_{false};
  std::vector<Buffer> buffers_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::unique_ptr<sensor_msgs::msg::Image> msg_;

  std::atomic<bool> running_{false};
  std::thread thread_;

  std::uint64_t frames_{0};
  std::uint64_t discarded_{0};
  std::uint64_t frames_at_report_{0};
  std::uint64_t discarded_at_report_{0};
  std::chrono::steady_clock::time_point last_report_{};
  double age_min_{-1.0};
  double age_max_{0.0};
};

}  // namespace hm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<hm::V4l2Capture>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("v4l2_capture"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
