#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "maxwell_msgs/msg/ball.hpp"
#include "vision.hpp"

// If the build system doesn't define it for this target, assume runtime build.
#ifndef MAXWELL_VISION_RUNTIME
#define MAXWELL_VISION_RUNTIME 1
#endif

class VisionNode final : public rclcpp::Node
{
public:
  VisionNode() : rclcpp::Node("vision_node")
  {
    publish_hz_ = this->declare_parameter<double>("publish_hz", 30.0);

#if MAXWELL_VISION_RUNTIME
    if (!VISION->start())
    {
      RCLCPP_FATAL(this->get_logger(),
                   "VISION->start() failed. Check net_cfg_file/net_weights_file and CUDA/Darknet setup.");
      throw std::runtime_error("VISION start failed");
    }
#else
    if (!VISION->start())
    {
      RCLCPP_WARN(this->get_logger(),
                  "maxwell_vision built with MAXWELL_VISION_RUNTIME=OFF; node will run without inference.");
    }
#endif

    sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_image(msg); });

    pub_ball_ = this->create_publisher<maxwell_msgs::msg::Ball>("/vision/ball", rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, publish_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { on_timer(); });

    RCLCPP_INFO(this->get_logger(), "vision_node started (publish_hz=%.2f).", publish_hz_);
  }

  ~VisionNode() override
  {
    VISION->stop();
  }

private:
  static bool to_bgr8(const sensor_msgs::msg::Image& msg, cv::Mat& out_bgr)
  {
    using sensor_msgs::image_encodings::BGR8;
    using sensor_msgs::image_encodings::RGB8;
    using sensor_msgs::image_encodings::MONO8;
    using sensor_msgs::image_encodings::BGRA8;
    using sensor_msgs::image_encodings::RGBA8;

    if (msg.data.empty() || msg.height == 0 || msg.width == 0) return false;

    // Wrap raw buffer as cv::Mat with stride = step
    const int h = static_cast<int>(msg.height);
    const int w = static_cast<int>(msg.width);

    const std::string& enc = msg.encoding;

    if (enc == BGR8)
    {
      cv::Mat bgr(h, w, CV_8UC3, const_cast<unsigned char*>(msg.data.data()), msg.step);
      out_bgr = bgr;
      return true;
    }
    if (enc == RGB8)
    {
      cv::Mat rgb(h, w, CV_8UC3, const_cast<unsigned char*>(msg.data.data()), msg.step);
      cv::cvtColor(rgb, out_bgr, cv::COLOR_RGB2BGR);
      return true;
    }
    if (enc == MONO8)
    {
      cv::Mat gray(h, w, CV_8UC1, const_cast<unsigned char*>(msg.data.data()), msg.step);
      cv::cvtColor(gray, out_bgr, cv::COLOR_GRAY2BGR);
      return true;
    }
    if (enc == BGRA8)
    {
      cv::Mat bgra(h, w, CV_8UC4, const_cast<unsigned char*>(msg.data.data()), msg.step);
      cv::cvtColor(bgra, out_bgr, cv::COLOR_BGRA2BGR);
      return true;
    }
    if (enc == RGBA8)
    {
      cv::Mat rgba(h, w, CV_8UC4, const_cast<unsigned char*>(msg.data.data()), msg.step);
      cv::cvtColor(rgba, out_bgr, cv::COLOR_RGBA2BGR);
      return true;
    }

    // Common YUYV/YUV422 encodings (names differ across drivers)
    if (enc == "yuyv" || enc == "yuv422" || enc == "yuv422_yuy2" || enc == "yuy2")
    {
      // YUY2/YUYV is 2 bytes per pixel -> CV_8UC2
      cv::Mat yuy2(h, w, CV_8UC2, const_cast<unsigned char*>(msg.data.data()), msg.step);
      cv::cvtColor(yuy2, out_bgr, cv::COLOR_YUV2BGR_YUY2);
      return true;
    }

    return false;
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr& msg)
  {
    cv::Mat bgr;
    if (!to_bgr8(*msg, bgr))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Unsupported image encoding: '%s' (need bgr8/rgb8/mono8/bgra8/rgba8/yuyv).",
                           msg->encoding.c_str());
      return;
    }

    // Feed frame into core (Vision will copy data into its own buffer)
    VISION->set_bgr_frame(bgr);

    last_header_stamp_ = msg->header.stamp;
    last_header_frame_ = msg->header.frame_id;
  }

  void on_timer()
  {
    VISION->process_once();

    object_det det;
    maxwell_msgs::msg::Ball out;

    if (last_header_stamp_.sec == 0 && last_header_stamp_.nanosec == 0)
    {
      const rclcpp::Time now_t = this->get_clock()->now();
      builtin_interfaces::msg::Time now_msg;
      now_msg.sec = static_cast<int32_t>(now_t.seconds());  // 秒
      now_msg.nanosec = static_cast<uint32_t>(now_t.nanoseconds() % 1000000000LL);  // 纳秒
      out.header.stamp = now_msg; 
    }
    else
    {
      out.header.stamp = last_header_stamp_;
    }
    out.header.frame_id = last_header_frame_.empty() ? "camera" : last_header_frame_;

    if (VISION->get_best_ball(det))
    {
      out.visible = true;
      const int px = det.x + det.w / 2;
      const int py = det.y + det.h;

      out.pixel_x = px;
      out.pixel_y = py;

      out.global_x = 0.0f;
      out.global_y = 0.0f;
      out.self_x = 0.0f;
      out.self_y = 0.0f;

      const float w = static_cast<float>(std::max(1, VISION->w()));
      const float h = static_cast<float>(std::max(1, VISION->h()));
      out.alpha = (static_cast<float>(px) - 0.5f * w) / w;
      out.beta  = (static_cast<float>(py) - 0.5f * h) / h;
    }
    else
    {
      out.visible = false;
      out.pixel_x = -1;
      out.pixel_y = -1;
      out.global_x = out.global_y = 0.0f;
      out.self_x = out.self_y = 0.0f;
      out.alpha = out.beta = 0.0f;
    }

    pub_ball_->publish(out);
  }

private:
  double publish_hz_{30.0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Publisher<maxwell_msgs::msg::Ball>::SharedPtr pub_ball_;
  rclcpp::TimerBase::SharedPtr timer_;

  builtin_interfaces::msg::Time last_header_stamp_{};
  std::string last_header_frame_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
