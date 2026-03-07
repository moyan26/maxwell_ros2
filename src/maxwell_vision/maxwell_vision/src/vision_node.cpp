#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "maxwell_msgs/msg/ball.hpp"
#include "vision.hpp"

#ifndef MAXWELL_VISION_RUNTIME
// If the build system doesn't define it for this target, assume runtime build.
#define MAXWELL_VISION_RUNTIME 1
#endif

// ROS2 vision_node:
// - subscribe /camera/image_raw
// - feed frames to Vision::set_bgr_frame()
// - timer calls Vision::process_once()
// - publish /vision/ball using maxwell_msgs/Ball
class VisionNode final : public rclcpp::Node
{
public:
  VisionNode() : rclcpp::Node("vision_node")
  {
    publish_hz_ = this->declare_parameter<double>("publish_hz", 30.0);

    // Start vision core once
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
  void on_image(const sensor_msgs::msg::Image::SharedPtr& msg)
  {
    // Convert to BGR8 cv::Mat
    cv::Mat bgr;
    try
    {
      // If source encoding already bgr8, this is a cheap view. Otherwise converts.
      bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "cv_bridge conversion failed: %s", e.what());
      return;
    }

    // Feed frame into core (thread-safe)
    VISION->set_bgr_frame(bgr);

    // Keep latest header for publishing
    last_header_stamp_ = msg->header.stamp;
    last_header_frame_ = msg->header.frame_id;
  }

  void on_timer()
  {
    // Run one inference step
    VISION->process_once();

    // Publish best ball (if any)
    object_det det;
    maxwell_msgs::msg::Ball out;

    out.header.stamp = (last_header_stamp_.sec == 0 && last_header_stamp_.nanosec == 0)
                         ? this->now()
                         : last_header_stamp_;
    out.header.frame_id = last_header_frame_.empty() ? "camera" : last_header_frame_;

    if (VISION->get_best_ball(det))
    {
      out.visible = true;

      // Pixel: use bottom-center of bbox (consistent with legacy debug drawing)
      const int px = det.x + det.w / 2;
      const int py = det.y + det.h;

      out.pixel_x = px;
      out.pixel_y = py;

      // No odometry/head pose in ROS2 vision_node MVP, so positions left as 0.
      out.global_x = 0.0f;
      out.global_y = 0.0f;
      out.self_x = 0.0f;
      out.self_y = 0.0f;

      // Approx alpha/beta from image center (fallback; exact cx/cy is internal)
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
      out.global_x = 0.0f;
      out.global_y = 0.0f;
      out.self_x = 0.0f;
      out.self_y = 0.0f;
      out.alpha = 0.0f;
      out.beta  = 0.0f;
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

