#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "maxwell_msgs/msg/ball.hpp"
#include "maxwell_msgs/msg/odom.hpp"
#include "maxwell_msgs/msg/world_state.hpp"

#include "worldmodel.hpp"  // from player/core include dirs

class WorldModelNode final : public rclcpp::Node
{
public:
  WorldModelNode()
  : rclcpp::Node("worldmodel_node")
  {
    // Params
    publish_hz_ = this->declare_parameter<double>("publish_hz", 30.0);

    // Subscriptions
    sub_ball_ = this->create_subscription<maxwell_msgs::msg::Ball>(
      "/vision/ball", rclcpp::SensorDataQoS(),
      [this](const maxwell_msgs::msg::Ball::SharedPtr msg) { on_ball(msg); });

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });

    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { (void)msg; /* reserved */ });

    sub_self_pose_ = this->create_subscription<maxwell_msgs::msg::Odom>(
      "/walk/self_pose", rclcpp::QoS(10),
      [this](const maxwell_msgs::msg::Odom::SharedPtr msg) { on_self_pose(msg); });

    // Publisher
    pub_world_ = this->create_publisher<maxwell_msgs::msg::WorldState>("/world/state", rclcpp::QoS(10));

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, publish_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { on_timer(); });

    RCLCPP_INFO(this->get_logger(), "worldmodel_node started (publish_hz=%.2f).", publish_hz_);
  }

private:
  void on_ball(const maxwell_msgs::msg::Ball::SharedPtr& msg)
  {
    const Eigen::Vector2d g(msg->global_x, msg->global_y);
    const Eigen::Vector2d s(msg->self_x, msg->self_y);
    const Eigen::Vector2i p(msg->pixel_x, msg->pixel_y);
    wm_.set_ball_pos(g, s, p, msg->alpha, msg->beta, msg->visible);
  }

  void on_self_pose(const maxwell_msgs::msg::Odom::SharedPtr& msg)
  {
    const Eigen::Vector2d pos(msg->x, msg->y);
    wm_.set_my_pose(pos, msg->yaw, msg->sure);
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr& msg)
  {
    // Quaternion -> RPY
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    ImuData data;
    data.roll  = roll;
    data.pitch = pitch;
    data.yaw   = yaw;

    data.ax = msg->linear_acceleration.x;
    data.ay = msg->linear_acceleration.y;
    data.az = msg->linear_acceleration.z;

    data.wx = msg->angular_velocity.x;
    data.wy = msg->angular_velocity.y;
    data.wz = msg->angular_velocity.z;

    const auto& t = msg->header.stamp;
    data.timestamp = static_cast<int64_t>(t.sec) * 1000LL + static_cast<int64_t>(t.nanosec) / 1000000LL;

    // Fall direction: not provided by sensor_msgs/Imu; keep as NONE (you can add detection later)
    wm_.update_imu(data, FALL_NONE);
  }

  void on_timer()
  {
    maxwell_msgs::msg::WorldState out;
    out.header.stamp = this->now();
    out.header.frame_id = "map";

    const auto self = wm_.self();
    out.self_pose.x = self.global.x();
    out.self_pose.y = self.global.y();
    out.self_pose.theta = self.dir;
    out.self_pose_sure = self.sure;

    const auto ball = wm_.ball();
    out.ball.header = out.header;
    out.ball.visible = ball.can_see;
    out.ball.global_x = static_cast<float>(ball.global.x());
    out.ball.global_y = static_cast<float>(ball.global.y());
    out.ball.self_x   = static_cast<float>(ball.self.x());
    out.ball.self_y   = static_cast<float>(ball.self.y());
    out.ball.pixel_x  = ball.pixel.x();
    out.ball.pixel_y  = ball.pixel.y();
    out.ball.alpha    = ball.alpha;
    out.ball.beta     = ball.beta;

    out.fall_direction = static_cast<int8_t>(wm_.fall_data());
    out.support_foot   = static_cast<uint8_t>(wm_.support_foot());

    pub_world_->publish(out);
  }

private:
  WorldModel wm_;

  double publish_hz_{30.0};

  rclcpp::Subscription<maxwell_msgs::msg::Ball>::SharedPtr sub_ball_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<maxwell_msgs::msg::Odom>::SharedPtr sub_self_pose_;

  rclcpp::Publisher<maxwell_msgs::msg::WorldState>::SharedPtr pub_world_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldModelNode>());
  rclcpp::shutdown();
  return 0;
}
