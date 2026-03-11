#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "maxwell_msgs/msg/world_state.hpp"
#include "maxwell_vision/msg/ball.hpp"          // ✅ 正确 include（单层）
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class WorldNode : public rclcpp::Node
{
public:
  WorldNode() : Node("world_node")
  {
    // 发布 /world/state
    pub_state_ = this->create_publisher<maxwell_msgs::msg::WorldState>("/world/state", 10);

    // 订阅 /vision/ball
    sub_ball_ = this->create_subscription<maxwell_vision::msg::Ball>(
      "/vision/ball", 10,
      std::bind(&WorldNode::ball_callback, this, _1));

    // 订阅 /imu/data
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&WorldNode::cb_imu, this, _1));

    // 订阅 /walk/self_pose
    sub_self_pose_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/walk/self_pose", 10,
      std::bind(&WorldNode::cb_self_pose, this, _1));

    // 定时发布
    timer_ = this->create_wall_timer(200ms, std::bind(&WorldNode::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "world_node started. pub=/world/state sub=/vision/ball,/imu/data,/walk/self_pose");
  }

private:
  // ---------------- IMU 回调：更新 fall_direction ----------------
  void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
    have_imu_ = true;

    // 简单判定（先可用，后续你再换成真实 fall 逻辑）
    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;

    // 你也可以把阈值改小一点更容易测试
    const double TH = 5.0;

    if (ax > TH) fall_direction_ = 1;       // forward
    else if (ax < -TH) fall_direction_ = -1; // backward
    else if (ay > TH) fall_direction_ = 2;   // left
    else if (ay < -TH) fall_direction_ = -2; // right
    else fall_direction_ = 0;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "IMU received: ax=%.2f ay=%.2f fall_direction=%d",
                         ax, ay, fall_direction_);
  }

  // ---------------- 自身位姿回调 ----------------
  void cb_self_pose(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    last_self_pose_ = *msg;
    have_self_pose_ = true;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Self pose received: x=%.2f y=%.2f theta=%.2f",
                         msg->x, msg->y, msg->theta);
  }

  // ---------------- 视觉球回调：更新 ball_seen/ball坐标 ----------------
  void ball_callback(const maxwell_vision::msg::Ball::SharedPtr msg)
  {
    last_ball_ = *msg;
    have_ball_ = true;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Ball received: seen=%d self(%.2f,%.2f) global(%.2f,%.2f)",
                         msg->seen, msg->self_x, msg->self_y, msg->global_x, msg->global_y);
  }

  // ---------------- 定时发布 /world/state ----------------
  void tick()
  {
    maxwell_msgs::msg::WorldState ws;

    // localization_time 先默认 true（你后续接定位模块再改）
    ws.localization_time = true;

    // self
    if (have_self_pose_) {
      ws.self_x = static_cast<float>(last_self_pose_.x);
      ws.self_y = static_cast<float>(last_self_pose_.y);
      ws.self_dir_deg = static_cast<float>(last_self_pose_.theta * 180.0 / 3.1415926);
    } else {
      ws.self_x = 0.0f;
      ws.self_y = 0.0f;
      ws.self_dir_deg = 0.0f;
    }

    // fall_direction
    ws.fall_direction = have_imu_ ? fall_direction_ : 0;

    // ball
    if (have_ball_) {
      ws.ball_seen = last_ball_.seen;
      ws.ball_self_x = last_ball_.self_x;
      ws.ball_self_y = last_ball_.self_y;
      ws.ball_global_x = last_ball_.global_x;
      ws.ball_global_y = last_ball_.global_y;
    } else {
      ws.ball_seen = false;
      ws.ball_self_x = 0.0f;
      ws.ball_self_y = 0.0f;
      ws.ball_global_x = 0.0f;
      ws.ball_global_y = 0.0f;
    }

    // 球门柱先默认 0（你后面接 worldmodel/vision 再改）
    ws.opp_post_left_x = 0.0f;
    ws.opp_post_left_y = 0.0f;
    ws.opp_post_right_x = 0.0f;
    ws.opp_post_right_y = 0.0f;

    pub_state_->publish(ws);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Publish /world/state: fall=%d ball_seen=%d self(%.2f,%.2f,%.1f)",
                         ws.fall_direction, ws.ball_seen, ws.self_x, ws.self_y, ws.self_dir_deg);
  }

private:
  // pub/sub
  rclcpp::Publisher<maxwell_msgs::msg::WorldState>::SharedPtr pub_state_;
  rclcpp::Subscription<maxwell_vision::msg::Ball>::SharedPtr sub_ball_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_self_pose_;
  rclcpp::TimerBase::SharedPtr timer_;

  // latest msgs
  sensor_msgs::msg::Imu last_imu_;
  geometry_msgs::msg::Pose2D last_self_pose_;
  maxwell_vision::msg::Ball last_ball_;

  // flags
  bool have_imu_{false};
  bool have_self_pose_{false};
  bool have_ball_{false};

  // state
  int32_t fall_direction_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldNode>());
  rclcpp::shutdown();
  return 0;
}