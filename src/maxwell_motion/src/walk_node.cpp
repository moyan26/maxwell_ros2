#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "maxwell_msgs/msg/behavior_cmd.hpp"
#include "maxwell_msgs/msg/joint_command.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class WalkNode : public rclcpp::Node
{
public:
  WalkNode() : Node("walk_node")
  {
    sub_cmd_ = this->create_subscription<maxwell_msgs::msg::BehaviorCmd>(
      "/behavior/cmd", 10, std::bind(&WalkNode::cmd_cb, this, _1));

    pub_joint_ = this->create_publisher<maxwell_msgs::msg::JointCommand>(
      "/joint_command", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&WalkNode::tick, this)); // 20Hz

    // 先假设 20 个关节（你们后面接真实机器人再改成真实关节数）
    joint_count_ = this->declare_parameter<int>("joint_count", 20);

    RCLCPP_INFO(this->get_logger(),
      "walk_node started. sub=/behavior/cmd pub=/joint_command (20Hz). joint_count=%d",
      joint_count_);
  }

private:
  void cmd_cb(const maxwell_msgs::msg::BehaviorCmd::SharedPtr msg)
  {
    last_cmd_ = *msg;
    have_cmd_ = true;

    // 记录当前动作名
    current_action_ = msg->action_name;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "recv /behavior/cmd: action=%s enable=%d type=%u x=%.2f y=%.2f dir=%.2f",
      msg->action_name.c_str(), msg->enable, msg->type, msg->x, msg->y, msg->dir);
  }

  void tick()
  {
    if (!have_cmd_) return;
    if (!last_cmd_.enable) return;

    // 时间参数，用于生成可变化的关节波形，方便你肉眼看出不同动作
    const double t = this->now().seconds();

    maxwell_msgs::msg::JointCommand out;
    out.positions.resize(joint_count_, 0.0f);
    out.velocities.resize(joint_count_, 0.0f);

    if (current_action_ == "get_up") {
      // 起身：给一个“大幅度、低频”的波形
      fill_wave(out, t, 0.6, 1.0);
    } else if (current_action_ == "goto_ball" || current_action_ == "get_ball") {
      // 去球：中幅度、稍高频
      fill_wave(out, t, 0.3, 2.5);
    } else if (current_action_ == "search_ball") {
      // 找球：小幅度、摆头/原地动作（用小波形表示）
      fill_wave(out, t, 0.12, 1.5);
    } else {
      // 其它：静止
      // positions/velocities 默认 0
    }

    pub_joint_->publish(out);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "pub /joint_command for action=%s (pos[0]=%.2f vel[0]=%.2f)",
      current_action_.c_str(),
      out.positions.empty() ? 0.0f : out.positions[0],
      out.velocities.empty() ? 0.0f : out.velocities[0]);
  }

  void fill_wave(maxwell_msgs::msg::JointCommand &out, double t, double amplitude, double freq_hz)
  {
    // 给每个关节一个相位差，形成“像在动”的效果
    for (int i = 0; i < joint_count_; ++i) {
      const double phase = 0.25 * i;
      const double w = 2.0 * M_PI * freq_hz;
      const double pos = amplitude * std::sin(w * t + phase);
      const double vel = amplitude * w * std::cos(w * t + phase);
      out.positions[i] = static_cast<float>(pos);
      out.velocities[i] = static_cast<float>(vel);
    }
  }

private:
  rclcpp::Subscription<maxwell_msgs::msg::BehaviorCmd>::SharedPtr sub_cmd_;
  rclcpp::Publisher<maxwell_msgs::msg::JointCommand>::SharedPtr pub_joint_;
  rclcpp::TimerBase::SharedPtr timer_;

  maxwell_msgs::msg::BehaviorCmd last_cmd_;
  bool have_cmd_{false};

  std::string current_action_{"idle"};
  int joint_count_{20};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkNode>());
  rclcpp::shutdown();
  return 0;
}
