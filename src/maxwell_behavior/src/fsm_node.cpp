#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "maxwell_msgs/msg/world_state.hpp"
#include "maxwell_msgs/msg/behavior_cmd.hpp"

using namespace std::chrono_literals;

class FSMNode : public rclcpp::Node
{
public:
  FSMNode() : Node("fsm_node")
  {
    // 初始状态（你可以改成 default_action/search_ball 都行）
    current_state_ = "default_action";

    // 订阅 /world/state
    state_sub_ = this->create_subscription<maxwell_msgs::msg::WorldState>(
      "/world/state",
      rclcpp::QoS(10),
      std::bind(&FSMNode::state_callback, this, std::placeholders::_1));

    // 发布 /behavior/cmd
    cmd_pub_ = this->create_publisher<maxwell_msgs::msg::BehaviorCmd>(
      "/behavior/cmd",
      rclcpp::QoS(10));

    // 定时器：100ms tick 一次
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&FSMNode::tick, this));

    RCLCPP_INFO(this->get_logger(), "fsm_node started. waiting for /world/state ...");
  }

private:
  void state_callback(const maxwell_msgs::msg::WorldState::SharedPtr msg)
  {
    last_state_ = *msg;
    have_state_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "world_state recv: fall=%d, ball_seen=%d, self(%.2f, %.2f, %.1f)",
      msg->fall_direction,
      msg->ball_seen,
      msg->self_x, msg->self_y, msg->self_dir_deg);

    // 最小决策逻辑：摔倒优先，其次看到球去球，否则找球
    if (msg->fall_direction != 0) {
      current_state_ = "get_up";
    } else if (msg->ball_seen) {
      current_state_ = "goto_ball";
    } else {
      current_state_ = "search_ball";
    }

    RCLCPP_INFO(this->get_logger(), "FSM state updated -> %s", current_state_.c_str());
  }

  void tick()
  {
    // 没收到 world_state：先保持默认行为（避免乱跑）
    if (!have_state_) {
      publish_behavior_command("default_action");
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "no /world/state received yet -> publish default_action");
      return;
    }

    // 收到 world_state 后，根据 current_state 发布行为
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "FSM tick: current_state=%s", current_state_.c_str());

    if (current_state_ == "get_up") {
      publish_behavior_command("get_up");
    } else if (current_state_ == "goto_ball") {
      publish_behavior_command("goto_ball");
    } else if (current_state_ == "search_ball") {
      publish_behavior_command("search_ball");
    } else {
      publish_behavior_command("default_action");
    }
  }

  void publish_behavior_command(const std::string & action_name)
  {
    maxwell_msgs::msg::BehaviorCmd msg;

    // 你当前的 BehaviorCmd 字段：
    // uint8 type
    // float32 x
    // float32 y
    // float32 dir
    // bool enable
    // string action_name
    msg.type = 1;
    msg.x = 0.0f;
    msg.y = 0.0f;
    msg.dir = 0.0f;
    msg.enable = true;
    msg.action_name = action_name;

    cmd_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "Published /behavior/cmd action_name=%s", action_name.c_str());
  }

private:
  // ROS2 entities
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<maxwell_msgs::msg::WorldState>::SharedPtr state_sub_;
  rclcpp::Publisher<maxwell_msgs::msg::BehaviorCmd>::SharedPtr cmd_pub_;

  // state cache
  maxwell_msgs::msg::WorldState last_state_;
  bool have_state_{false};
  std::string current_state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FSMNode>());
  rclcpp::shutdown();
  return 0;
}