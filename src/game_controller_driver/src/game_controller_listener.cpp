#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>

class GameControllerListener : public rclcpp::Node
{
public:
    GameControllerListener() : Node("game_controller_listener")
    {
        // 订阅比赛状态
        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "game_controller/state", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "State: %s", msg->data.c_str());
            });

        // 订阅比分
        score_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "game_controller/score", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Our Score: %d", msg->data);
            });

        // 订阅剩余时间
        time_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "game_controller/time_remaining", 10,
            [this](const std_msgs::msg::Int16::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Time Remaining: %d seconds", msg->data);
            });

        // 订阅次要状态
        sec_state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "game_controller/secondary_state", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Secondary State: %s", msg->data.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "Game Controller Listener started");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr score_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr time_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sec_state_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameControllerListener>());
    rclcpp::shutdown();
    return 0;
}
