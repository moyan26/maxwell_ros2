#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class MotorListener : public rclcpp::Node
{
public:
    MotorListener() : Node("motor_listener")
    {
        // 订阅关节状态
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                static int count = 0;
                count++;
                
                // 每 100 帧输出一次摘要
                if (count % 100 == 0) {
                    RCLCPP_INFO(this->get_logger(), 
                        "=== Joint States [%d] ===", count);
                    RCLCPP_INFO(this->get_logger(), "Joint count: %zu", msg->name.size());
                    
                    // 输出前 5 个关节的位置
                    for (size_t i = 0; i < std::min(size_t(5), msg->name.size()); ++i) {
                        double deg = msg->position[i] * 180.0 / M_PI;
                        RCLCPP_INFO(this->get_logger(), 
                            "  %s: %.2f deg", msg->name[i].c_str(), deg);
                    }
                    if (msg->name.size() > 5) {
                        RCLCPP_INFO(this->get_logger(), "  ... and %zu more", 
                                   msg->name.size() - 5);
                    }
                }
            });

        RCLCPP_INFO(this->get_logger(), "Motor Listener started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /joint_states");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorListener>());
    rclcpp::shutdown();
    return 0;
}
