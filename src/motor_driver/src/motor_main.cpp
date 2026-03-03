#include <rclcpp/rclcpp.hpp>
#include "motor_driver/motor_node.hpp"

int main(int argc, char *argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<MotorNode>();

    // 启动电机控制
    if (!node->start()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start motor node");
        rclcpp::shutdown();
        return 1;
    }

    // 运行节点
    rclcpp::spin(node);

    // 停止电机
    node->stop();

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
