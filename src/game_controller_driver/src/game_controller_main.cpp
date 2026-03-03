#include <rclcpp/rclcpp.hpp>
#include "game_controller_driver/game_controller_node.hpp"

using namespace game_controller_driver;

int main(int argc, char *argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<GameControllerNode>();

    // 启动接收
    if (!node->start()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start game controller node");
        rclcpp::shutdown();
        return 1;
    }

    // 运行节点
    rclcpp::spin(node);

    // 停止接收
    node->stop();

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
