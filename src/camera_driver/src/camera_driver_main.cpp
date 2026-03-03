#include <rclcpp/rclcpp.hpp>
#include "camera_driver/camera_node.hpp"

int main(int argc, char *argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<CameraNode>();

    // 启动相机采集
    if (!node->start()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start camera node");
        rclcpp::shutdown();
        return 1;
    }

    // 运行节点（使用 MultiThreadedExecutor 因为节点内部有采集线程）
    rclcpp::spin(node);

    // 停止相机
    node->stop();

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
