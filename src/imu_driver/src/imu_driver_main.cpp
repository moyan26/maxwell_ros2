#include <rclcpp/rclcpp.hpp>
#include "imu_driver/imu_node.hpp"

int main(int argc, char *argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<ImuNode>();

    // 启动 IMU 读取
    if (!node->start()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start IMU node");
        rclcpp::shutdown();
        return 1;
    }

    // 运行节点（阻塞直到节点被关闭）
    rclcpp::spin(node);

    // 停止 IMU
    node->stop();

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
