#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iomanip>

class ImuListener : public rclcpp::Node
{
public:
    ImuListener() : Node("imu_listener")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                static int count = 0;
                count++;
                
                // 每 10 帧输出一次（约 100ms）
                if (count % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), 
                        "=== IMU Data [%d] ===", count);
                    RCLCPP_INFO(this->get_logger(), 
                        "Orientation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
                        msg->orientation.x, msg->orientation.y,
                        msg->orientation.z, msg->orientation.w);
                    RCLCPP_INFO(this->get_logger(), 
                        "Angular Velocity: [x=%.3f, y=%.3f, z=%.3f] rad/s",
                        msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z);
                    RCLCPP_INFO(this->get_logger(), 
                        "Linear Accel: [x=%.3f, y=%.3f, z=%.3f] m/s^2",
                        msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z);
                }
            });
        
        RCLCPP_INFO(this->get_logger(), "IMU Listener started, listening on /imu/data");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuListener>());
    rclcpp::shutdown();
    return 0;
}
