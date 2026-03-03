/**
 * @brief IMU 数据订阅示例
 * 
 * 演示如何订阅 /imu/data 和 /imu/fall_direction 话题
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber() : Node("imu_subscriber")
    {
        // 订阅 IMU 数据
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 
            rclcpp::SensorDataQoS(),
            std::bind(&ImuSubscriber::imuCallback, this, std::placeholders::_1)
        );

        // 订阅摔倒检测
        fall_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "imu/fall_direction",
            10,
            std::bind(&ImuSubscriber::fallCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "IMU Subscriber started");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 将四元数转换为欧拉角
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 转换为角度
        roll = roll * 180.0 / M_PI;
        pitch = pitch * 180.0 / M_PI;
        yaw = yaw * 180.0 / M_PI;

        RCLCPP_INFO(this->get_logger(), 
            "IMU - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", 
            roll, pitch, yaw);
        
        RCLCPP_INFO(this->get_logger(),
            "Angular Vel - x: %.3f, y: %.3f, z: %.3f rad/s",
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        
        RCLCPP_INFO(this->get_logger(),
            "Accel - x: %.3f, y: %.3f, z: %.3f m/s²",
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);
    }

    void fallCallback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        std::string status;
        switch (msg->data) {
            case 0:  status = "STANDING"; break;
            case 1:  status = "FALL_FORWARD"; break;
            case -1: status = "FALL_BACKWARD"; break;
            case 2:  status = "FALL_LEFT"; break;
            case -2: status = "FALL_RIGHT"; break;
            default: status = "UNKNOWN"; break;
        }
        
        if (msg->data != 0) {
            RCLCPP_WARN(this->get_logger(), "FALL DETECTED: %s (%d)", status.c_str(), msg->data);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr fall_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuSubscriber>());
    rclcpp::shutdown();
    return 0;
}
