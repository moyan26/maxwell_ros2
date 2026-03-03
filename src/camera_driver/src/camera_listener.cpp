#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraListener : public rclcpp::Node
{
public:
    CameraListener() : Node("camera_listener")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                frame_count_++;
                
                // 每 30 帧输出一次信息
                if (frame_count_ % 30 == 0) {
                    RCLCPP_INFO(this->get_logger(), 
                        "Received frame %d: %dx%d, encoding=%s",
                        frame_count_, msg->width, msg->height, msg->encoding.c_str());
                }
                
                // 可选：显示图像（需要 GUI）
                if (show_image_) {
                    try {
                        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
                        cv::imshow("Camera", image);
                        cv::waitKey(1);
                    } catch (cv_bridge::Exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                    }
                }
            });
        
        this->declare_parameter<bool>("show_image", false);
        show_image_ = this->get_parameter("show_image").as_bool();
        
        RCLCPP_INFO(this->get_logger(), "Camera Listener started");
        if (show_image_) {
            RCLCPP_INFO(this->get_logger(), "Image display enabled (requires GUI)");
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    int frame_count_ = 0;
    bool show_image_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraListener>());
    rclcpp::shutdown();
    return 0;
}
