#include "camera_driver/camera_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>

using namespace std::chrono_literals;

CameraNode::CameraNode(const rclcpp::NodeOptions &options)
    : Node("camera_node", options)
    , start_time_(this->now())
{
    // ============== 声明并获取参数 ==============
    this->declare_parameter<bool>("mock_mode", true);  // 默认模拟模式
    this->declare_parameter<std::string>("device_name", "/dev/video0");
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<int>("fps", 30);
    this->declare_parameter<std::string>("mock_pattern", "color_bar");  // color_bar, noise, gradient
    this->declare_parameter<std::string>("frame_id", "camera_link");

    mock_mode_ = this->get_parameter("mock_mode").as_bool();
    device_name_ = this->get_parameter("device_name").as_string();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    fps_ = this->get_parameter("fps").as_int();
    mock_pattern_ = this->get_parameter("mock_pattern").as_string();
    std::string frame_id = this->get_parameter("frame_id").as_string();

    // ============== 创建发布者 ==============
    // 注意：ImageTransport 需要在 start() 中初始化，因为构造函数中不能使用 shared_from_this()

    RCLCPP_INFO(this->get_logger(), "Camera Node initialized");
    RCLCPP_INFO(this->get_logger(), "Mock mode: %s", mock_mode_ ? "enabled" : "disabled");
    if (!mock_mode_) {
        RCLCPP_INFO(this->get_logger(), "Device: %s", device_name_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d, FPS: %d", 
                image_width_, image_height_, fps_);
    RCLCPP_INFO(this->get_logger(), "Mock pattern: %s", mock_pattern_.c_str());
}

CameraNode::~CameraNode()
{
    stop();
}

bool CameraNode::start()
{
    if (is_running_) {
        return true;
    }

    // 创建发布者（使用 SensorDataQoS 适合高频传感器数据，与 RViz2 兼容）
    rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos);

    if (!mock_mode_) {
        // 真实相机模式
        if (!openCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", device_name_.c_str());
            return false;
        }
    }

    is_running_ = true;
    capture_thread_ = std::thread(&CameraNode::captureLoop, this);

    RCLCPP_INFO(this->get_logger(), "Camera Node started successfully");
    return true;
}

void CameraNode::stop()
{
    is_running_ = false;
    
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    if (!mock_mode_) {
        closeCamera();
    }

    RCLCPP_INFO(this->get_logger(), "Camera Node stopped");
}

bool CameraNode::openCamera()
{
    cap_.open(device_name_);
    
    if (!cap_.isOpened()) {
        return false;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    return true;
}

void CameraNode::closeCamera()
{
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void CameraNode::captureLoop()
{
    cv::Mat frame;
    auto period = std::chrono::milliseconds(1000 / fps_);

    while (is_running_) {
        auto start = std::chrono::steady_clock::now();

        if (mock_mode_) {
            // 模拟模式：生成测试图像
            generateMockImage(frame);
        } else {
            // 真实相机模式
            if (!cap_.read(frame)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
                continue;
            }
        }

        // 发布图像
        if (!frame.empty()) {
            publishImage(frame);
            frame_count_++;

            // 每 5 秒输出一次统计
            auto elapsed = (this->now() - start_time_).seconds();
            if (elapsed > 0 && frame_count_ % (fps_ * 5) == 0) {
                double actual_fps = frame_count_ / elapsed;
                RCLCPP_INFO(this->get_logger(), 
                    "Frames: %d, Elapsed: %.1fs, Actual FPS: %.1f",
                    frame_count_, elapsed, actual_fps);
            }
        }

        // 控制帧率
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

void CameraNode::generateMockImage(cv::Mat &image)
{
    image.create(image_height_, image_width_, CV_8UC3);
    
    static int frame_num = 0;
    frame_num++;

    if (mock_pattern_ == "color_bar") {
        // 生成移动彩色条纹
        int bar_width = image_width_ / 8;
        for (int i = 0; i < image_width_; i++) {
            int color_idx = ((i / bar_width) + frame_num / 5) % 8;
            cv::Scalar color;
            switch (color_idx) {
                case 0: color = cv::Scalar(255, 255, 255); break;  // 白
                case 1: color = cv::Scalar(255, 255, 0); break;   // 黄
                case 2: color = cv::Scalar(0, 255, 255); break;   // 青
                case 3: color = cv::Scalar(0, 255, 0); break;     // 绿
                case 4: color = cv::Scalar(255, 0, 255); break;   // 品红
                case 5: color = cv::Scalar(255, 0, 0); break;     // 红
                case 6: color = cv::Scalar(0, 0, 255); break;     // 蓝
                case 7: color = cv::Scalar(0, 0, 0); break;       // 黑
            }
            cv::line(image, cv::Point(i, 0), cv::Point(i, image_height_), color, 1);
        }
        
        // 添加文字
        cv::putText(image, "MOCK MODE - Color Bar", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(image, "Frame: " + std::to_string(frame_num), cv::Point(20, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }
    else if (mock_pattern_ == "noise") {
        // 随机噪声
        cv::randu(image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        
        cv::putText(image, "MOCK MODE - Noise", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }
    else if (mock_pattern_ == "gradient") {
        // 渐变图案
        for (int y = 0; y < image_height_; y++) {
            for (int x = 0; x < image_width_; x++) {
                image.at<cv::Vec3b>(y, x)[0] = (x * 255) / image_width_;   // B
                image.at<cv::Vec3b>(y, x)[1] = (y * 255) / image_height_;  // G
                image.at<cv::Vec3b>(y, x)[2] = ((x + y) * 255) / (image_width_ + image_height_);  // R
            }
        }
        
        cv::putText(image, "MOCK MODE - Gradient", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }
    else {
        // 默认纯色背景
        image = cv::Scalar(128, 128, 128);
        cv::putText(image, "MOCK MODE", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }
}

void CameraNode::publishImage(const cv::Mat &image)
{
    // 转换 OpenCV 图像到 ROS2 消息
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = this->get_parameter("frame_id").as_string();

    // 使用 cv_bridge 转换
    cv_bridge::CvImage cv_image(header, "bgr8", image);
    sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();

    // 发布
    image_pub_->publish(*msg);
}
