#ifndef __CAMERA_NODE_HPP
#define __CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <atomic>

/**
 * @brief ROS2 Camera 节点类
 * 
 * 功能：
 * 1. 从相机采集图像（V4L2 或 MindVision）
 * 2. 发布 sensor_msgs/Image 消息到 /camera/image_raw
 * 3. 支持模拟模式（无硬件测试）
 */
class CameraNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    explicit CameraNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief 析构函数
     */
    ~CameraNode();

    /**
     * @brief 启动相机采集
     */
    bool start();

    /**
     * @brief 停止相机采集
     */
    void stop();

private:
    /**
     * @brief 相机采集线程函数
     */
    void captureLoop();

    /**
     * @brief 模拟模式：生成测试图像
     */
    void generateMockImage(cv::Mat &image);

    /**
     * @brief 发布图像消息
     */
    void publishImage(const cv::Mat &image);

    /**
     * @brief 打开真实相机（V4L2）
     */
    bool openCamera();

    /**
     * @brief 关闭相机
     */
    void closeCamera();

    // ============== ROS2 发布者 ==============
    // 使用标准 Publisher 而不是 image_transport，确保 RViz2 兼容
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // ============== 参数 ==============
    std::string device_name_;        // 设备路径，如 /dev/video0
    int image_width_;                // 图像宽度
    int image_height_;               // 图像高度
    int fps_;                        // 帧率
    bool mock_mode_;                 // 模拟模式
    std::string mock_pattern_;       // 模拟图案类型 (color_bar, noise, gradient)

    // ============== OpenCV 相机 ==============
    cv::VideoCapture cap_;

    // ============== 线程控制 ==============
    std::atomic_bool is_running_{false};
    std::thread capture_thread_;
    
    // ============== 统计 ==============
    int frame_count_ = 0;
    rclcpp::Time start_time_;
};

#endif // __CAMERA_NODE_HPP
