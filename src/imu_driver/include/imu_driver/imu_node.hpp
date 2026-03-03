#ifndef __IMU_NODE_HPP
#define __IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int8.hpp>
#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <eigen3/Eigen/Dense>

// 摔倒方向枚举，与原始代码保持一致
enum FallDirection
{
    FALL_NONE = 0,
    FALL_FORWARD = 1,
    FALL_BACKWARD = -1,
    FALL_LEFT = 2,
    FALL_RIGHT = -2
};

/**
 * @brief ROS2 IMU 节点类
 * 
 * 功能：
 * 1. 通过串口读取 IMU 数据
 * 2. 发布 sensor_msgs/Imu 消息到 /imu/data
 * 3. 发布摔倒检测消息到 /imu/fall_direction
 */
class ImuNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数，声明参数并初始化
     */
    explicit ImuNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief 析构函数，确保线程安全退出
     */
    ~ImuNode();

    /**
     * @brief 启动 IMU 读取线程
     */
    bool start();
    
    /**
     * @brief 停止 IMU 读取
     */
    void stop();

private:
    // ============== IMU 数据结构与协议 ==============
    
    struct ImuData
    {
        float pitch = 0.0, roll = 0.0, yaw = 0.0;
        float ax = 0.0, ay = 0.0, az = 0.0;      // 加速度 (m/s^2)
        float wx = 0.0, wy = 0.0, wz = 0.0;      // 角速度 (rad/s)
        int timestamp = 0;
    };

    enum ItemID_t
    {
        kItemID = 0x90,
        kItemAccRaw = 0xA0,
        kItemAccCalibrated = 0xA1,
        kItemAccFiltered = 0xA2,
        kItemAccLinear = 0xA5,
        kItemGyoRaw = 0xB0,
        kItemGyoCalibrated = 0xB1,
        kItemGyoFiltered = 0xB2,
        kItemRotationEular = 0xD0,
        kItemRotationEular2 = 0xD9,
        kItemRotationQuat = 0xD1,
        kItemPressure = 0xF0,
        kItemTemperature = 0xF1,
        kItemEnd = 0x00,
    };

    struct Packet_t
    {
        uint32_t ofs;
        uint8_t buf[128];  // MAX_PACKET_LEN
        uint16_t payload_len;
        uint16_t len;
        uint8_t type;
    };

    // ============== ROS2 发布者 ==============
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;           // /imu/data
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr fall_direction_pub_;  // /imu/fall_direction

    // ============== 参数 ==============
    
    std::string dev_name_;           // 串口设备名
    int baudrate_;                   // 波特率
    Eigen::Vector2f pitch_range_;    // 不摔倒的俯仰角范围
    Eigen::Vector2f roll_range_;     // 不摔倒的横滚角范围
    std::string frame_id_;           // TF 坐标系名
    float init_dir_;                 // 初始方向偏移
    bool mock_mode_;                 // 模拟模式（无硬件测试用）

    // ============== Boost.Asio 串口 ==============
    
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_;
    std::thread io_thread_;          // asio 事件循环线程

    // ============== 数据缓冲区 ==============
    
    unsigned char buff_[2];
    Packet_t rx_pkt_;
    int16_t acc_[3];
    int16_t gyo_[3];
    int16_t mag_[3];
    float eular_[3];
    float quat_[4];
    uint8_t id;

    // ============== 状态 ==============
    
    std::atomic_bool is_running_{false};
    std::atomic_int fall_direction_{FALL_NONE};
    std::mutex data_mutex_;
    
    // ============== 模拟模式定时器 ==============
    rclcpp::TimerBase::SharedPtr mock_timer_;
    ImuData current_data_;

    // ============== 私有方法 ==============
    
    /**
     * @brief 打开串口
     */
    bool openSerial();
    
    /**
     * @brief 异步读取数据循环
     */
    void readData();
    
    /**
     * @brief 数据包解码状态机
     */
    bool packetDecode(uint8_t c);
    
    /**
     * @brief 数据包处理回调
     */
    void onDataReceived(Packet_t &pkt);
    
    /**
     * @brief 将原始数据转换为 ROS Imu 消息并发布
     */
    void publishImuData(const ImuData &data);
    
    /**
     * @brief 计算并发布摔倒方向
     */
    void publishFallDirection(float pitch, float roll);

    /**
     * @brief 模拟模式：生成并发布模拟数据
     */
    void publishMockData();
    
    /**
     * @brief CRC16 校验计算
     */
    void crc16Update(uint16_t *currentCrc, const uint8_t *src, uint32_t length);
};

#endif // __IMU_NODE_HPP
