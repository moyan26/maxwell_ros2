#include "imu_driver/imu_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

// 常量定义
const float G = 9.8;

// 状态机状态
enum DecodeStatus
{
    kStatus_Idle,//空闲/等待帧头
    kStatus_Cmd,//命令类型
    kStatus_LenLow,//长度低8位
    kStatus_LenHigh,//长度高8位
    kStatus_CRCLow,//CRC校验低8位
    kStatus_CRCHigh,//CRC校验高8位
    kStatus_Data,//接收Payload数据
};

ImuNode::ImuNode(const rclcpp::NodeOptions &options)
    : Node("imu_node", options)
    , serial_(io_service_)
    , init_dir_(0.0)
{
    // ============== 声明并获取参数 ==============
    // 替代原来的 CONF->get_config_value
    this->declare_parameter<bool>("mock_mode", false);  // 模拟模式
    this->declare_parameter<std::string>("dev_name", "/dev/ttyTHS2");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<std::vector<double>>("pitch_range", {-30.0, 40.0});
    this->declare_parameter<std::vector<double>>("roll_range", {-40.0, 40.0});
    this->declare_parameter<std::string>("frame_id", "imu_link");

    mock_mode_ = this->get_parameter("mock_mode").as_bool();
    dev_name_ = this->get_parameter("dev_name").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    auto pitch_vec = this->get_parameter("pitch_range").as_double_array();
    auto roll_vec = this->get_parameter("roll_range").as_double_array();
    frame_id_ = this->get_parameter("frame_id").as_string();

    pitch_range_ << pitch_vec[0], pitch_vec[1];
    roll_range_ << roll_vec[0], roll_vec[1];

    // ============== 创建发布者 ==============
    // QoS 设置：使用 SensorDataQoS 适合高频传感器数据
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
    fall_direction_pub_ = this->create_publisher<std_msgs::msg::Int8>("imu/fall_direction", 10);

    RCLCPP_INFO(this->get_logger(), "IMU Node initialized");
    RCLCPP_INFO(this->get_logger(), "Mock mode: %s", mock_mode_ ? "enabled" : "disabled");
    if (!mock_mode_) {
        RCLCPP_INFO(this->get_logger(), "Device: %s, Baudrate: %d", dev_name_.c_str(), baudrate_);
    }
    RCLCPP_INFO(this->get_logger(), "Pitch range: [%.1f, %.1f], Roll range: [%.1f, %.1f]",
                pitch_range_.x(), pitch_range_.y(), roll_range_.x(), roll_range_.y());
}

ImuNode::~ImuNode()
{
    stop();
}

bool ImuNode::start()
{
    if (mock_mode_) {
        // 模拟模式：创建定时器生成模拟数据
        is_running_ = true;
        mock_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz
            [this]() { this->publishMockData(); });
        RCLCPP_INFO(this->get_logger(), "IMU Node started in MOCK mode (100Hz)");
        return true;
    }

    if (!openSerial()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", dev_name_.c_str());
        return false;
    }

    is_running_ = true;
    
    // 启动 Boost.Asio 事件循环线程
    io_thread_ = std::thread([this]() {
        io_service_.run();
    });

    // 启动异步读取
    readData();

    RCLCPP_INFO(this->get_logger(), "IMU Node started successfully");
    return true;
}

void ImuNode::stop()
{
    is_running_ = false;
    
    // 关闭串口和 io_service
    if (serial_.is_open()) {
        serial_.close();
    }
    io_service_.stop();

    // 等待线程结束
    if (io_thread_.joinable()) {
        io_thread_.join();
    }

    RCLCPP_INFO(this->get_logger(), "IMU Node stopped");
}

bool ImuNode::openSerial()
{
    try {
        serial_.open(dev_name_);
        serial_.set_option(boost::asio::serial_port::baud_rate(baudrate_));
        serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial_.set_option(boost::asio::serial_port::character_size(8));
        return true;
    } catch (std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
        return false;
    }
}

void ImuNode::readData()
{
    if (!is_running_) return;

    // 异步读取一个字节
    serial_.async_read_some(
        boost::asio::buffer(buff_, 1),
        [this](boost::system::error_code ec, std::size_t bytes_transferred) {
            if (!ec && bytes_transferred > 0) {
                // 解码数据
                packetDecode(buff_[0]);
            } else if (ec) {
                RCLCPP_WARN(this->get_logger(), "Serial read error: %s", ec.message().c_str());
            }
            // 继续读取
            readData();
        }
    );
}

void ImuNode::crc16Update(uint16_t *currentCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currentCrc;
    for (uint32_t j = 0; j < lengthInBytes; ++j) {
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (uint32_t i = 0; i < 8; ++i) {
            uint32_t temp = crc << 1;
            if (crc & 0x8000) {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currentCrc = crc;
}

bool ImuNode::packetDecode(uint8_t c)
{
    static uint16_t CRCReceived = 0;
    static uint16_t CRCCalculated = 0;
    static uint8_t status = kStatus_Idle;
    static uint8_t crc_header[4] = {0x5A, 0xA5, 0x00, 0x00};

    switch (status) {
        case kStatus_Idle:
            if (c == 0x5A)
                status = kStatus_Cmd;
            break;
        case kStatus_Cmd:
            rx_pkt_.type = c;
            switch (rx_pkt_.type) {
                case 0xA5:  // Data
                    status = kStatus_LenLow;
                    break;
                case 0xA6:  // Ping
                    status = kStatus_Idle;
                    break;
                case 0xA7:  // Ping Respond
                    rx_pkt_.ofs = 0;
                    status = kStatus_Data;
                    break;
                default:
                    status = kStatus_Idle;
                    break;
            }
            break;
        case kStatus_LenLow:
            rx_pkt_.payload_len = c;
            crc_header[2] = c;
            status = kStatus_LenHigh;
            break;
        case kStatus_LenHigh:
            rx_pkt_.payload_len |= (c << 8);
            crc_header[3] = c;
            status = kStatus_CRCLow;
            break;
        case kStatus_CRCLow:
            CRCReceived = c;
            status = kStatus_CRCHigh;
            break;
        case kStatus_CRCHigh:
            CRCReceived |= (c << 8);
            rx_pkt_.ofs = 0;
            CRCCalculated = 0;
            status = kStatus_Data;
            break;
        case kStatus_Data:
            rx_pkt_.buf[rx_pkt_.ofs++] = c;
            if (rx_pkt_.type == 0xA7 && rx_pkt_.ofs >= 8) {
                rx_pkt_.payload_len = 8;
                onDataReceived(rx_pkt_);
                status = kStatus_Idle;
            }
            if (rx_pkt_.ofs >= 128) {  // MAX_PACKET_LEN
                status = kStatus_Idle;
                return false;
            }
            if (rx_pkt_.ofs >= rx_pkt_.payload_len && rx_pkt_.type == 0xA5) {
                // 计算 CRC
                crc16Update(&CRCCalculated, crc_header, 4);
                crc16Update(&CRCCalculated, rx_pkt_.buf, rx_pkt_.ofs);
                // CRC 匹配
                if (CRCCalculated == CRCReceived) {
                    onDataReceived(rx_pkt_);
                }
                status = kStatus_Idle;
            }
            break;
        default:
            status = kStatus_Idle;
            break;
    }
    return true;
}

void ImuNode::onDataReceived(Packet_t &pkt)
{
    if (pkt.type != 0xA5) {
        return;
    }

    int offset = 0;
    uint8_t *p = pkt.buf;
    ImuData data;

    while (offset < pkt.payload_len) {
        switch (p[offset]) {
            case kItemID:
                id = p[1];
                offset += 2;
                break;
            case kItemAccRaw:
            case kItemAccCalibrated:
            case kItemAccFiltered:
            case kItemAccLinear:
                memcpy(acc_, p + offset + 1, sizeof(acc_));
                data.ax = acc_[0] * 0.01f * G;  // 转换为 m/s^2
                data.ay = acc_[1] * 0.01f * G;
                data.az = acc_[2] * 0.01f * G;
                offset += 7;
                break;
            case kItemGyoRaw:
            case kItemGyoCalibrated:
            case kItemGyoFiltered:
                memcpy(gyo_, p + offset + 1, sizeof(gyo_));
                data.wx = gyo_[0] * 0.01f * (M_PI / 180.0f);  // 转换为 rad/s
                data.wy = gyo_[1] * 0.01f * (M_PI / 180.0f);
                data.wz = gyo_[2] * 0.01f * (M_PI / 180.0f);
                offset += 7;
                break;
            case kItemRotationEular:
                eular_[0] = ((float)(int16_t)(p[offset + 1] + (p[offset + 2] << 8))) / 100.0f;
                eular_[1] = ((float)(int16_t)(p[offset + 3] + (p[offset + 4] << 8))) / 100.0f;
                eular_[2] = ((float)(int16_t)(p[offset + 5] + (p[offset + 6] << 8))) / 10.0f;
                data.pitch = eular_[0];
                data.roll = eular_[1];
                data.yaw = eular_[2];
                
                // 计算摔倒方向
                if (data.pitch < pitch_range_.x()) {
                    fall_direction_ = FALL_BACKWARD;
                } else if (data.pitch > pitch_range_.y()) {
                    fall_direction_ = FALL_FORWARD;
                } else if (data.roll < roll_range_.x()) {
                    fall_direction_ = FALL_RIGHT;
                } else if (data.roll > roll_range_.y()) {
                    fall_direction_ = FALL_LEFT;
                } else {
                    fall_direction_ = FALL_NONE;
                }
                offset += 7;
                break;
            case kItemRotationEular2:
                memcpy(eular_, p + offset + 1, sizeof(eular_));
                offset += 13;
                break;
            case kItemRotationQuat:
                memcpy(quat_, p + offset + 1, sizeof(quat_));
                offset += 17;
                break;
            case kItemPressure:
                offset += 5;
                break;
            case kItemTemperature:
                offset += 5;
                break;
            default:
                return;
        }
    }

    // 设置时间戳
    data.timestamp = this->now().nanoseconds() / 1000000;  // 毫秒时间戳
    
    // 归一化 yaw 角
    data.yaw = data.yaw - init_dir_;
    // 保持在 -180 ~ 180 范围
    while (data.yaw > 180.0f) data.yaw -= 360.0f;
    while (data.yaw < -180.0f) data.yaw += 360.0f;

    // 更新当前数据
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_data_ = data;
    }

    // 发布数据
    publishImuData(data);
    publishFallDirection(data.pitch, data.roll);
}

void ImuNode::publishImuData(const ImuData &data)
{
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    // 使用 tf2 将欧拉角转换为四元数
    // ROS 坐标系约定：x-前，y-左，z-上
    // IMU 数据：pitch 绕 y，roll 绕 x，yaw 绕 z
    tf2::Quaternion q;
    q.setRPY(
        data.roll * M_PI / 180.0f,   // roll -> rad
        data.pitch * M_PI / 180.0f,  // pitch -> rad
        data.yaw * M_PI / 180.0f     // yaw -> rad
    );
    
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();

    // 角速度 (rad/s)
    msg.angular_velocity.x = data.wx;
    msg.angular_velocity.y = data.wy;
    msg.angular_velocity.z = data.wz;

    // 线性加速度 (m/s^2)
    msg.linear_acceleration.x = data.ax;
    msg.linear_acceleration.y = data.ay;
    msg.linear_acceleration.z = data.az;

    // 协方差矩阵（未知，设为 0）
    msg.orientation_covariance[0] = -1;  // -1 表示未知
    msg.angular_velocity_covariance[0] = -1;
    msg.linear_acceleration_covariance[0] = -1;

    imu_pub_->publish(msg);
}

void ImuNode::publishFallDirection(float pitch, float roll)
{
    auto msg = std_msgs::msg::Int8();
    msg.data = fall_direction_.load();
    fall_direction_pub_->publish(msg);

    // DEBUG 输出（可选）
    static int count = 0;
    if (++count % 100 == 0) {  // 每 100 帧输出一次
        RCLCPP_DEBUG(this->get_logger(), 
            "Pitch: %.2f, Roll: %.2f, Yaw: %.2f, Fall: %d",
            pitch, roll, current_data_.yaw, msg.data);
    }
}

void ImuNode::publishMockData()
{
    static float t = 0.0f;
    t += 0.01f;  // 10ms 步进

    ImuData data;
    
    // 生成正弦波模拟数据
    data.pitch = 5.0f * std::sin(t);           // 俯仰角 ±5度
    data.roll = 3.0f * std::cos(t * 0.7f);     // 横滚角 ±3度  
    data.yaw = 10.0f * std::sin(t * 0.3f);     // 偏航角
    
    // 角速度 (rad/s) - 模拟微小抖动
    data.wx = 0.05f * std::sin(t * 2.0f);
    data.wy = 0.03f * std::cos(t * 1.5f);
    data.wz = 0.02f * std::sin(t * 0.5f);
    
    // 线性加速度 (m/s^2) - 基础重力 + 微小波动
    data.ax = 0.1f * std::sin(t * 1.2f);
    data.ay = 0.08f * std::cos(t * 0.9f);
    data.az = 9.8f + 0.1f * std::sin(t * 0.8f);
    
    data.timestamp = this->now().nanoseconds() / 1000000;

    // 计算摔倒检测
    if (data.pitch < pitch_range_.x()) {
        fall_direction_ = FALL_BACKWARD;
    } else if (data.pitch > pitch_range_.y()) {
        fall_direction_ = FALL_FORWARD;
    } else if (data.roll < roll_range_.x()) {
        fall_direction_ = FALL_RIGHT;
    } else if (data.roll > roll_range_.y()) {
        fall_direction_ = FALL_LEFT;
    } else {
        fall_direction_ = FALL_NONE;
    }

    // 发布数据
    publishImuData(data);
    publishFallDirection(data.pitch, data.roll);

    // 每 5 秒输出一次状态
    static int count = 0;
    if (++count % 500 == 0) {
        RCLCPP_INFO(this->get_logger(), 
            "[Mock] Pitch: %.2f°, Roll: %.2f°, Yaw: %.2f°, Fall: %d",
            data.pitch, data.roll, data.yaw, fall_direction_.load());
    }
}
