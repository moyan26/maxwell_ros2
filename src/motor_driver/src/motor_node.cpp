#include "motor_driver/motor_node.hpp"
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

MotorNode::MotorNode(const rclcpp::NodeOptions &options)
    : Node("motor_node", options)
    , start_time_(this->now())
{
    // ============== 声明并获取参数 ==============
    this->declare_parameter<bool>("mock_mode", true);  // 默认模拟模式
    this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 3000000);
    this->declare_parameter<double>("control_rate", 100.0);  // 100Hz
    this->declare_parameter<double>("max_speed", 360.0);     // 360度/秒
    
    // 关节名称列表（18个自由度，与 robot.conf 对应）
    this->declare_parameter<std::vector<std::string>>("joint_names", {
        // 头部 (2)
        "jhead1", "jhead2",
        // 右臂 (2)
        "jrshoulder1", "jrelbow",
        // 左臂 (2)
        "jlshoulder1", "jlelbow",
        // 右腿 (6)
        "jrhip3", "jrhip2", "jrhip1", "jrknee", "jrankle2", "jrankle1",
        // 左腿 (6)
        "jlhip3", "jlhip2", "jlhip1", "jlknee", "jlankle2", "jlankle1"
    });

    mock_mode_ = this->get_parameter("mock_mode").as_bool();
    device_name_ = this->get_parameter("device_name").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    control_rate_ = this->get_parameter("control_rate").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    joint_names_ = this->get_parameter("joint_names").as_string_array();

    // 设置关节限制（度），与 robot.conf 中的 min/max 对应
    // 头部
    joint_limits_["jhead1"] = {-180.0, 180.0};
    joint_limits_["jhead2"] = {-180.0, 180.0};
    
    // 右臂
    joint_limits_["jrshoulder1"] = {-180.0, 180.0};
    joint_limits_["jrelbow"] = {-180.0, 180.0};
    
    // 左臂
    joint_limits_["jlshoulder1"] = {-180.0, 180.0};
    joint_limits_["jlelbow"] = {-180.0, 180.0};
    
    // 右腿
    joint_limits_["jrhip3"] = {-180.0, 180.0};
    joint_limits_["jrhip2"] = {-180.0, 180.0};
    joint_limits_["jrhip1"] = {-180.0, 180.0};
    joint_limits_["jrknee"] = {-180.0, 180.0};
    joint_limits_["jrankle2"] = {-180.0, 180.0};
    joint_limits_["jrankle1"] = {-180.0, 180.0};
    
    // 左腿
    joint_limits_["jlhip3"] = {-180.0, 180.0};
    joint_limits_["jlhip2"] = {-180.0, 180.0};
    joint_limits_["jlhip1"] = {-180.0, 180.0};
    joint_limits_["jlknee"] = {-180.0, 180.0};
    joint_limits_["jlankle2"] = {-180.0, 180.0};
    joint_limits_["jlankle1"] = {-180.0, 180.0};

    // 初始化关节位置为 0
    for (const auto &name : joint_names_) {
        current_positions_[name] = 0.0;
        target_positions_[name] = 0.0;
        current_velocities_[name] = 0.0;
        current_efforts_[name] = 0.0;
    }

    // ============== 创建发布者和订阅者 ==============
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    
    joint_commands_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_commands", 10,
        std::bind(&MotorNode::onJointCommand, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Motor Node initialized");
    RCLCPP_INFO(this->get_logger(), "Mock mode: %s", mock_mode_ ? "enabled" : "disabled");
    if (!mock_mode_) {
        RCLCPP_INFO(this->get_logger(), "Device: %s, Baudrate: %d", 
                    device_name_.c_str(), baudrate_);
    }
    RCLCPP_INFO(this->get_logger(), "Control rate: %.1f Hz", control_rate_);
    RCLCPP_INFO(this->get_logger(), "Joint count: %zu", joint_names_.size());
}

MotorNode::~MotorNode()
{
    stop();
}

bool MotorNode::start()
{
    if (is_running_) {
        return true;
    }

    if (!mock_mode_) {
        // TODO: 初始化 Dynamixel SDK
        RCLCPP_WARN(this->get_logger(), 
            "Real motor mode not fully implemented, using mock mode");
        mock_mode_ = true;
    }

    is_running_ = true;
    control_thread_ = std::thread(&MotorNode::controlLoop, this);

    RCLCPP_INFO(this->get_logger(), "Motor Node started successfully");
    return true;
}

void MotorNode::stop()
{
    is_running_ = false;
    
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    RCLCPP_INFO(this->get_logger(), "Motor Node stopped");
}

void MotorNode::controlLoop()
{
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    
    while (is_running_) {
        auto start = std::chrono::steady_clock::now();

        // 更新位置（模拟模式）
        if (mock_mode_) {
            updateMockPositions();
        }

        // 发布关节状态
        publishJointStates();

        // 统计
        loop_count_++;
        if (loop_count_ % static_cast<int>(control_rate_) == 0) {
            auto elapsed = (this->now() - start_time_).seconds();
            double actual_rate = loop_count_ / elapsed;
            RCLCPP_INFO(this->get_logger(), 
               "Control loop: %d iterations, actual rate: %.1f Hz",
                loop_count_, actual_rate);
        }

        // 控制频率
        auto end = std::chrono::steady_clock::now();
        auto elapsed = end - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

void MotorNode::updateMockPositions()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    double dt = 1.0 / control_rate_;
    
    for (const auto &name : joint_names_) {
        double current = current_positions_[name];
        double target = target_positions_[name];
        
        // 计算最大位移
        double max_delta = max_speed_ * dt;
        
        // 向目标位置移动
        double delta = target - current;
        delta = std::clamp(delta, -max_delta, max_delta);
        
        double new_pos = current + delta;
        
        // 应用关节限制
        new_pos = limitJointAngle(name, new_pos);
        
        current_positions_[name] = new_pos;
        
        // 计算速度
        current_velocities_[name] = delta / dt;
        
        // 模拟力矩（简单模型）
        current_efforts_[name] = delta * 0.1;  // 比例于移动距离
    }
}

void MotorNode::publishJointStates()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    
    for (const auto &name : joint_names_) {
        msg.name.push_back(name);
        msg.position.push_back(current_positions_[name] * M_PI / 180.0);  // 转换为弧度
        msg.velocity.push_back(current_velocities_[name] * M_PI / 180.0);
        msg.effort.push_back(current_efforts_[name]);
    }
    
    joint_states_pub_->publish(msg);
}

void MotorNode::onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string &name = msg->name[i];
        
        // 检查是否为已知关节
        if (target_positions_.find(name) != target_positions_.end()) {
            // 转换为度并限制范围
            double target_deg = msg->position[i] * 180.0 / M_PI;
            target_positions_[name] = limitJointAngle(name, target_deg);
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Received joint command with %zu joints", msg->name.size());
}

double MotorNode::limitJointAngle(const std::string &name, double angle)
{
    auto it = joint_limits_.find(name);
    if (it != joint_limits_.end()) {
        return std::clamp(angle, it->second.first, it->second.second);
    }
    return angle;
}

bool MotorNode::isConnected() const
{
    return true;  // 模拟模式始终连接
}
