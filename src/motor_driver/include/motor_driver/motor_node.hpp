#ifndef __MOTOR_NODE_HPP
#define __MOTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>

/**
 * @brief ROS2 Motor 节点类
 * 
 * 功能：
 * 1. 接收目标关节位置（/joint_commands）
 * 2. 发布实际关节状态（/joint_states）
 * 3. 支持模拟模式（无硬件测试）
 * 4. 模拟真实电机的响应延迟和限制
 */
class MotorNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    explicit MotorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief 析构函数
     */
    ~MotorNode();

    /**
     * @brief 启动电机控制
     */
    bool start();

    /**
     * @brief 停止电机控制
     */
    void stop();

private:
    /**
     * @brief 控制循环线程
     */
    void controlLoop();

    /**
     * @brief 发布关节状态
     */
    void publishJointStates();

    /**
     * @brief 处理目标位置命令
     */
    void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief 更新模拟电机位置（向目标位置移动）
     */
    void updateMockPositions();

    /**
     * @brief 限制关节角度范围
     */
    double limitJointAngle(const std::string &name, double angle);

    /**
     * @brief 检查是否连接（模拟模式始终返回 true）
     */
    bool isConnected() const;

    // ============== ROS2 发布者和订阅者 ==============
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_commands_sub_;

    // ============== 参数 ==============
    bool mock_mode_;                    // 模拟模式
    std::string device_name_;           // 串口设备（真实模式）
    int baudrate_;                      // 波特率
    double control_rate_;               // 控制频率（Hz）
    double max_speed_;                  // 最大关节速度（度/秒）
    std::vector<std::string> joint_names_;  // 关节名称列表
    std::map<std::string, std::pair<double, double>> joint_limits_;  // 关节限制（min, max）

    // ============== 状态数据 ==============
    std::map<std::string, double> current_positions_;   // 当前位置
    std::map<std::string, double> target_positions_;    // 目标位置
    std::map<std::string, double> current_velocities_;  // 当前速度
    std::map<std::string, double> current_efforts_;     // 当前力矩

    // ============== 线程控制 ==============
    std::atomic_bool is_running_{false};
    std::thread control_thread_;
    std::mutex data_mutex_;

    // ============== 统计 ==============
    int loop_count_ = 0;
    rclcpp::Time start_time_;
};

#endif // __MOTOR_NODE_HPP
