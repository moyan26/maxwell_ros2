# Motor 模块 ROS2 迁移对比文档

## 1. 概述

本文档对比了 RoboCup 机器人 Motor 模块在迁移到 ROS2 前后的架构差异。

- **原架构**：基于观察者模式的纯 C++ 实现，直接调用 Dynamixel SDK
- **新架构**：基于 ROS2 的发布/订阅模式，支持模拟模式

---

## 2. 架构对比

### 2.1 整体架构

#### 原架构（基于观察者模式）
```
┌─────────────────────────────────────────────────────────────┐
│                      原 Motor 模块                           │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────┐     ┌─────────────────┐                │
│  │   Motor 类      │────▶│  Dynamixel SDK  │                │
│  │  (继承 Sensor)  │     │  (串口通信)      │                │
│  └────────┬────────┘     └─────────────────┘                │
│           │                                                  │
│           │ notify(SENSOR_MOTOR)                            │
│           ▼                                                  │
│  ┌──────────────────────────────────────┐                   │
│  │          其他模块（观察者）            │                   │
│  │  - Robot::set_real_degs()            │                   │
│  │  - WorldModel                        │                   │
│  │  - 其他传感器                         │                   │
│  └──────────────────────────────────────┘                   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

#### 新架构（基于 ROS2）
```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Motor 节点                           │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────┐     ┌─────────────────┐                │
│  │   MotorNode     │◀───▶│  模拟/真实电机   │                │
│  │  (rclcpp::Node) │     │  (可切换)        │                │
│  └────────┬────────┘     └─────────────────┘                │
│           │                                                  │
│    ┌──────┴──────┐                                          │
│    │             │                                          │
│    ▼             ▼                                          │
│ /joint_commands  /joint_states                              │
│ (订阅)           (发布)                                      │
│    │             │                                          │
│    │    ┌────────┴────────┐                                 │
│    │    │                 │                                 │
│    ▼    ▼                 ▼                                 │
│ 决策节点              RViz2/调试工具                          │
│ (发送目标位置)        (显示实际位置)                          │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 详细对比

### 3.1 类定义对比

| 特性 | 原代码 | ROS2 代码 |
|------|--------|-----------|
| **基类** | `Sensor` + `Timer` | `rclcpp::Node` |
| **文件位置** | `controller/player/sensor/motor.hpp` | `motor_driver/include/motor_driver/motor_node.hpp` |
| **类名** | `Motor` | `MotorNode` |
| **命名空间** | 无 | `motor_driver` |

#### 原代码类定义
```cpp
// motor.hpp
class Motor: public Sensor, public Timer
{
public:
    Motor();
    ~Motor();
    bool start();
    void stop();
    void run();
    
private:
    void set_torq(uint8_t e);
    void set_led(uint8_t s);
    void set_gpos();
    bool read_all_pos();
    
    // Dynamixel SDK 对象
    std::shared_ptr<dynamixel::PortHandler> portHandler_;
    std::shared_ptr<dynamixel::PacketHandler> packetHandler_;
    std::shared_ptr<dynamixel::GroupSyncWrite> torqWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> gposWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> ledWrite_;
    std::shared_ptr<dynamixel::GroupSyncRead> pposRead_;
};
```

#### ROS2 类定义
```cpp
// motor_node.hpp
class MotorNode : public rclcpp::Node
{
public:
    explicit MotorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MotorNode();
    bool start();
    void stop();

private:
    void controlLoop();
    void publishJointStates();
    void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg);
    void updateMockPositions();
    
    // ROS2 发布者和订阅者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_commands_sub_;
    
    // 参数
    bool mock_mode_;
    double control_rate_;
    std::vector<std::string> joint_names_;
};
```

---

### 3.2 输入输出对比

#### 输入方式

| 方式 | 原代码 | ROS2 代码 |
|------|--------|-----------|
| **命令输入** | 内部调用 `ROBOT->set_degs()` | 订阅 `/joint_commands` Topic |
| **参数配置** | 配置文件 `hardware.conf` | ROS2 参数系统 |
| **实时性** | 直接函数调用 | Topic 回调（约 1ms 延迟） |

#### 原代码输入
```cpp
// 原代码：通过全局对象 ROBOT 设置目标位置
void SomeModule::planMotion() {
    std::map<int, float> target_degs;
    target_degs[11] = 30.0;  // left_hip_pitch
    target_degs[12] = -60.0; // left_knee_pitch
    
    ROBOT->set_degs(target_degs);  // 内部调用，Motor 模块通过 Timer 读取
}
```

#### ROS2 输入
```cpp
// ROS2：通过 Topic 发送命令
// 终端命令
ros2 topic pub /joint_commands sensor_msgs/msg/JointState "{
  name: ['left_hip_pitch', 'left_knee_pitch'],
  position: [0.524, -1.047]
}"

// 或代码中
auto cmd = sensor_msgs::msg::JointState();
cmd.name = {"left_hip_pitch", "left_knee_pitch"};
cmd.position = {0.524, -1.047};  // 弧度
joint_cmd_pub_->publish(cmd);
```

#### 输出方式

| 方式 | 原代码 | ROS2 代码 |
|------|--------|-----------|
| **状态输出** | `ROBOT->set_real_degs()` | 发布 `/joint_states` Topic |
| **数据内容** | 仅位置 | 位置 + 速度 + 力矩 |
| **可观测性** | 需调试代码查看 | RViz2/ros2 topic echo |

---

### 3.3 控制循环对比

#### 原代码控制循环
```cpp
// motor.cpp
void Motor::run() {
    if (Timer::is_alive_) {
        // 获取目标位置（从 ROBOT 对象）
        ROBOT->set_degs(MADT->get_degs());
        
        if (OPTS->use_debug())
            virtul_act();  // 仿真模式
            
        if (OPTS->use_robot())
            real_act();      // 真实电机
            
        notify(SENSOR_MOTOR);  // 通知观察者
        p_count_++;
    }
}

void Motor::real_act() {
    if (!is_connected_) {
        // 检查连接
        packetHandler_->ping(portHandler_, ping_id_, ...);
        set_torq(1);
        is_connected_ = true;
    } else {
        set_gpos();  // 发送目标位置到电机
        set_led(led_status_);
    }
}
```

#### ROS2 控制循环
```cpp
// motor_node.cpp
void MotorNode::controlLoop() {
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    
    while (is_running_) {
        auto start = std::chrono::steady_clock::now();
        
        if (mock_mode_) {
            updateMockPositions();  // 模拟电机响应
        } else {
            // TODO: 调用 Dynamixel SDK
        }
        
        publishJointStates();  // 发布状态到 Topic
        
        // 控制频率
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

void MotorNode::updateMockPositions() {
    double dt = 1.0 / control_rate_;
    
    for (const auto &name : joint_names_) {
        double current = current_positions_[name];
        double target = target_positions_[name];
        
        // 速度限制
        double max_delta = max_speed_ * dt;
        double delta = std::clamp(target - current, -max_delta, max_delta);
        
        current_positions_[name] = current + delta;
        current_velocities_[name] = delta / dt;
    }
}
```

---

### 3.4 模拟模式对比

| 特性 | 原代码 | ROS2 代码 |
|------|--------|-----------|
| **开关方式** | 编译选项 `OPTS->use_debug()` | 运行时参数 `mock_mode` |
| **实现位置** | `virtul_act()` + `SERVER->write()` | `updateMockPositions()` |
| **物理仿真** | 无（直接设置目标位置） | 有（速度限制、平滑插值） |
| **可视化** | TCP 发送给 Server | RViz2 显示 |

#### 原代码模拟
```cpp
void Motor::virtul_act() {
    tcp_command cmd;
    cmd.type = JOINT_DATA;
    
    for (auto &jm : ROBOT->get_joint_map()) {
        jd.id = jm.second->jid_;
        jd.deg = jm.second->get_deg();  // 直接设置，无过渡
        j_data.append((char *)(&jd), sizeof(robot_joint_deg));
    }
    
    SERVER->write(cmd);  // 发送到外部 Server 显示
}
```

#### ROS2 模拟
```cpp
void MotorNode::updateMockPositions() {
    for (const auto &name : joint_names_) {
        double current = current_positions_[name];
        double target = target_positions_[name];
        
        // 速度限制，模拟真实电机响应
        double max_delta = max_speed_ * dt;
        double delta = target - current;
        delta = std::clamp(delta, -max_delta, max_delta);
        
        // 更新位置
        double new_pos = current + delta;
        new_pos = limitJointAngle(name, new_pos);  // 关节限制
        
        current_positions_[name] = new_pos;
        current_velocities_[name] = delta / dt;
        current_efforts_[name] = delta * 0.1;  // 简化力矩模型
    }
}
```

---

## 4. 优缺点对比

### 4.1 原代码优缺点

| 优点 | 缺点 |
|------|------|
| ✅ 直接内存访问，延迟低 | ❌ 强耦合，难以单独测试 |
| ✅ 代码简单，无外部依赖 | ❌ 必须连接真实电机或仿真 Server |
| ✅ 实时性好 | ❌ 无法记录和回放数据 |
| | ❌ 调试困难，需重新编译 |
| | ❌ 无法远程监控 |

### 4.2 ROS2 代码优缺点

| 优点 | 缺点 |
|------|------|
| ✅ 解耦，模块化设计 | ❌ Topic 通信有约 1ms 延迟 |
| ✅ 支持模拟模式，无需硬件 | ❌ 需要学习 ROS2 概念 |
| ✅ 可记录/回放数据（ros2 bag） | ❌ 内存占用稍高 |
| ✅ RViz2 实时可视化 | ❌ 初次配置较复杂 |
| ✅ 远程调试和监控 | |
| ✅ 社区生态丰富 | |

---

## 5. 使用场景对比

### 场景 1：本地调试

**原代码**
```
1. 编译代码
2. 启动仿真 Server
3. 运行程序
4. 无法查看中间状态
```

**ROS2**
```
1. ros2 run motor_driver motor_node
2. ros2 topic echo /joint_states
3. ros2 topic pub /joint_commands ...
4. rviz2 可视化
```

### 场景 2：数据记录

| 原代码 | ROS2 |
|--------|------|
| 无法实现 | `ros2 bag record /joint_states /joint_commands` |

### 场景 3：远程监控

| 原代码 | ROS2 |
|--------|------|
| 需自定义网络协议 | 天然支持，多机 ROS2 通信 |

---

## 6. 迁移建议

### 适合迁移的场景
- ✅ 需要可视化调试
- ✅ 需要数据记录和回放
- ✅ 多模块协作开发
- ✅ 算法验证和仿真

### 保留原代码的场景
- ⚠️ 对实时性要求极高（<1ms）
- ⚠️ 资源受限的嵌入式平台

---

## 7. 总结

ROS2 迁移将 Motor 模块从**紧耦合的内部组件**转变为**独立的服务节点**，带来了更好的模块化、可测试性和可观测性，适合 RoboCup 机器人这样复杂系统的开发和调试。

对于比赛时的最终部署，可以考虑：
- **开发阶段**：使用 ROS2 版本（便于调试）
- **比赛阶段**：如有性能需求，可切换回原版本或使用 ROS2 的实时配置
